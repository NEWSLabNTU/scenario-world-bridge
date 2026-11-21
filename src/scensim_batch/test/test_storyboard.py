"""The compiled storyboard (092 K2.3) on the point-mass engine: SSv2's condition history, its
element state machine, SpeedAction's dynamics, the exits, the refusals by name, the rows the
checker and the executor agree on, and the truth export read back by the gap trace's reader."""
import json
import random
import re
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1] / "scensim_sampler"))
sys.path.insert(0, str(HERE.parents[1] / "sim_vehicle_ecu"))
sys.path.insert(0, str(HERE.parents[1] / "scensim_launch" / "scripts"))

try:
    import numpy as np
    import torch
    from sim_vehicle_ecu import batch as ecu_batch
    from test_core import DT, PointMass, StraightWorld, Accel, cmds
except ImportError:
    torch = None

CASES = HERE.parents[1] / "scensim_sampler" / "examples" / "cases"
DOCS = HERE.parents[2] / "docs" / "design" / "compilable-subset.md"
NIGHT = ("world knob illumination=night",)


def ssv2_condition(history, now, value, delay, edge, current):
    """Condition::update_condition, scalar, line by line (condition.hpp). `history`: a list of
    (time, result), changed in place. Returns the new current_value."""
    history.append((now, value))
    it = next((i for i, (t, _) in enumerate(history) if t > history[-1][0] - delay),
              len(history))
    k = 1 if edge in ("none", "sticky") else 2
    if k <= it:
        a = history[it - 1][1]
        b = history[it - 2][1] if k == 2 else None
        out = {"none": lambda: a, "sticky": lambda: current or a, "rising": lambda: a and not b,
               "falling": lambda: not a and b, "risingOrFalling": lambda: a != b}[edge]()
        del history[:it - k]
        return out
    return False


def case(story, stop="30", params=""):
    """CS2's entities and Init with `story` as its Story and a SimulationTime stop."""
    text = (CASES / "cs2_lead_brakes_wet_night.xosc").read_text()
    text = re.sub(r"<Story .*</Story>", story, text, flags=re.S)
    text = re.sub(r'<SimulationTimeCondition value="30"', f'<SimulationTimeCondition value="{stop}"',
                  text)
    if params:
        text = text.replace("</ParameterDeclarations>", params + "</ParameterDeclarations>")
    d = tempfile.mkdtemp()
    p = Path(d) / "s.xosc"
    p.write_text(text)
    return p


def time_trigger(t, edge="rising"):
    return (f'<StartTrigger><ConditionGroup><Condition name="c{t}" delay="0" '
            f'conditionEdge="{edge}"><ByValueCondition><SimulationTimeCondition value="{t}" '
            f'rule="greaterThan"/></ByValueCondition></Condition></ConditionGroup></StartTrigger>')


def speed(target, shape="step", dim="time", value="0"):
    return (f'<PrivateAction><LongitudinalAction><SpeedAction><SpeedActionDynamics '
            f'dynamicsShape="{shape}" value="{value}" dynamicsDimension="{dim}"/>'
            f'<SpeedActionTarget><AbsoluteTargetSpeed value="{target}"/></SpeedActionTarget>'
            f'</SpeedAction></LongitudinalAction></PrivateAction>')


def story(*events, act_start=time_trigger(0, "none")):
    body = "".join(events)
    return (f'<Story name="main"><Act name="act"><ManeuverGroup name="mg" maximumExecutionCount="1">'
            f'<Actors selectTriggeringEntities="false"><EntityRef entityRef="lead"/></Actors>'
            f'<Maneuver name="m">{body}</Maneuver></ManeuverGroup>{act_start}</Act></Story>')


def event(name, action, trigger, count=1, priority="parallel"):
    return (f'<Event name="{name}" priority="{priority}" maximumExecutionCount="{count}">'
            f'<Action name="{name}_a">{action}</Action>{trigger}</Event>')


def exit_(kind):
    return f'<UserDefinedAction><CustomCommandAction type="{kind}"/></UserDefinedAction>'


@unittest.skipIf(torch is None, "no torch or no sim_vehicle_ecu")
class Conditions(unittest.TestCase):
    def test_the_history_is_ssv2s_per_slot_with_masks_and_delays(self):
        from scensim_batch.storyboard import Condition
        rnd = random.Random(5)
        n, dt = 6, 0.05
        for edge in ("none", "rising", "falling", "risingOrFalling", "sticky"):
            delay = torch.tensor([0.0, 0.0, 0.1, 0.25, 0.05, 1.0], dtype=torch.float64)
            vals = [[rnd.random() < 0.5 for _ in range(n)] for _ in range(80)]
            masks = [[rnd.random() < 0.8 for _ in range(n)] for _ in range(80)]
            step = {"k": 0}

            class Ctx:
                t = torch.zeros(n, dtype=torch.float64)
            ctx = Ctx()
            cond = Condition("c", lambda c: torch.tensor(vals[step["k"]]), edge, delay)
            ref = [([], False) for _ in range(n)]
            for k in range(80):
                step["k"] = k
                ctx.t = torch.full((n,), k * dt, dtype=torch.float64)
                m = torch.tensor(masks[k])
                got = cond.evaluate(ctx, m)
                for s in range(n):
                    if masks[k][s]:
                        h, cur = ref[s]
                        cur = ssv2_condition(h, k * dt, vals[k][s], float(delay[s]), edge, cur)
                        ref[s] = (h, cur)
                        self.assertEqual(bool(got[s]), cur, (edge, k, s))
                    else:
                        self.assertFalse(bool(got[s]))

    def test_rules_are_ssv2s(self):
        from scensim_batch.storyboard import compare, EPS
        # 0.5 + EPS/2 is the next double after 0.5: within epsilon, so equalTo; not exact,
        # so notEqualTo too (SSv2's std::not_equal_to)
        a = torch.tensor([0.5, 0.5 + EPS / 2, 2.0], dtype=torch.float64)
        self.assertNotEqual(float(a[1]), 0.5)
        self.assertEqual(compare("equalTo", a, 0.5).tolist(), [True, True, False])
        self.assertEqual(compare("notEqualTo", a, 0.5).tolist(), [False, True, True])


@unittest.skipIf(torch is None, "no torch or no sim_vehicle_ecu")
class Episodes(unittest.TestCase):
    def build(self, path, recs, **kw):
        from scensim_batch import compile as C, core
        c = C.compile_scenario(path, recs, StraightWorld(), provides=NIGHT)
        c.y[:, c.index("ego")] = 3.5     # beside the lead: the point mass never hits it
        comp = ecu_batch.load_batch(Accel, {}, {}, [1] * len(recs))
        run = core.BatchRun(c, StraightWorld(), PointMass(c.n), comp, route_length=2000.0)
        return c, run

    def test_cs2s_lead_brakes_per_slot_at_its_own_onset_as_ssv2_brakes(self):
        recs = [{"values": {"onset_s": 60.0, "decel": 6.5}}, {"values": {"onset_s": 90.0,
                                                                          "decel": 3.0}}]
        c, run = self.build(CASES / "cs2_lead_brakes_wet_night.xosc", recs)
        lead = c.index("lead")
        run.reset()
        x, v, a, outcome = [], [], [], []
        for _ in range(int(32 / DT)):
            obs = run.step(cmds(2, 0.0), DT)
            x.append(obs["x"][:, lead].clone())
            v.append(obs["v"][:, lead].clone())
            a.append(run.vehicles.a[:, 0].clone())
            outcome.append(run.outcome())
        x, v, a = torch.stack(x), torch.stack(v), torch.stack(a)
        for s, (onset, decel) in enumerate(((60.0, 6.5), (90.0, 3.0))):
            # the first step whose speed falls: the step after the lead came within 1 m
            fall = int(torch.nonzero(v[1:, s] < v[:-1, s] - 1e-9)[0]) + 1
            self.assertLess(abs(float(x[fall - 1, s]) - onset), 1.0 + float(v[fall, s]) * DT)
            self.assertGreaterEqual(float(a[:, s].min()), -decel - 1e-9)   # the limit holds
            self.assertEqual(float(v[-1, s]), 0.0)                          # and it stops
        ev = run.storyboard.by_name("event", "brake")
        from scensim_batch.storyboard import COMPLETE
        self.assertTrue(bool((run.storyboard.state[id(ev)] == COMPLETE).all()))
        # the StopTrigger (t > 30) completes the storyboard; the slots run on, as in SSv2
        sb = run.storyboard
        self.assertEqual(outcome[-1], ["running", "running"])
        self.assertTrue(bool((sb.complete & sb.stopped).all()))
        # the deceleration limit is SSv2's default again once the speed passed the target
        self.assertTrue(bool((run.vehicles.c.max_deceleration == 10.0).all()))

    def test_exit_success_and_failure_end_their_slots_at_their_time(self):
        p = case(story(event("ok", exit_("exitSuccess"), time_trigger("$t_ok")),
                       event("bad", exit_("exitFailure"), time_trigger("$t_bad"))),
                 params='<ParameterDeclaration name="t_ok" parameterType="double" value="1"/>'
                        '<ParameterDeclaration name="t_bad" parameterType="double" value="2"/>')
        recs = [{"values": {"t_ok": 1.0, "t_bad": 2.0}}, {"values": {"t_ok": 3.0, "t_bad": 0.5}}]
        c, run = self.build(p, recs)
        run.reset()
        ends = {}
        for k in range(100):
            run.step(cmds(2, 0.0), DT)
            for s, o in enumerate(run.outcome()):
                if o != "running" and s not in ends:
                    ends[s] = (o, round(float(run.ctx.t[s]), 2))
        # SimulationTime > t: the first tick whose time, summed step by step as SSv2 sums it,
        # is past t (20 x 0.05 is 1.0000000000000002 > 1)
        def first_past(t):
            acc = 0.0
            while not acc > t:
                acc += DT
            return round(acc, 2)
        self.assertEqual(ends, {0: ("success", first_past(1.0)), 1: ("failure", first_past(0.5))})
        # an ended slot's time stands
        self.assertAlmostEqual(float(run.ctx.t[0]), first_past(1.0))

    def test_a_state_condition_latches_and_an_event_reruns_its_actions(self):
        # e1 runs three times (edge none: each tick its trigger holds), stepping the lead's
        # speed; e2 starts on e1's action's startTransition (latched: seen once, true for good)
        trig_e2 = ('<StartTrigger><ConditionGroup><Condition name="s" delay="0" '
                   'conditionEdge="none"><ByValueCondition><StoryboardElementStateCondition '
                   'storyboardElementType="action" storyboardElementRef="e1_a" '
                   'state="startTransition"/></ByValueCondition></Condition></ConditionGroup>'
                   '</StartTrigger>')
        p = case(story(event("e1", speed(7.0), time_trigger(0.5, "none"), count=3),
                       event("e2", speed(9.0), trig_e2)))
        c, run = self.build(p, [{"values": {}}])
        run.reset()
        sb = run.storyboard
        e1, e2 = sb.by_name("event", "e1"), sb.by_name("event", "e2")
        first = {}
        for k in range(30):
            run.step(cmds(1, 0.0), DT)
            for ev in (e1, e2):
                if int(sb.count[id(ev)][0]) and ev.name not in first:
                    first[ev.name] = k
        self.assertEqual(int(sb.count[id(e1)][0]), 3)
        self.assertEqual(int(sb.count[id(e2)][0]), 1)
        self.assertEqual(first["e1"], first["e2"])        # the same tick, after e1's start
        self.assertEqual(float(run.vehicles.target[0, 0]), 9.0)

    def test_an_overriding_event_stops_its_running_sibling(self):
        slow = speed(0.0, "linear", "rate", "-0.5")      # takes many ticks
        p = case(story(event("long", slow, time_trigger(0.2)),
                       event("cut", speed(12.0), time_trigger(1.0), priority="override")))
        c, run = self.build(p, [{"values": {}}])
        run.reset()
        sb = run.storyboard
        from scensim_batch.storyboard import COMPLETE, RUNNING
        long_ = sb.by_name("event", "long")
        states = []
        for _ in range(40):
            run.step(cmds(1, 0.0), DT)
            states.append(int(sb.state[id(long_)][0]))
        self.assertIn(RUNNING, states)
        self.assertEqual(states[-1], COMPLETE)
        self.assertEqual(float(run.vehicles.target[0, 0]), 12.0)

    def test_what_the_executor_does_not_run_is_refused_by_name(self):
        from scensim_batch import compile as C
        with self.assertRaisesRegex(C.CompileError, "LaneChangeAction.*executor pending"):
            C.compile_scenario(CASES / "cs4_lidar_dropout_cutin.xosc", [{"values": {}}],
                               StraightWorld(), provides=NIGHT)
        # and the checker says so too, as a condition the executor does not meet
        from scensim_sampler import subset
        v = subset.check(CASES / "cs4_lidar_dropout_cutin.xosc")
        self.assertTrue(any(c.startswith("LaneChangeAction: batch executor pending")
                            for c in v.conditions), v.conditions)
        # an attribute form it does not run, inside an implemented row
        p = case(story(event("e", speed(5.0, "linear", "rate", "1"),
                             '<StartTrigger><ConditionGroup><Condition name="d" delay="0" '
                             'conditionEdge="rising"><ByEntityCondition><TriggeringEntities '
                             'triggeringEntitiesRule="any"><EntityRef entityRef="ego"/>'
                             '</TriggeringEntities><EntityCondition><RelativeDistanceCondition '
                             'entityRef="lead" relativeDistanceType="longitudinal" '
                             'coordinateSystem="entity" freespace="true" value="5" '
                             'rule="lessThan"/></EntityCondition></ByEntityCondition>'
                             '</Condition></ConditionGroup></StartTrigger>')))
        with self.assertRaisesRegex(C.CompileError, "freespace=true.*executor pending"):
            C.compile_scenario(p, [{"values": {}}], StraightWorld(), provides=NIGHT)

    def test_the_truth_of_each_slot_reads_back_as_a_schema_2_file(self):
        import gap_trace
        recs = [{"values": {"onset_s": 60.0}, "seeds": {"scenario": 11}},
                {"values": {"onset_s": 90.0}, "seeds": {"scenario": 12}}]
        c, run = self.build(CASES / "cs2_lead_brakes_wet_night.xosc", recs)
        c.y[0, c.index("ego")] = 3.5        # slot 0's ego in the next lane: no contact
        c.y[1, c.index("ego")] = 0.0
        c.x[1, c.index("ego")] = 30.0       # slot 1's ego starts just behind the lead
        from scensim_batch.truth import TruthRecorder
        rec = TruthRecorder(run, map_name="straight", step_time=DT, start_unix=0.0)
        rec.begin(run.reset())
        for _ in range(int(31 / DT)):
            rec.record(run.step(cmds(2, 0.0), DT))
        with tempfile.TemporaryDirectory() as d:
            dirs = rec.write(d)
            self.assertEqual(sorted(x.name for x in dirs), ["slot_0_ep0", "slot_1_ep0"])
            for k, x in enumerate(sorted(dirs)):
                tr = gap_trace.load_trace(x / "truth.jsonl")
                self.assertEqual(tr.episode.schema, 2)
                self.assertEqual(set(tr.entities), {e.name for e in c.entities})
                self.assertTrue(tr.end.complete)
                self.assertEqual(json.loads((x / "sample.json").read_text()), recs[k])
                out = json.loads((x / "outcome.json").read_text())
                self.assertEqual((out["outcome"], out["seed"], out["storyboard_stopped"]),
                                 ("running", 11 + k, True))
                f0 = tr.frames[0].entities["ego"].sim_pose
                self.assertAlmostEqual(f0.x, float(c.x[k, c.index("ego")]))
            # slot 1's ego drives into the lead (it starts 6 m behind at 22 m/s): one contact
            t1 = gap_trace.load_trace(sorted(dirs)[1] / "truth.jsonl")
            t0 = gap_trace.load_trace(sorted(dirs)[0] / "truth.jsonl")
            self.assertEqual([x.name for x in t1.contacts][:1], ["lead"])
            self.assertEqual(t0.contacts, [])


class Rows(unittest.TestCase):
    """The checker (S2's table) and the executor never disagree silently (133): every batch
    row and batch command is either run by the executor or pending, never both, and the
    pending ones are listed in compilable-subset.md."""

    def test_every_batch_row_is_implemented_or_pending(self):
        try:
            from scensim_batch.storyboard import IMPLEMENTED
        except ImportError:
            self.skipTest("no torch")
        from scensim_sampler import subset
        rows = {k for k, (cls, *_) in subset.TABLE.items() if cls == subset.BATCH}
        rows |= {k for k, (cls, *_) in subset.COMMANDS.items() if cls == subset.BATCH}
        self.assertEqual(sorted(rows - IMPLEMENTED - subset.EXECUTOR_PENDING), [])
        self.assertEqual(sorted(IMPLEMENTED & subset.EXECUTOR_PENDING), [])
        self.assertEqual(sorted((IMPLEMENTED | subset.EXECUTOR_PENDING) - rows), [])
        text = DOCS.read_text()
        section = re.split(r"^## (?:\d+\. )?Executor pending$", text, maxsplit=1, flags=re.M)
        self.assertEqual(len(section), 2, "compilable-subset.md has no 'Executor pending' section")
        missing = [k for k in subset.EXECUTOR_PENDING if f"`{k}`" not in section[1]]
        self.assertEqual(missing, [])


_ = np
