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

    def test_an_instant_event_runs_all_its_executions_in_one_tick(self):
        # an exit-free instant action under a held trigger, 100 executions: SSv2's goto
        # re-dispatches it within one tick until its count is spent (243's review)
        p = case(story(event("many", exit_(":"), time_trigger(0.0, "none"), count=100)))
        c, run = self.build(p, [{"values": {}}])
        run.reset()
        run.step(cmds(1, 0.0), DT)
        ev = run.storyboard.by_name("event", "many")
        self.assertEqual(int(run.storyboard.count[id(ev)][0]), 100)

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
        with self.assertRaisesRegex(C.CompileError, "scensim.fault: batch executor pending"):
            C.compile_scenario(CASES / "cs4_lidar_dropout_cutin.xosc", [{"values": {}}],
                               StraightWorld(), provides=NIGHT)
        # and the checker says so too, as a condition the executor does not meet; the lane
        # change itself runs now
        from scensim_sampler import subset
        v = subset.check(CASES / "cs4_lidar_dropout_cutin.xosc")
        self.assertTrue(any(c.startswith("CustomCommandAction scensim.fault: batch executor "
                                         "pending") for c in v.conditions), v.conditions)
        self.assertFalse(any("LaneChangeAction" in c for c in v.conditions), v.conditions)
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


def acquire(entity_s):
    return ('<PrivateAction><RoutingAction><AcquirePositionAction><Position><LanePosition '
            f'roadId="" laneId="34513" s="{entity_s}" offset="0"/></Position>'
            '</AcquirePositionAction></RoutingAction></PrivateAction>')


def with_init(path, entity, actions):
    text = Path(path).read_text().replace(f'<Private entityRef="{entity}">',
                                          f'<Private entityRef="{entity}">{actions}', 1)
    Path(path).write_text(text)
    return path


@unittest.skipIf(torch is None, "no torch or no sim_vehicle_ecu")
class RouteGoal(unittest.TestCase):
    """The ego's routing under --ego policy: the policy's task, the slot's route goal (133)."""

    def build(self, path, recs):
        from scensim_batch import compile as C, core
        c = C.compile_scenario(path, recs, StraightWorld(), provides=NIGHT)
        c.y[:, c.index("ego")] = 3.5
        comp = ecu_batch.load_batch(Accel, {}, {}, [1] * len(recs))
        return c, core.BatchRun(c, StraightWorld(), PointMass(c.n), comp, route_length=2000.0)

    def test_an_init_acquire_position_is_each_slots_route_goal_with_progress(self):
        p = with_init(case(story(event("e", exit_(":"), time_trigger(100)))), "ego",
                      acquire("$goal_s"))
        p.write_text(p.read_text().replace("</ParameterDeclarations>", '<ParameterDeclaration '
                     'name="goal_s" parameterType="double" value="80"/></ParameterDeclarations>'))
        recs = [{"values": {"goal_s": 80.0, "ego_speed": 10.0}},
                {"values": {"goal_s": 150.0, "ego_speed": 10.0}}]
        c, run = self.build(p, recs)
        obs = run.reset()
        r = obs["route"]
        self.assertTrue(bool(r["has_goal"].all()))
        self.assertEqual(r["goal"][:, 0].tolist(), [80.0, 150.0])
        # the chord from the ego (5, 3.5) to the goal (80, 0)
        self.assertAlmostEqual(float(r["remaining"][0]), (75.0 ** 2 + 3.5 ** 2) ** 0.5, places=6)
        reached_at = {}
        for k in range(200):
            obs = run.step(cmds(2, 0.0), DT)           # 10 m/s, no acceleration
            r = obs["route"]
            if k == 39:    # 2 s in, 20 m along: the progress is the ego's projection on the chord
                ex, ey = float(obs["x"][0, c.index("ego")]), 3.5
                ux, uy = 75.0, -3.5
                along = ((ex - 5.0) * ux + (ey - 3.5) * uy) / (ux ** 2 + uy ** 2) ** 0.5
                self.assertAlmostEqual(float(r["progress"][0]), along, delta=1e-6)
            for s in range(2):
                if bool(r["goal_reached_now"][s]):
                    self.assertNotIn(s, reached_at)    # fires once
                    reached_at[s] = float(obs["x"][s, c.index("ego")])
        # reached within the tolerance (1 m) of the goal; the ego's lateral offset (3.5 m)
        # keeps slot 0 from it until the goal is checked in the ego's lane: hence lane y = 3.5
        self.assertEqual(sorted(reached_at), [])       # 3.5 m beside the goal: never within 1 m
        self.assertAlmostEqual(float(r["remaining"][0]), 0.0, delta=1e-6)   # past the route's end

    def test_reached_latches_in_the_goals_lane(self):
        p = with_init(case(story(event("e", exit_(":"), time_trigger(100)))), "ego", acquire(60))
        c, run = self.build(p, [{"values": {"ego_speed": 10.0}}])
        c.y[:, c.index("ego")] = 0.0                    # in the goal's lane
        c.y[:, c.index("lead")] = 7.0                   # the lead out of the way
        run.reset()
        hits = []
        for k in range(120):
            obs = run.step(cmds(1, 0.0), DT)
            if bool(obs["route"]["goal_reached_now"][0]):
                hits.append(float(obs["x"][0, c.index("ego")]))
        self.assertEqual(len(hits), 1)
        self.assertLessEqual(abs(hits[0] - 60.0), 1.0)
        self.assertTrue(bool(obs["route"]["goal_reached"][0]))      # latched past the goal

    def test_a_story_assign_route_gives_the_goal_when_its_event_starts(self):
        route = ('<PrivateAction><RoutingAction><AssignRouteAction><Route name="r" closed="false">'
                 '<Waypoint routeStrategy="shortest"><Position><WorldPosition x="40" y="10"/>'
                 '</Position></Waypoint><Waypoint routeStrategy="shortest"><Position>'
                 '<WorldPosition x="90" y="10" h="0"/></Position></Waypoint></Route>'
                 '</AssignRouteAction></RoutingAction></PrivateAction>')
        st = story(event("go", route, time_trigger(1.0))).replace(
            '<EntityRef entityRef="lead"/></Actors>', '<EntityRef entityRef="ego"/></Actors>')
        c, run = self.build(case(st), [{"values": {}}])
        obs = run.reset()
        self.assertFalse(bool(obs["route"]["has_goal"][0]))
        for _ in range(25):
            obs = run.step(cmds(1, 0.0), DT)
        r = obs["route"]
        self.assertTrue(bool(r["has_goal"][0]))
        self.assertEqual(r["goal"][0].tolist(), [90.0, 10.0, 0.0])
        # the chord route passes through the first waypoint
        d = (r["points"][0] - torch.tensor([40.0, 10.0], dtype=torch.float64)).norm(dim=-1)
        self.assertLess(float(d.min()), 1.0)

    def test_the_checker_and_compile_agree_on_who_is_routed(self):
        from scensim_batch import compile as C
        from scensim_sampler import subset
        ego = with_init(case(story(event("e", exit_(":"), time_trigger(1)))), "ego", acquire(60))
        v = subset.check(ego)
        self.assertTrue(v.compiles)
        self.assertFalse(any("AcquirePositionAction" in x for x in v.conditions), v.conditions)
        npc_ = with_init(case(story(event("e", exit_(":"), time_trigger(1)))), "lead", acquire(60))
        v = subset.check(npc_)
        self.assertTrue(any(x.startswith("AcquirePositionAction: batch executor pending")
                            for x in v.conditions), v.conditions)
        with self.assertRaisesRegex(C.CompileError, "AcquirePositionAction: batch executor pending"):
            C.compile_scenario(npc_, [{"values": {}}], StraightWorld(), provides=NIGHT)
        traj = ('<PrivateAction><RoutingAction><FollowTrajectoryAction><TrajectoryRef/>'
                '</FollowTrajectoryAction></RoutingAction></PrivateAction>')
        st = story(event("t", traj, time_trigger(1))).replace(
            '<EntityRef entityRef="lead"/></Actors>', '<EntityRef entityRef="ego"/></Actors>')
        v = subset.check(case(st))
        self.assertFalse(v.compiles)
        self.assertEqual(v.refusals[0]["construct"], "FollowTrajectoryAction on the ego")
        self.assertIn("overriding the batch's policy", v.refusals[0]["reason"])

    def test_a_story_time_position_relative_to_an_entity_is_refused(self):
        from scensim_batch import compile as C
        cond = ('<StartTrigger><ConditionGroup><Condition name="r" delay="0" conditionEdge="rising">'
                '<ByEntityCondition><TriggeringEntities triggeringEntitiesRule="any"><EntityRef '
                'entityRef="ego"/></TriggeringEntities><EntityCondition><ReachPositionCondition '
                'tolerance="1"><Position><RelativeObjectPosition entityRef="lead" dx="-5" dy="0"/>'
                '</Position></ReachPositionCondition></EntityCondition></ByEntityCondition>'
                '</Condition></ConditionGroup></StartTrigger>')
        with self.assertRaisesRegex(C.CompileError, "RelativeObjectPosition at story time"):
            C.compile_scenario(case(story(event("e", exit_(":"), cond))), [{"values": {}}],
                               StraightWorld(), provides=NIGHT)


@unittest.skipIf(torch is None, "no torch or no sim_vehicle_ecu")
class ScenarioTime(unittest.TestCase):
    """SSv2's scenario time (SimulationClock, also in the fork under clock_source:=simulator,
    which moves ROS time only): A(S + k) - A(S), A the step summed in a double."""

    def ssv2(self, s, k, dt=DT):
        a = 0.0
        for _ in range(s):
            a += dt
        a_s = a
        for _ in range(k):
            a += dt
        return a - a_s

    def test_the_batch_time_is_ssv2s_for_any_start_frame(self):
        from scensim_batch import compile as C, core
        for s in (0, 1, 7, 113):
            c = C.compile_scenario(case(story(event("e", exit_(":"), time_trigger(100)))),
                                   [{"values": {}}], StraightWorld(), provides=NIGHT)
            run = core.BatchRun(c, StraightWorld(), PointMass(1),
                                ecu_batch.load_batch(Accel, {}, {}, [1]), start_frames=s,
                                step_time=DT)
            run.reset()
            for k in range(1, 61):
                run.step(cmds(1, 0.0), DT)
                self.assertEqual(float(run.ctx.t[0]), self.ssv2(s, k), (s, k))

    def test_which_frame_passes_a_boundary_depends_on_the_start_frame(self):
        # what K2.4 has to carry from a seats run: t > 1 is first true at frame 20 or 21
        firsts = {s: next(k for k in range(1, 40) if self.ssv2(s, k) > 1.0) for s in range(40)}
        self.assertEqual(set(firsts.values()) - {20, 21}, set())
        self.assertGreater(len(set(firsts.values())), 1, firsts)


def ssv2_hermite_length(p0, p1, v0, v1, n=100):
    """HermiteCurve(start, goal, start_vec, goal_vec).getLength(100), scalar, as written."""
    ax, bx, cx = 2 * p0[0] - 2 * p1[0] + v0[0] + v1[0], -3 * p0[0] + 3 * p1[0] - 2 * v0[0] - v1[0], v0[0]
    ay, by, cy = 2 * p0[1] - 2 * p1[1] + v0[1] + v1[1], -3 * p0[1] + 3 * p1[1] - 2 * v0[1] - v1[1], v0[1]
    ds, ret = 1.0 / n, 0.0
    for i in range(n):
        s = i * ds
        xd = (3 * s * s) * ax + 2 * s * bx + cx
        yd = (3 * s * s) * ay + 2 * s * by + cy
        ret = ret + (xd * xd + yd * yd) ** 0.5 * ds
    return ret


def lane_change(target='<AbsoluteTargetLane value="34513"/>', shape="sinusoidal", dim="time",
                value="$lc_t"):
    return ('<PrivateAction><LateralAction><LaneChangeAction><LaneChangeActionDynamics '
            f'dynamicsShape="{shape}" value="{value}" dynamicsDimension="{dim}"/>'
            f'<LaneChangeTarget>{target}</LaneChangeTarget></LaneChangeAction></LateralAction>'
            '</PrivateAction>')


@unittest.skipIf(torch is None, "no torch or no sim_vehicle_ecu")
class LaneChange(unittest.TestCase):
    """SSv2's LaneChangeAction (FORCE): the Hermite curve, its forced speed, the end."""

    def build(self, action, recs, start=1.0):
        from scensim_batch import compile as C, core
        p = case(story(event("lc", action, time_trigger(start))),
                 params='<ParameterDeclaration name="lc_t" parameterType="double" value="3"/>')
        # the lead one lane to the left (34514, y = 3.5), cutting back into 34513
        text = p.read_text()
        i = text.index('<Private entityRef="lead">')
        p.write_text(text[:i] + text[i:].replace('laneId="34513"', 'laneId="34514"', 1))
        c = C.compile_scenario(p, recs, StraightWorld(), provides=NIGHT)
        c.y[:, c.index("ego")] = -7.0        # the ego two lanes away
        comp = ecu_batch.load_batch(Accel, {}, {}, [1] * len(recs))
        return c, core.BatchRun(c, StraightWorld(), PointMass(c.n), comp, route_length=2000.0)

    def run_until(self, run, c, seconds):
        lead = c.index("lead")
        ys, vs = [], []
        for _ in range(int(round(seconds / DT))):
            obs = run.step(cmds(c.n, 0.0), DT)
            ys.append(obs["y"][:, lead].clone())
            vs.append(obs["v"][:, lead].clone())
        return torch.stack(ys), torch.stack(vs)

    def test_a_timed_cut_in_takes_its_time_per_slot_and_ends_on_the_target_lane(self):
        recs = [{"values": {"lc_t": 3.0}}, {"values": {"lc_t": 5.0}}]
        c, run = self.build(lane_change(), recs)
        run.reset()
        ys, vs = self.run_until(run, c, 12.0)
        veh = run.vehicles
        for k, T in enumerate((3.0, 5.0)):
            moving = torch.nonzero((ys[:, k] < 3.5 - 1e-9) & (ys[:, k] > 1e-9)).flatten()
            first, last = int(moving[0]), int(moving[-1])
            # FORCE: the curve's length at length / T, so it ends ceil(T / dt) steps in
            self.assertLessEqual(abs((last - first + 2) * DT - T), DT + 1e-9, (k, first, last))
            self.assertEqual(float(ys[-1, k]), 0.0)                 # on 34513, offset 0
            during = vs[first + 1:last, k]
            self.assertTrue(bool(((during - during[0]).abs() < 1e-12).all()))
        self.assertFalse(bool(veh.lc.any()))
        # the event (and its action) completed the tick it started, as in SSv2
        ev = run.storyboard.by_name("event", "lc")
        from scensim_batch.storyboard import COMPLETE
        self.assertTrue(bool((run.storyboard.state[id(ev)] == COMPLETE).all()))

    def test_the_curve_and_its_length_are_ssv2s(self):
        c, run = self.build(lane_change(), [{"values": {"lc_t": 4.0}}], start=0.5)
        run.reset()
        lead = c.index("lead")
        for _ in range(int(round(0.6 / DT))):
            run.step(cmds(1, 0.0), DT)
        veh = run.vehicles
        self.assertTrue(bool(veh.lc[0, 0]))
        # from the lead's pose at the start, 20 m along to the crossing of lane 34513
        coef = veh.lc_coef[0, 0]
        p0 = coef[3].tolist()
        goal = (p0[0] + 20.0, 0.0)
        size = ((goal[0] - p0[0]) ** 2 + (goal[1] - p0[1]) ** 2) ** 0.5 * 0.5
        want = ssv2_hermite_length(p0, goal, (size, 0.0), (size, 0.0))
        self.assertAlmostEqual(float(veh.lc_len[0, 0]), want, places=12)
        self.assertAlmostEqual(float(veh.lc_v[0, 0]), want / 4.0, places=12)
        self.assertEqual(p0[1], 3.5)
        _ = lead

    def test_a_linear_change_by_distance_keeps_the_speed_on_the_chord(self):
        c, run = self.build(lane_change(shape="linear", dim="distance", value="30"),
                            [{"values": {}}])
        run.reset()
        self.run_until(run, c, 1.0)
        veh = run.vehicles
        v_before = float(veh.v[0, 0])
        ys, vs = self.run_until(run, c, 0.1)
        self.assertTrue(bool(veh.lc[0, 0]))
        self.assertAlmostEqual(float(veh.lc_len[0, 0]), (30.0 ** 2 + 3.5 ** 2) ** 0.5, places=9)
        self.assertAlmostEqual(float(vs[-1, 0]), v_before, places=9)

    def test_a_relative_target_counts_lanes_from_its_entity(self):
        target = '<RelativeTargetLane entityRef="lead" value="-1"/>'
        c, run = self.build(lane_change(target=target), [{"values": {"lc_t": 2.0}}])
        run.reset()
        self.run_until(run, c, 6.0)
        self.assertEqual(run.vehicles.lc_to_lane[0][0], 34513)

    def test_the_egos_lane_change_is_refused_under_policy(self):
        from scensim_sampler import subset
        st = story(event("e", lane_change(), time_trigger(1))).replace(
            '<EntityRef entityRef="lead"/></Actors>', '<EntityRef entityRef="ego"/></Actors>')
        v = subset.check(case(st))
        self.assertFalse(v.compiles)
        self.assertEqual(v.refusals[0]["construct"], "LaneChangeAction on the ego")


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
        # the ego's routing is the policy's goal; another entity's is still pending
        from scensim_batch.storyboard import EGO_GOAL
        self.assertEqual(EGO_GOAL, subset.EGO_GOAL)
        self.assertTrue(EGO_GOAL <= subset.EXECUTOR_PENDING)
        text = DOCS.read_text()
        section = re.split(r"^## (?:\d+\. )?Executor pending$", text, maxsplit=1, flags=re.M)
        self.assertEqual(len(section), 2, "compilable-subset.md has no 'Executor pending' section")
        missing = [k for k in subset.EXECUTOR_PENDING if f"`{k}`" not in section[1]]
        self.assertEqual(missing, [])


_ = np
