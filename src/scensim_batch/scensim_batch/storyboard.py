"""The compiled storyboard (roadmap 092 K2.3; batch-executor.md section 6): a scenario's Story,
Act, ManeuverGroup, Maneuver, Event and Action elements as one state machine per slot, their
triggers as per-slot boolean masks, their actions on the batch's entities.

It ports SSv2's interpreter (openscenario_interpreter), not the OpenSCENARIO text where the two
differ, since a batch episode must replay on seats:

- an element's evaluation is `StoryboardElement::evaluate`: its stop trigger first (an override),
  then the dispatch: standby -> (its start trigger) start -> running -> (its children
  accomplished) end -> standby again while executions remain, else complete; a stop transition
  overrides and evaluates every child, then completes. All in one tick, re-dispatching after an
  end, as SSv2's `goto dispatch` does;
- a condition is `Condition::update_condition`: every evaluation appends (time, value) to its
  history; the value used is the newest entry at least `delay` old, an edge compares it with
  the one before; a trigger evaluated only when its element is (a start trigger in standby)
  appends only then;
- `equalTo` is |a - b| < machine epsilon, `notEqualTo` exact; standstill is |v| <= epsilon;
- a lane-coordinate longitudinal or lateral relative distance is signed (SSv2's "historical
  reasons"); an entity-coordinate one is absolute;
- a StoryboardElementStateCondition latches: once its element has passed through the state, it
  stays true (SSv2's transition callback);
- a SpeedAction: step sets the target and the speed at once and ends; linear (and cubic and
  sinusoidal, which SSv2 maps to linear) at a rate sets the acceleration to the rate and the
  acceleration or deceleration limit to its magnitude until the speed passes the target, then
  resets the limits; at a time, the rate is (target - v) / time, unless the target is already
  within 0.01; a distance dimension only sets the target; it ends when the speed equals the
  target (to machine epsilon);
- exitSuccess and exitFailure end the slot with that outcome. Nothing else ends it: SSv2 keeps
  evaluating a storyboard that completed or was stopped, and only the test runner's timeout
  (a Failure, "Timeout") ends such a run; `complete` and `stopped` say which slots are there.

What it does not run is refused by name when it is compiled (`StoryboardError`), and listed in
`scensim_sampler.subset.EXECUTOR_PENDING` for the checker. IMPLEMENTED is the rows it runs.

A tick evaluates the whole storyboard for every live slot, at the slot's simulation time, on
the state the last step left (SSv2 evaluates the storyboard, then updates the entities).
"""
import math

import torch

from . import npc

IMPLEMENTED = frozenset("""
AbsoluteTargetSpeed RelativeTargetSpeed SpeedAction TeleportAction
AccelerationCondition CollisionCondition DistanceCondition ParameterCondition
ReachPositionCondition RelativeDistanceCondition RelativeSpeedCondition
SimulationTimeCondition SpeedCondition StandStillCondition StoryboardElementStateCondition
WorldPosition LanePosition RelativeWorldPosition RelativeObjectPosition
exitSuccess exitFailure :
""".split())
# the ego's routing, compiled into the slot's route goal (the policy's task), not executed;
# another entity's routing is executor pending (scensim_sampler.subset.EGO_GOAL)
EGO_GOAL = frozenset(("AcquirePositionAction", "AssignRouteAction"))

EPS = 2.220446049250313e-16          # std::numeric_limits<double>::epsilon()
REACHED = 0.01                       # LongitudinalSpeedPlanner::isTargetSpeedReached's tolerance

STANDBY, RUNNING, COMPLETE, STOPPING = 0, 1, 2, 3
STATES = {"standbyState": STANDBY, "runningState": RUNNING, "completeState": COMPLETE}
TRANSITIONS = ("startTransition", "endTransition", "stopTransition")
OUTCOMES = ("running", "success", "failure")


class StoryboardError(ValueError):
    pass


def _pending(where, what):
    return StoryboardError(f"{where}: {what} (batch executor pending)")


# --- rules ----------------------------------------------------------------------------------

def compare(rule, a, b, where="rule"):
    if rule == "greaterThan":
        return a > b
    if rule == "lessThan":
        return a < b
    if rule == "greaterOrEqual":
        return a >= b
    if rule == "lessOrEqual":
        return a <= b
    if rule == "equalTo":
        return (a - b).abs() < EPS
    if rule == "notEqualTo":
        return a != b
    raise StoryboardError(f"{where}: rule {rule!r}")


# --- conditions -----------------------------------------------------------------------------

class Condition:
    """One Condition: `fn(ctx) -> [N] bool`, its edge and delay [N], its history per slot."""

    def __init__(self, name, fn, edge, delay):
        if edge not in ("none", "rising", "falling", "risingOrFalling", "sticky"):
            raise StoryboardError(f"Condition {name!r}: conditionEdge {edge!r}")
        self.name, self.fn, self.edge, self.delay = name, fn, edge, delay
        self.reset_all(1)

    def reset_all(self, n):
        self.t, self.val, self.ok = [], [], []      # history: [N] tensors per entry
        self.current = torch.zeros(n, dtype=torch.bool)

    def reset(self, mask):
        self.ok = [o & ~mask for o in self.ok]
        self.current = self.current & ~mask

    def evaluate(self, ctx, mask):
        """The condition for the slots in `mask` (others keep their value and history)."""
        raw = self.fn(ctx).to(torch.bool)
        n = raw.shape[0]
        if self.current.shape[0] != n:
            self.reset_all(n)
        self.t.append(ctx.t.clone())
        self.val.append(raw)
        self.ok.append(mask.clone())
        T, V, O = torch.stack(self.t), torch.stack(self.val), torch.stack(self.ok)   # [H, N]
        # the entries before SSv2's iterator: time <= now - delay
        old = O & (T <= ctx.t - self.delay)
        k = 1 if self.edge in ("none", "sticky") else 2
        count = old.sum(0)
        rank = torch.cumsum(old.to(torch.int64), 0)                      # 1-based among `old`
        a_sel = old & (rank == count)                                     # newest old entry
        b_sel = old & (rank == count - 1)
        a = (V & a_sel).any(0)
        b = (V & b_sel).any(0)
        enough = count >= k
        if self.edge == "none":
            out = a
        elif self.edge == "sticky":
            out = self.current | a
        elif self.edge == "rising":
            out = a & ~b
        elif self.edge == "falling":
            out = ~a & b
        else:
            out = a != b
        out = enough & out
        # SSv2 erases the entries before the oldest it used
        keep_from = torch.where(enough, count - (k - 1), torch.zeros_like(count))
        self.ok = list(O & ~(old & (rank < keep_from)))
        while self.ok and not bool(self.ok[0].any()):
            self.t.pop(0), self.val.pop(0), self.ok.pop(0)
        self.current = torch.where(mask, out, self.current)
        return self.current & mask


class Trigger:
    """ConditionGroups: OR of ANDs, every condition evaluated (SSv2 evaluates them all)."""

    def __init__(self, groups):
        self.groups = groups

    def evaluate(self, ctx, mask):
        out = torch.zeros_like(mask)
        for g in self.groups:
            acc = mask.clone()
            for c in g:
                acc = acc & c.evaluate(ctx, mask)
            out = out | acc
        return out & mask

    def conditions(self):
        return [c for g in self.groups for c in g]


# --- the context ----------------------------------------------------------------------------

class Context:
    """What the conditions and actions read: every entity's state [N, E] after the last step."""

    def __init__(self, run):
        self.run = run
        c = run.c
        self.n, self.e = c.n, len(c.entities)
        self.t = torch.zeros(self.n, dtype=torch.float64)
        self.standstill = torch.zeros(self.n, self.e, dtype=torch.float64)
        self.obs = None

    def update(self, obs, dt):
        self.obs = obs
        v = obs["v"]
        still = v.abs() <= EPS
        self.standstill = torch.where(still, self.standstill + dt, torch.zeros_like(v))

    def col(self, name, i):
        return self.obs[name][:, i]

    def accel(self, i):
        run = self.run
        if i == run.ego:
            return self.obs["ego"]["accel_x"]
        if i in run.defaults:
            return run.vehicles.a[:, run.defaults.index(i)]
        return torch.zeros(self.n, dtype=torch.float64)


# --- the storyboard -------------------------------------------------------------------------

class Node:
    def __init__(self, kind, name, children=(), start=None, stop=None, max_exec=1,
                 priority=None, action=None):
        self.kind, self.name = kind, name
        self.children = list(children)
        self.start, self.stop = start, stop
        self.max_exec, self.priority, self.action = max_exec, priority, action
        self.parent = None
        for ch in self.children:
            ch.parent = self


class Storyboard:
    """The state machine of one storyboard over N slots."""

    def __init__(self, root, n, latches):
        self.root, self.n = root, n
        self.nodes = []
        self._index(root)
        self.state = {id(x): torch.zeros(n, dtype=torch.int8) for x in self.nodes}
        self.count = {id(x): torch.zeros(n, dtype=torch.int64) for x in self.nodes}
        self.latches = latches      # (node id, state name) -> [Latch]
        self.outcome = torch.zeros(n, dtype=torch.int8)     # OUTCOMES index
        self.stopped = torch.zeros(n, dtype=torch.bool)     # the storyboard's StopTrigger fired
        self.ctx = None

    def _index(self, x):
        self.nodes.append(x)
        for ch in x.children:
            self._index(ch)

    def by_name(self, kind, name):
        for x in self.nodes:
            if x.kind == kind and x.name == name:
                return x
        return None

    def conditions(self):
        out = []
        for x in self.nodes:
            for t in (x.start, x.stop):
                if t is not None:
                    out += t.conditions()
        return out

    # --- per slot ---------------------------------------------------------------------------

    def reset(self, mask):
        for x in self.nodes:
            self.state[id(x)] = torch.where(mask, torch.zeros_like(self.state[id(x)]),
                                            self.state[id(x)])
            self.count[id(x)] = torch.where(mask, torch.zeros_like(self.count[id(x)]),
                                            self.count[id(x)])
            if x.action is not None:
                x.action.reset(mask)
        for c in self.conditions():
            c.reset(mask)
        for ls in self.latches.values():
            for latch in ls:
                latch.reset(mask)
        self.outcome = torch.where(mask, torch.zeros_like(self.outcome), self.outcome)
        self.stopped = self.stopped & ~mask

    def tick(self, ctx):
        """Evaluate the storyboard once for every live slot."""
        self.ctx = ctx
        live = self.outcome == 0
        if bool(live.any()):
            self.evaluate(self.root, live)
        return self.outcome

    @property
    def complete(self):
        """[N]: the storyboard reached its completeState (all stories done, or stopped)."""
        return self.state[id(self.root)] == COMPLETE

    def end_slot(self, mask, outcome):
        mask = mask & (self.outcome == 0)
        self.outcome = torch.where(mask, torch.full_like(self.outcome, OUTCOMES.index(outcome)),
                                   self.outcome)

    # --- SSv2's StoryboardElement::evaluate, per slot ----------------------------------------

    def _emit(self, x, state, mask):
        for latch in self.latches.get((id(x), state), ()):
            latch.seen = latch.seen | mask

    def _set(self, x, value, mask):
        self.state[id(x)] = torch.where(mask, torch.full_like(self.state[id(x)], value),
                                        self.state[id(x)])

    def override(self, x, mask):
        st = self.state[id(x)]
        m = mask & (st != STANDBY) & (st != STOPPING)
        if bool(m.any()):
            self._set(x, STOPPING, m)
            self._emit(x, "stopTransition", m)
            if x is self.root:
                self.stopped = self.stopped | m

    def accomplished(self, x, mask):
        if x.action is not None:
            return x.action.done & mask
        out = mask.clone()
        for ch in x.children:
            out = out & (self.state[id(ch)] == COMPLETE)
        return out

    def evaluate(self, x, mask):
        mask = mask & (self.outcome == 0)
        if x.stop is not None and bool(mask.any()):
            self.override(x, x.stop.evaluate(self.ctx, mask))
        m = mask
        # an end re-dispatches the slots that went back to standby: SSv2 loops as long as the
        # element accomplishes at once, at most its executions (243); one pass more ends it
        for _ in range(max(1, x.max_exec) + 1):
            if not bool(m.any()):
                return
            st = self.state[id(x)]
            sb = m & (st == STANDBY)
            if bool(sb.any()):
                go = sb if x.start is None else x.start.evaluate(self.ctx, sb)
                if bool(go.any()):
                    self._start(x, go)
            run = m & (self.state[id(x)] == RUNNING) & (self.outcome == 0)
            again = torch.zeros_like(m)
            if bool(run.any()):
                if x.action is not None:
                    x.action.run(self, run)
                else:
                    for ch in x.children:
                        self.evaluate(ch, run)
                run = run & (self.outcome == 0)
                acc = self.accomplished(x, run)
                if bool(acc.any()):
                    self._emit(x, "endTransition", acc)
                    more = acc & (self.count[id(x)] < x.max_exec)
                    self._set(x, STANDBY, more)
                    self._set(x, COMPLETE, acc & ~more)
                    self._emit(x, "standbyState", more)
                    self._emit(x, "completeState", acc & ~more)
                    again = more
            stp = m & (self.state[id(x)] == STOPPING)
            if bool(stp.any()):
                notacc = stp & ~self.accomplished(x, stp)
                if bool(notacc.any()):
                    if x.action is not None:
                        x.action.stop(self, notacc)
                    for ch in x.children:
                        self.override(ch, notacc)
                        self.evaluate(ch, notacc)
                self._set(x, COMPLETE, stp)
                self._emit(x, "completeState", stp)
            m = again
        raise StoryboardError(f"{x.kind} {x.name!r}: more than its {x.max_exec} executions in "
                              "one tick")

    def _start(self, x, go):
        """startTransition, start(), runningState (and the execution counted)."""
        self._emit(x, "startTransition", go)
        if x.kind == "event":
            if x.priority == "override" and x.parent is not None:
                # Maneuver::overrideEvents: every event of the maneuver, this one too (in its
                # startTransition, so its stopTransition fires; it goes running all the same);
                # the others are evaluated when the maneuver next runs them
                self._emit(x, "stopTransition", go)
                for sib in x.parent.children:
                    if sib is not x:
                        self.override(sib, go)
            for ch in x.children:     # Event::start: its actions to startTransition, run again
                self._start(ch, go)
        if x.action is not None:
            x.action.start(self, go)
        self._set(x, RUNNING, go)
        self.count[id(x)] = torch.where(go, self.count[id(x)] + 1, self.count[id(x)])
        self._emit(x, "runningState", go)


class Latch:
    """StoryboardElementStateCondition's current_state, per slot: seen once, true for good."""

    def __init__(self, n):
        self.seen = torch.zeros(n, dtype=torch.bool)

    def reset(self, mask):
        self.seen = self.seen & ~mask


# --- actions --------------------------------------------------------------------------------

class Action:
    def __init__(self, n):
        self.done = torch.zeros(n, dtype=torch.bool)

    def reset(self, mask):
        self.done = self.done & ~mask

    def start(self, sb, go):
        self.done = self.done & ~go

    def run(self, sb, mask):
        pass

    def stop(self, sb, mask):
        pass


class Instant(Action):
    """An action that ends as it starts: EnvironmentAction (its rendering is a condition the
    environment declared), the no-op command, an Init-only form."""

    def start(self, sb, go):
        self.done = self.done | go


class Exit(Instant):
    def __init__(self, n, outcome):
        super().__init__(n)
        self.outcome = outcome

    def start(self, sb, go):
        super().start(sb, go)
        sb.end_slot(go, self.outcome)


class GoalAction(Instant):
    """The ego's AcquirePositionAction or AssignRouteAction: the slot's route goal from now
    (BatchRun.route), for the policy; it ends as it starts."""

    def __init__(self, n, waypoints):
        super().__init__(n)
        self.waypoints = waypoints          # [(x [N], y [N], yaw [N])]

    def start(self, sb, go):
        super().start(sb, go)
        sb.ctx.run.route.set(go, self.waypoints)


class SpeedAction(Action):
    """SSv2's SpeedAction on default vehicles (`actors`: their columns in BatchRun.vehicles)."""

    def __init__(self, n, actors, entity_cols, shape, dimension, value, target):
        super().__init__(n)
        self.actors, self.cols = actors, entity_cols
        self.shape, self.dimension, self.value = shape, dimension, value
        self.target = target                     # fn(ctx) -> [N] absolute target at start
        self.goal = torch.zeros(n, dtype=torch.float64)
        self.reached = torch.zeros(n, len(actors), dtype=torch.bool)

    def reset(self, mask):
        super().reset(mask)
        self.reached = self.reached & ~mask[:, None]

    def start(self, sb, go):
        super().start(sb, go)
        run = sb.ctx.run
        veh = run.vehicles
        goal = self.target(sb.ctx)
        self.goal = torch.where(go, goal, self.goal)
        self.reached = self.reached & ~go[:, None]
        transition = "step" if self.shape == "step" else "linear"
        for a in self.actors:
            m = go[:, None] & (torch.arange(veh.v.shape[1]) == a)[None, :]
            if not bool(m.any()):
                continue
            tgt = goal[:, None].expand_as(veh.v)
            v = veh.v
            if self.dimension == "distance":      # Constraint::Type::NONE: the target only
                veh.set_target_speed(tgt, m)
                continue
            if self.dimension == "time":
                reached = (tgt - v).abs() <= REACHED
                m = m & ~reached
                t = self.value[:, None].expand_as(v)
                instant = t.abs() <= EPS
                step = m & (instant | (transition == "step"))
                lin = m & ~step
                rate = torch.where(lin, (tgt - v) / torch.where(lin, t, torch.ones_like(t)),
                                   torch.zeros_like(v))
                lin = lin & ~((tgt - v).abs() <= REACHED)
            else:                                  # rate
                step = m & (transition == "step")
                lin = m & ~step
                rate = self.value[:, None].expand_as(v)
            if bool(step.any()):
                veh.set_target_speed(tgt, step)
                veh.v = torch.where(step, tgt, veh.v)
            if bool(lin.any()):
                veh.a = torch.where(lin, rate, veh.a)
                up = lin & (v <= tgt)
                down = lin & ~up & (v >= tgt)
                veh.c.max_acceleration = torch.where(up, rate.abs(), veh.c.max_acceleration)
                veh.c.max_deceleration = torch.where(down, rate.abs(), veh.c.max_deceleration)
                run.limit_jobs(up, down, tgt)
                veh.set_target_speed(tgt, lin)

    def run(self, sb, mask):
        if self.shape == "step":                   # SpeedAction::endsImmediately
            self.done = self.done | mask
            return
        veh = sb.ctx.run.vehicles
        v = veh.v[:, self.actors]
        hit = (self.goal[:, None] - v).abs() < EPS
        self.reached = self.reached | (hit & mask[:, None])
        self.done = self.done | (mask & self.reached.all(-1))


# --- compiling ------------------------------------------------------------------------------

def compile_storyboard(sb_el, ctx_info):
    """The Storyboard of a scenario's <Storyboard> element. `ctx_info`: the compile step's
    parameters, entities, world and position resolver (compile._StoryInfo)."""
    return _Compiler(ctx_info).storyboard(sb_el)


class _Compiler:
    def __init__(self, info):
        self.i = info
        self.n = info.n
        self.latches = {}
        self.pending_latches = []        # (Latch, kind, name, state, where), bound at the end
        self.names = {}

    # elements
    def storyboard(self, el):
        stories = [self.story(s, "/Storyboard") for s in el.findall("Story")]
        stop = self.trigger(el.find("StopTrigger"), "/Storyboard/StopTrigger")
        root = Node("storyboard", "storyboard", stories, stop=stop)
        sb = Storyboard(root, self.n, self.latches)
        for latch, kind, name, state, where in self.pending_latches:
            x = sb.by_name(kind, name)
            if x is None:
                raise StoryboardError(f"{where}: no {kind} named {name!r}")
            self.latches.setdefault((id(x), state), []).append(latch)
        return sb

    def story(self, el, where):
        here = f"{where}/Story[@name='{el.get('name')}']"
        if el.find("ParameterDeclarations/ParameterDeclaration") is not None:
            raise _pending(here, "a Story's own ParameterDeclarations")
        acts = [self.act(a, here) for a in el.findall("Act")]
        return Node("story", el.get("name"), acts)

    def act(self, el, where):
        here = f"{where}/Act[@name='{el.get('name')}']"
        mgs = [self.maneuver_group(m, here) for m in el.findall("ManeuverGroup")]
        start = self.trigger(el.find("StartTrigger"), here + "/StartTrigger")
        stop = self.trigger(el.find("StopTrigger"), here + "/StopTrigger")
        return Node("act", el.get("name"), mgs, start=start, stop=stop)

    def maneuver_group(self, el, where):
        here = f"{where}/ManeuverGroup[@name='{el.get('name')}']"
        actors = el.find("Actors")
        if actors is not None and str(actors.get("selectTriggeringEntities")).lower() == "true":
            raise _pending(here, "selectTriggeringEntities=true")
        refs = [r.get("entityRef") for r in (actors.findall("EntityRef")
                                             if actors is not None else [])]
        if el.find("CatalogReference") is not None:
            raise _pending(here, "a Maneuver from a catalog")
        mans = [self.maneuver(m, here, refs) for m in el.findall("Maneuver")]
        return Node("maneuverGroup", el.get("name"), mans,
                    max_exec=self.int_(el.get("maximumExecutionCount", "1"), here))

    def maneuver(self, el, where, actors):
        here = f"{where}/Maneuver[@name='{el.get('name')}']"
        events = [self.event(e, here, actors) for e in el.findall("Event")]
        return Node("maneuver", el.get("name"), events)

    def event(self, el, where, actors):
        here = f"{where}/Event[@name='{el.get('name')}']"
        pr = el.get("priority", "parallel")
        pr = {"overwrite": "override"}.get(pr, pr)
        if pr not in ("override", "parallel"):
            raise _pending(here, f"priority {pr!r} (SSv2: skip is unimplemented)")
        acts = [self.action(a, here, actors) for a in el.findall("Action")]
        start = self.trigger(el.find("StartTrigger"), here + "/StartTrigger")
        return Node("event", el.get("name"), acts, start=start,
                    max_exec=self.int_(el.get("maximumExecutionCount", "1"), here), priority=pr)

    def action(self, el, where, actors):
        here = f"{where}/Action[@name='{el.get('name')}']"
        return Node("action", el.get("name"), action=self.action_body(el, here, actors))

    def int_(self, text, where):
        v = self.i.P.floats(text, where)
        if not bool((v == v[0]).all()):
            raise _pending(where, "a maximumExecutionCount that differs between slots")
        return int(v[0])

    # actions
    def action_body(self, el, here, actors):
        n = self.n
        g = el.find("GlobalAction")
        if g is not None:
            if g.find("EnvironmentAction") is not None:
                return Instant(n)
            raise _pending(here, f"GlobalAction/{next(iter(g)).tag}")
        u = el.find("UserDefinedAction/CustomCommandAction")
        if u is not None:
            t = (u.get("type") or "").strip()
            key = t.split("(")[0].strip()
            if key == "exitSuccess":
                return Exit(n, "success")
            if key == "exitFailure":
                return Exit(n, "failure")
            if key == ":":
                return Instant(n)
            raise _pending(here, f"CustomCommandAction {key}")
        p = el.find("PrivateAction")
        if p is None:
            raise _pending(here, next(iter(el)).tag if len(el) else "an empty Action")
        ra = p.find("RoutingAction")
        if ra is not None:
            kind = next(iter(ra)).tag
            if kind in EGO_GOAL and actors and all(
                    self.i.entities[self.i.index(a, here)].role == "ego" for a in actors):
                return GoalAction(n, self.waypoints(next(iter(ra)), here + "/RoutingAction/"
                                                    + kind, story=True))
            raise _pending(here, f"RoutingAction/{kind}" + (" on the ego" if kind ==
                                                            "FollowTrajectoryAction" else ""))
        sp = p.find("LongitudinalAction/SpeedAction")
        if sp is not None:
            return self.speed_action(sp, here + "/PrivateAction/LongitudinalAction/SpeedAction",
                                     actors)
        first = next(iter(p), None)
        inner = next(iter(first), None) if first is not None else None
        raise _pending(here, "/".join(x.tag for x in (first, inner) if x is not None)
                       + " in the story")

    def speed_action(self, el, here, actors):
        i, P = self.i, self.i.P
        cols = []
        for a in actors:
            idx = i.index(a, here)
            role = i.entities[idx].role
            if role != "default":
                raise _pending(here, f"a SpeedAction on {a!r}, a {role} entity (it acts on "
                                     "SSv2's default vehicles)")
            cols.append(i.defaults.index(idx))
        d = el.find("SpeedActionDynamics")
        if d is None:
            raise StoryboardError(f"{here}: no SpeedActionDynamics")
        shape, dim = d.get("dynamicsShape"), d.get("dynamicsDimension")
        if shape not in ("linear", "cubic", "sinusoidal", "step"):
            raise StoryboardError(f"{here}: dynamicsShape {shape!r}")
        if dim not in ("rate", "time", "distance"):
            raise StoryboardError(f"{here}: dynamicsDimension {dim!r}")
        value = P.floats(d.get("value"), here + "/SpeedActionDynamics@value")
        tgt = el.find("SpeedActionTarget")
        ab = tgt.find("AbsoluteTargetSpeed") if tgt is not None else None
        rel = tgt.find("RelativeTargetSpeed") if tgt is not None else None
        if ab is not None:
            speed = P.floats(ab.get("value"), here + "/.../AbsoluteTargetSpeed@value")
            target = lambda ctx: speed                       # noqa: E731
        elif rel is not None:
            if str(rel.get("continuous")).lower() == "true":
                raise _pending(here, "a continuous RelativeTargetSpeed")
            ref = i.index(rel.get("entityRef"), here)
            val = P.floats(rel.get("value"), here + "/.../RelativeTargetSpeed@value")
            kind = rel.get("speedTargetValueType")
            if kind not in ("delta", "factor"):
                raise StoryboardError(f"{here}: speedTargetValueType {kind!r}")
            target = ((lambda ctx: ctx.col("v", ref) + val) if kind == "delta"
                      else (lambda ctx: ctx.col("v", ref) * val))
        else:
            raise StoryboardError(f"{here}: no SpeedActionTarget")
        return SpeedAction(self.n, cols, actors, shape, dim, value, target)

    def position(self, pos, where):
        """A story-time position: world or lane coordinates. One relative to an entity would be
        resolved at compile time against the entity's Init pose, not where it is when the
        condition or action runs: refused."""
        if pos is not None and next(iter(pos), None) is not None and \
                next(iter(pos)).tag.startswith("Relative"):
            raise _pending(where, f"a {next(iter(pos)).tag} at story time")
        return self.i.position(pos, where)

    def waypoints(self, el, where, story):
        return routing_waypoints(el, where, self.position if story else self.i.position)

    # triggers and conditions
    def trigger(self, el, where):
        if el is None:
            return None
        groups = []
        for k, g in enumerate(el.findall("ConditionGroup")):
            groups.append([self.condition(c, f"{where}/ConditionGroup[{k + 1}]")
                           for c in g.findall("Condition")])
        if not groups:
            return Trigger([[Condition("never", lambda ctx: torch.zeros(ctx.n, dtype=torch.bool),
                                       "none", torch.zeros(self.n, dtype=torch.float64))]])
        return Trigger(groups)

    def condition(self, el, where):
        name = el.get("name")
        here = f"{where}/Condition[@name='{name}']"
        delay = self.i.P.floats(el.get("delay", "0"), here + "@delay")
        bv, be = el.find("ByValueCondition"), el.find("ByEntityCondition")
        if bv is not None:
            fn = self.by_value(next(iter(bv)), here + "/ByValueCondition")
        elif be is not None:
            fn = self.by_entity(be, here + "/ByEntityCondition")
        else:
            raise StoryboardError(f"{here}: neither ByValueCondition nor ByEntityCondition")
        return Condition(name, fn, el.get("conditionEdge", "none"), delay)

    def by_value(self, c, here):
        P = self.i.P
        here = f"{here}/{c.tag}"
        rule = c.get("rule")
        if c.tag == "SimulationTimeCondition":
            value = P.floats(c.get("value"), here + "@value")
            return lambda ctx: compare(rule, ctx.t, value, here)
        if c.tag == "ParameterCondition":
            name = c.get("parameterRef")
            vals = P.value("$" + name, here + "@parameterRef")
            ref = P.value(c.get("value"), here + "@value")
            nums = [(_num(a), _num(b)) for a, b in zip(vals, ref)]
            if all(a is not None and b is not None for a, b in nums):
                a = torch.tensor([x for x, _ in nums], dtype=torch.float64)
                b = torch.tensor([y for _, y in nums], dtype=torch.float64)
                res = compare(rule, a, b, here)
            elif rule in ("equalTo", "notEqualTo"):
                eq = torch.tensor([str(x) == str(y) for x, y in zip(vals, ref)])
                res = eq if rule == "equalTo" else ~eq
            else:
                raise StoryboardError(f"{here}: rule {rule!r} on a string parameter")
            return lambda ctx: res     # a parameter is fixed for the slot (no mid-run set)
        if c.tag == "StoryboardElementStateCondition":
            kind = c.get("storyboardElementType")
            state = c.get("state")
            if state not in STATES and state not in TRANSITIONS:
                raise _pending(here, f"state {state!r}")
            latch = Latch(self.n)
            self.pending_latches.append((latch, kind, c.get("storyboardElementRef"), state,
                                         here))
            code = STATES.get(state)

            def fn(ctx, latch=latch, code=code, kind=kind, ref=c.get("storyboardElementRef")):
                sb = ctx.storyboard
                if code is not None:
                    x = sb.by_name(kind, ref)
                    latch.seen = latch.seen | (sb.state[id(x)] == code)
                return latch.seen
            return fn
        raise _pending(here, c.tag)

    def by_entity(self, be, here):
        i = self.i
        te = be.find("TriggeringEntities")
        rule = te.get("triggeringEntitiesRule")
        if rule not in ("any", "all"):
            raise StoryboardError(f"{here}: triggeringEntitiesRule {rule!r}")
        ents = [i.index(r.get("entityRef"), here) for r in te.findall("EntityRef")]
        c = next(iter(be.find("EntityCondition")))
        per = self.entity_condition(c, f"{here}/EntityCondition/{c.tag}")

        def fn(ctx):
            vals = torch.stack([per(ctx, e) for e in ents], -1)
            return vals.any(-1) if rule == "any" else vals.all(-1)
        return fn

    def entity_condition(self, c, here):
        i, P = self.i, self.i.P
        rule = c.get("rule")
        tag = c.tag
        if tag in ("SpeedCondition", "AccelerationCondition", "RelativeSpeedCondition") and \
                c.get("direction") is not None:
            raise _pending(here, "a direction")
        if tag == "SpeedCondition":
            value = P.floats(c.get("value"), here + "@value")
            return lambda ctx, e: compare(rule, ctx.col("v", e), value, here)
        if tag == "AccelerationCondition":
            value = P.floats(c.get("value"), here + "@value")
            return lambda ctx, e: compare(rule, ctx.accel(e), value, here)
        if tag == "RelativeSpeedCondition":
            ref = i.index(c.get("entityRef"), here)
            value = P.floats(c.get("value"), here + "@value")
            return lambda ctx, e: compare(rule, ctx.col("v", e) - ctx.col("v", ref), value, here)
        if tag == "StandStillCondition":
            dur = P.floats(c.get("duration"), here + "@duration")
            return lambda ctx, e: compare("greaterThan", ctx.standstill[:, e], dur, here)
        if tag == "ReachPositionCondition":
            tol = P.floats(c.get("tolerance"), here + "@tolerance")
            px, py, _ = self.position(c.find("Position"), here + "/Position")
            return lambda ctx, e: compare("lessThan", torch.hypot(
                ctx.col("x", e) - px, ctx.col("y", e) - py), tol, here)
        if tag in ("RelativeDistanceCondition", "DistanceCondition"):
            return self.distance(c, here, rule)
        if tag == "CollisionCondition":
            return self.collision(c, here)
        raise _pending(here, tag)

    def distance(self, c, here, rule):
        i, P = self.i, self.i.P
        if str(c.get("freespace")).lower() == "true":
            raise _pending(here, "freespace=true")
        if (c.get("routingAlgorithm") or "undefined") != "undefined":
            raise _pending(here, f"routingAlgorithm {c.get('routingAlgorithm')!r}")
        cs = c.get("coordinateSystem") or "entity"
        kind = c.get("relativeDistanceType") or "euclidianDistance"
        value = P.floats(c.get("value"), here + "@value")
        if c.tag == "RelativeDistanceCondition":
            ref = i.index(c.get("entityRef"), here)
            other = lambda ctx: (ctx.col("x", ref), ctx.col("y", ref))     # noqa: E731
        else:
            if kind != "euclidianDistance" and kind != "cartesianDistance":
                raise _pending(here, f"a DistanceCondition by {kind}")
            px, py, _ = self.position(c.find("Position"), here + "/Position")
            other = lambda ctx: (px, py)                                    # noqa: E731
        if kind in ("euclidianDistance", "cartesianDistance"):
            def euclid(ctx, e):
                ox, oy = other(ctx)
                return torch.hypot(ox - ctx.col("x", e), oy - ctx.col("y", e))
            dist = euclid
        elif cs == "entity":
            def entity(ctx, e):
                ox, oy = other(ctx)
                dx, dy = ox - ctx.col("x", e), oy - ctx.col("y", e)
                h = ctx.col("yaw", e)
                lon = torch.cos(h) * dx + torch.sin(h) * dy
                lat = -torch.sin(h) * dx + torch.cos(h) * dy
                return (lon if kind == "longitudinal" else lat).abs()
            dist = entity
        elif cs == "lane":
            st = getattr(i.world, "lane_st", None)
            if st is None:
                raise _pending(here, "coordinateSystem=lane on a world adapter without lane_st")

            def lane(ctx, e):     # signed: SSv2's historical lane-coordinate distance
                ox, oy = other(ctx)
                s0, t0 = st(ctx.col("x", e), ctx.col("y", e))
                s1, t1 = st(ox, oy)
                return (s1 - s0) if kind == "longitudinal" else (t1 - t0)
            dist = lane
        else:
            raise _pending(here, f"coordinateSystem {cs!r}")
        return lambda ctx, e: compare(rule, dist(ctx, e), value, here)

    def collision(self, c, here):
        i = self.i
        er = c.find("EntityRef")
        bt = c.find("ByType")
        if er is not None:
            targets = [i.index(er.get("entityRef"), here)]
        elif bt is not None:
            want = bt.get("type")
            kinds = {"vehicle": "vehicle", "pedestrian": "pedestrian", "miscellaneous": "misc"}
            targets = [k for k, e in enumerate(i.entities) if e.kind == kinds.get(want)]
        else:
            raise StoryboardError(f"{here}: neither EntityRef nor ByType")

        def fn(ctx, e):
            hit = torch.zeros(ctx.n, dtype=torch.bool)
            for t in targets:
                if t != e:
                    hit = hit | overlap(ctx, e, t)
            return hit
        return fn


def routing_waypoints(el, where, position):
    """An AcquirePositionAction's position or an AssignRouteAction's waypoints, each
    `(x, y, yaw)` [N] through `position(pos_el, where)`."""
    if el.tag == "AcquirePositionAction":
        return [position(el.find("Position"), where + "/Position")]
    if el.find("CatalogReference") is not None:
        raise _pending(where, "a Route from a catalog")
    wps = el.findall("Route/Waypoint")
    if not wps:
        raise StoryboardError(f"{where}: a Route without Waypoints")
    return [position(w.find("Position"), f"{where}/Route/Waypoint[{k + 1}]/Position")
            for k, w in enumerate(wps)]


class RouteGoal:
    """The slot's route goal (the ego's routing, the policy's task): the route polyline
    [N, P, 2] from where the ego was when it was given to the goal, the goal pose [N, 3], and per
    step the ego's progress along it, the distance left, the distance to the goal and whether
    it has been reached (within `tolerance`, latched). The route is the world adapter's
    (`route_through(x, y, waypoints [N, W, 2], points)`) when it has one, else the chords
    through the waypoints."""

    def __init__(self, run, points=128, tolerance=1.0):
        n = run.c.n
        self.run, self.p, self.tolerance = run, points, tolerance
        z = torch.zeros(n, dtype=torch.float64)
        self.has = torch.zeros(n, dtype=torch.bool)
        self.reached = torch.zeros(n, dtype=torch.bool)
        self.points = torch.zeros(n, points, 2, dtype=torch.float64)
        self.goal = torch.zeros(n, 3, dtype=torch.float64)
        self.progress, self.remaining, self.to_goal = z.clone(), z.clone(), z.clone()

    def reset(self, mask):
        self.has = self.has & ~mask
        self.reached = self.reached & ~mask

    def set(self, mask, waypoints):
        if not bool(mask.any()):
            return
        run = self.run
        obs = run.obs
        e = run.ego
        x0 = obs["x"][:, e] if obs is not None else run.c.x[:, e]
        y0 = obs["y"][:, e] if obs is not None else run.c.y[:, e]
        wp = torch.stack([torch.stack([w[0], w[1]], -1) for w in waypoints], 1)   # [N, W, 2]
        through = getattr(run.world, "route_through", None)
        if through is not None:
            pts = torch.as_tensor(through(x0, y0, wp, self.p), dtype=torch.float64)
        else:
            pts = _resample(torch.cat([torch.stack([x0, y0], -1)[:, None], wp], 1), self.p)
        self.points = torch.where(mask[:, None, None], pts, self.points)
        last = waypoints[-1]
        g = torch.stack([last[0], last[1], last[2]], -1)
        self.goal = torch.where(mask[:, None], g, self.goal)
        self.has = self.has | mask
        self.reached = self.reached & ~mask

    def update(self, obs):
        """The ego's progress, from the observation; returns the route's block of it."""
        e = self.run.ego
        x, y = obs["x"][:, e], obs["y"][:, e]
        r = npc.Routes(self.points[:, None])
        s, _ = r.project(torch.stack([x, y], -1)[:, None, None])
        s = s[:, 0, 0]
        self.progress = torch.where(self.has, s, torch.zeros_like(s))
        self.remaining = torch.where(self.has, r.length[:, 0] - s, torch.zeros_like(s))
        self.to_goal = torch.where(self.has, torch.hypot(self.goal[:, 0] - x, self.goal[:, 1] - y),
                                   torch.zeros_like(s))
        now = self.has & ~self.reached & (self.to_goal <= self.tolerance)
        self.reached = self.reached | now
        return {"has_goal": self.has.clone(), "points": self.points, "goal": self.goal,
                "progress": self.progress, "remaining": self.remaining,
                "to_goal": self.to_goal, "goal_reached": self.reached.clone(),
                "goal_reached_now": now}


def _resample(poly, p):
    """A polyline [N, K, 2] as `p` points evenly spaced along its arc length [N, P, 2]."""
    n = poly.shape[0]
    pts = poly[:, None].expand(n, p, *poly.shape[1:])        # one route per sample point
    r = npc.Routes(pts)
    want = torch.linspace(0, 1, p, dtype=torch.float64)[None] * r.length[:, :1]
    x, y, _ = r.pose(want)
    return torch.stack([x, y], -1)


def _num(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def overlap(ctx, a, b):
    """Whether entities a and b's boxes overlap, per slot (separating axes)."""
    run = ctx.run
    ents = run.c.entities
    pa = _corners(ctx, a, ents[a])
    pb = _corners(ctx, b, ents[b])
    sep = torch.zeros(ctx.n, dtype=torch.bool)
    for poly in (pa, pb):
        for k in range(4):
            e = poly[:, (k + 1) % 4] - poly[:, k]
            ax = torch.stack([-e[:, 1], e[:, 0]], -1)                  # [N, 2]
            ra = (pa * ax[:, None]).sum(-1)
            rb = (pb * ax[:, None]).sum(-1)
            sep = sep | (ra.max(-1).values < rb.min(-1).values) | \
                (rb.max(-1).values < ra.min(-1).values)
    return ~sep


def _corners(ctx, i, ent):
    x, y, h = ctx.col("x", i), ctx.col("y", i), ctx.col("yaw", i)
    c, s = torch.cos(h), torch.sin(h)
    cx = x + c * ent.cx - s * ent.cy
    cy = y + s * ent.cx + c * ent.cy
    return npc.box_corners(cx, cy, h, torch.full_like(x, ent.length),
                           torch.full_like(x, ent.width))


_ = math
