"""SSv2's default vehicle behaviour, batched over N slots of E vehicles each, in torch (roadmap
092 K2.1; docs/design/batch-executor.md section 2).

What a vehicle with no controller does in SSv2 (behavior_tree_plugin's
vehicle_entity_behavior.xml): it follows its lane at the target speed its SpeedActions set
(else the lane's speed limit), and when an entity is close ahead on its path it follows that
entity, slowing to its speed or stopping. Its speed comes from SSv2's jerk-limited planner
(traffic_simulator's LongitudinalSpeedPlanner). This module ports both:

- the planner (`plan_linear_jerk`, `dynamic_states`, `running_distance`), line for line;
- FollowLane's front-entity check and FollowFrontEntity's speed rule, with SSv2's constants
  (front_entity_stop_margin 5 m, front_entity_margin 5 m, speed_step 2 m/s, the horizon
  clamp(5 v, 20, 50) m, the +-90 degree front cone);
- the move: arc length advanced by the mean of the old and new speed times the step.

Not ported (yet): stop lines, traffic lights, crossing and conflicting entities, yield, right
of way, lane changes and moving backward. A route with any of them is outside this model, and
the executor's compile step refuses it until they are added (section 2 lists them).

The path is a piecewise-linear centreline per vehicle (SSv2 uses a Catmull-Rom spline through
the same lanelet centreline): positions and headings differ from SSv2's by the spline's
deviation from the chords, which the agreement measurement bounds.

Shapes: N slots, E vehicles, P route points, M other entities (the ego and anything not driven
here). Every tensor is float64 on one device; nothing loops over slots in Python.
"""
import math

import torch

FRONT_ENTITY_STOP_MARGIN = 5.0   # FollowLaneAction::front_entity_stop_margin
FRONT_ENTITY_MARGIN = 5.0        # FollowFrontEntityAction::front_entity_margin
SPEED_STEP = 2.0                 # FollowFrontEntityAction::speed_step
TWIST_TOLERANCE = 0.01           # LongitudinalSpeedPlanner::getRunningDistance
# traffic_simulator_msgs/DynamicConstraints defaults
DEFAULT_CONSTRAINTS = {"max_acceleration": 10.0, "max_acceleration_rate": 3.0,
                       "max_deceleration": 10.0, "max_deceleration_rate": 3.0, "max_speed": 50.0}


class Constraints:
    """DynamicConstraints per vehicle: five [N, E] tensors."""

    def __init__(self, max_acceleration, max_acceleration_rate, max_deceleration,
                 max_deceleration_rate, max_speed):
        self.max_acceleration = max_acceleration
        self.max_acceleration_rate = max_acceleration_rate
        self.max_deceleration = max_deceleration
        self.max_deceleration_rate = max_deceleration_rate
        self.max_speed = max_speed

    @classmethod
    def default(cls, shape, device=None):
        return cls(**{k: torch.full(shape, v, dtype=torch.float64, device=device)
                      for k, v in DEFAULT_CONSTRAINTS.items()})


# --- the planner: LongitudinalSpeedPlanner, vectorised ------------------------------------

def plan_linear_jerk(target, v, a, c, dt):
    accelerating = v <= target
    down_lo = torch.maximum(-c.max_deceleration, (target - v) / dt)
    # std::clamp(x, lo, hi) as min(max(x, lo), hi), in that order
    up = torch.minimum(torch.maximum(a + dt * c.max_acceleration_rate, torch.zeros_like(a)),
                       torch.minimum(c.max_acceleration, (target - v) / dt))
    down = torch.minimum(torch.maximum(a - dt * c.max_deceleration_rate, down_lo),
                         torch.zeros_like(a))
    new = torch.where(accelerating, up, down)
    return (new - a) / dt


def dynamic_states(target, c, v, a, dt):
    """`(v, a, jerk)` after one step towards `target` (getDynamicStates)."""
    target = torch.minimum(target, c.max_speed)
    jerk = plan_linear_jerk(target, v, a, c, dt)
    a1 = torch.minimum(torch.maximum(a + dt * jerk, -c.max_deceleration), c.max_acceleration)
    v1 = torch.minimum(torch.maximum(v + a1 * dt, -c.max_speed), c.max_speed)
    a1 = (v1 - v) / dt
    jerk = (a1 - a) / dt
    return v1, a1, jerk


def running_distance(target, c, v, a, jerk, dt, max_steps=100000):
    """The distance to reach `target` (getRunningDistance): the planner stepped until every
    vehicle is within 0.01 m/s of it, each vehicle stopping its sum when it is."""
    done = (target - v).abs() <= TWIST_TOLERANCE
    out = torch.zeros_like(v)
    for _ in range(max_steps):
        if bool(done.all()):
            return out
        v1, a1, j1 = dynamic_states(target, c, v, a, dt)
        step = v1 * dt + a1 * dt * dt / 2.0 + j1 * dt * dt * dt / 6.0
        out = torch.where(done, out, out + step)
        v, a = torch.where(done, v, v1), torch.where(done, a, a1)
        done = done | ((target - v).abs() <= TWIST_TOLERANCE)
    raise RuntimeError(f"running_distance: not reached in {max_steps} steps")


# --- routes ---------------------------------------------------------------------------------

class Routes:
    """One centreline per vehicle: points [N, E, P, 2] (a short route repeats its last point),
    with cumulative arc length [N, E, P]."""

    def __init__(self, points):
        self.points = points
        seg = (points[..., 1:, :] - points[..., :-1, :]).norm(dim=-1)
        self.s = torch.cat([torch.zeros_like(seg[..., :1]), seg.cumsum(-1)], dim=-1)
        self.length = self.s[..., -1]

    def pose(self, s):
        """`(x, y, yaw)` at arc length `s` [N, E] (clamped to the route)."""
        s = torch.minimum(torch.maximum(s, torch.zeros_like(s)), self.length)
        i = torch.searchsorted(self.s, s.unsqueeze(-1), right=True).squeeze(-1) - 1
        i = i.clamp(0, self.s.shape[-1] - 2)
        p0 = self.points.gather(-2, i[..., None, None].expand(*i.shape, 1, 2)).squeeze(-2)
        p1 = self.points.gather(-2, (i + 1)[..., None, None].expand(*i.shape, 1, 2)).squeeze(-2)
        s0 = self.s.gather(-1, i[..., None]).squeeze(-1)
        s1 = self.s.gather(-1, (i + 1)[..., None]).squeeze(-1)
        span = (s1 - s0)
        f = torch.where(span > 0, (s - s0) / torch.where(span > 0, span, torch.ones_like(span)),
                        torch.zeros_like(span))
        xy = p0 + (p1 - p0) * f[..., None]
        d = p1 - p0
        yaw = torch.atan2(d[..., 1], d[..., 0])
        return xy[..., 0], xy[..., 1], yaw

    def project(self, xy):
        """Points [N, E, K, 2] onto each vehicle's own route: `(s, lateral distance)` [N, E, K]."""
        a, b = self.points[..., :-1, :], self.points[..., 1:, :]          # [N, E, P-1, 2]
        ab = b - a
        L2 = (ab * ab).sum(-1).clamp_min(1e-18)
        q = xy[..., :, None, :]                                           # [N, E, K, 1, 2]
        t = (((q - a[..., None, :, :]) * ab[..., None, :, :]).sum(-1) / L2[..., None, :]
             ).clamp(0, 1)                                                # [N, E, K, P-1]
        foot = a[..., None, :, :] + t[..., None] * ab[..., None, :, :]
        dist = (q - foot).norm(dim=-1)
        j = dist.argmin(-1)                                               # [N, E, K]
        s0 = self.s[..., :-1][..., None, :].expand_as(dist).gather(-1, j[..., None]).squeeze(-1)
        segl = L2.sqrt()[..., None, :].expand_as(dist).gather(-1, j[..., None]).squeeze(-1)
        tj = t.gather(-1, j[..., None]).squeeze(-1)
        return s0 + tj * segl, dist.gather(-1, j[..., None]).squeeze(-1)


def box_corners(x, y, yaw, length, width):
    """The four corners [..., 4, 2] of boxes centred at (x, y)."""
    c, s = torch.cos(yaw), torch.sin(yaw)
    hl, hw = length / 2, width / 2
    out = []
    for sl, sw in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
        dx, dy = sl * hl, sw * hw
        out.append(torch.stack([x + c * dx - s * dy, y + s * dx + c * dy], -1))
    return torch.stack(out, -2)


# --- the behaviour --------------------------------------------------------------------------

class DefaultVehicles:
    """E vehicles per slot driven by SSv2's default behaviour (FollowLane, FollowFrontEntity).

    `routes`: Routes; `s0`, `v0` [N, E] start arc length and speed; `length`, `width` [N, E]
    their boxes; `speed_limit` [N, E] (the route's, used when no SpeedAction has set a target);
    `active` [N, E] bool (padding off). `constraints`: Constraints, default SSv2's."""

    def __init__(self, routes, s0, v0, length, width, speed_limit, active=None,
                 constraints=None, see_around=None):
        self.routes = routes
        self.s, self.v = s0.clone(), v0.clone()
        self.a, self.jerk = torch.zeros_like(s0), torch.zeros_like(s0)
        self.length, self.width, self.speed_limit = length, width, speed_limit
        self.active = torch.ones_like(s0, dtype=torch.bool) if active is None else active
        self.c = constraints or Constraints.default(s0.shape, s0.device)
        self.see_around = (torch.ones_like(self.active) if see_around is None else see_around)
        self.target = torch.full_like(s0, math.nan)   # NaN: no SpeedAction yet
        self.mode = torch.zeros_like(s0, dtype=torch.int8)   # 0 follow lane, 1 follow front

    def set_target_speed(self, speed, mask):
        """A SpeedAction with a step to its target: mask [N, E]."""
        self.target = torch.where(mask, speed, self.target)

    def poses(self):
        return self.routes.pose(self.s)

    def front(self, others=None):
        """Distance to the front entity on each vehicle's path, inf where none: SSv2's
        getDistanceToFrontEntity (the cone, the horizon, splineDistanceToBoundingBox)."""
        x, y, yaw = self.poses()
        n, e = x.shape
        ents = [(x, y, yaw, self.length, self.width, self.active)]
        if others is not None:
            ents.append((others["x"], others["y"], others["yaw"], others["length"],
                         others["width"], others.get("active",
                                                     torch.ones_like(others["x"], dtype=torch.bool))))
        X = torch.cat([t[0] for t in ents], 1)           # [N, K]
        Y = torch.cat([t[1] for t in ents], 1)
        YAW = torch.cat([t[2] for t in ents], 1)
        LEN = torch.cat([t[3] for t in ents], 1)
        WID = torch.cat([t[4] for t in ents], 1)
        ACT = torch.cat([t[5] for t in ents], 1)
        k = X.shape[1]
        # the cone: |angle of (other - self) against self's yaw| <= pi/2
        dx = X[:, None, :] - x[:, :, None]
        dy = Y[:, None, :] - y[:, :, None]
        vy = torch.atan2(dy, dx) - yaw[:, :, None]
        in_cone = torch.atan2(torch.sin(vy), torch.cos(vy)).abs() <= math.pi / 2
        self_mask = torch.zeros(n, e, k, dtype=torch.bool, device=x.device)
        self_mask[:, torch.arange(e), torch.arange(e)] = True
        # along own path: the other's centre, and its box's distance to the path
        centre = torch.stack([X, Y], -1)[:, None].expand(n, e, k, 2)
        s_other, _ = self.routes.project(centre)
        corners = box_corners(X, Y, YAW, LEN, WID)               # [N, K, 4, 2]
        cs, cd = self.routes.project(corners[:, None].expand(n, e, k, 4, 2).reshape(n, e, k * 4, 2))
        lateral = cd.reshape(n, e, k, 4).min(-1).values
        horizon = torch.clamp(self.v * 5.0, 20.0, 50.0)
        stop = running_distance(torch.zeros_like(self.v), self.c, self.v, self.a, self.jerk,
                                self.dt)
        horizon = torch.maximum(horizon, stop)
        ds = s_other - self.s[:, :, None]
        gap = ds - self.length[:, :, None] / 2 - LEN[:, None, :] / 2
        dist = gap + self.length[:, :, None] / 2     # SSv2 adds half its own length back
        ok = (ACT[:, None, :] & self.active[:, :, None] & ~self_mask & in_cone & (ds > 0)
              & (gap >= 0) & (lateral <= self.width[:, :, None] / 2)
              & (ds <= horizon[:, :, None] + LEN[:, None, :]))
        dist = torch.where(ok, dist, torch.full_like(dist, math.inf))
        best, idx = dist.min(-1)
        V = torch.cat([self.v] + ([others["v"]] if others is not None else []), 1)
        v_front = V.gather(1, idx.clamp(max=k - 1))
        return best, v_front, stop

    def step(self, dt, others=None):
        """One step of every vehicle. `others`: {x, y, yaw, length, width, v, active} [N, M]."""
        self.dt = dt
        target = torch.where(torch.isnan(self.target), self.speed_limit, self.target)
        d, v_front, stop = self.front(others)
        L = self.length
        # FollowLane fails, and FollowFrontEntity takes over, when the front entity is within
        # the stop distance plus its length plus 5 m (see_around only)
        follow = self.see_around & (d <= stop + L + FRONT_ENTITY_STOP_MARGIN)
        stable = stop + L + FRONT_ENTITY_MARGIN
        requested = torch.where(d <= stop, torch.zeros_like(d),
                                torch.where(d >= stable, v_front + SPEED_STEP, v_front))
        requested = torch.minimum(torch.maximum(requested, torch.zeros_like(d)), target)
        requested = torch.where(target <= v_front, target, requested)
        local = torch.where(follow, requested, target)
        v1, a1, j1 = dynamic_states(local, self.c, self.v, self.a, dt)
        ds = (v1 + self.v) / 2.0 * dt
        on = self.active
        self.s = torch.where(on, self.s + ds, self.s)
        self.v, self.a, self.jerk = (torch.where(on, v1, self.v), torch.where(on, a1, self.a),
                                     torch.where(on, j1, self.jerk))
        self.mode = follow.to(torch.int8)
        return self.poses()
