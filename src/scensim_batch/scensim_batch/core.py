"""One batch's step, independent of the physics engine (roadmap 092 K2.2; batch-executor.md
section 4): the ego through its control component and a physics backend, SSv2's default
vehicles through scensim_batch.npc, props where Init put them, all slots at once.

    run = BatchRun(compiled, world, backend, component)
    run.reset()                       # every slot to its record
    obs = run.step(commands, dt)      # the ego's commands, a dict of [N] arrays (S3's form)

The backend is the engine: Isaac Lab's PhysX vehicle in the environment (K2.2's env), a
kinematic stand-in in tests. Its contract:

- `reset(mask, x, y, yaw, speed)`: slots `mask` [N] to those poses and speeds ([N] each);
- `apply(actuation)`: the component's actuation (S3: drive, brake [N, 4], tire_angle [N]);
- `advance(dt)`: one physics step;
- `state()`: {x, y, yaw, v, yaw_rate, accel_x, accel_y [N], wheel_speeds [N, 4]}.

The component is a vectorised batch (`sim_vehicle_ecu.batch.load_batch`, then
`require_vectorised`): a LoopBatch is refused here, for an executor that needs throughput.
"""
import numpy as np
import torch

from . import npc


def request_motion(kind, commands):
    """`(velocity [N], gear [N])` of the requests, as aib's Vehicle.apply takes them for the
    sticky-tire release: Autoware's velocity and gear; Ackermann's speed (0 when stale) and its
    sign's gear; a VESC's 0 and drive (what moves it is the component's alone)."""
    from sim_vehicle_ecu.signals import GEAR_DRIVE, GEAR_REVERSE
    n = len(next(iter(commands.values())))
    if kind == "autoware":
        valid = commands.get("valid", np.ones(n, bool))
        return (np.where(valid, commands["velocity"], 0.0),
                np.where(valid, commands["gear"], GEAR_DRIVE).astype(np.int64))
    if kind == "ackermann":
        v = np.where(np.isinf(commands["age"]), 0.0, commands["speed"])
        return v, np.where(v < 0, GEAR_REVERSE, GEAR_DRIVE).astype(np.int64)
    return np.zeros(n), np.full(n, GEAR_DRIVE, dtype=np.int64)


class BatchRun:
    def __init__(self, compiled, world, backend, component, route_length=300.0,
                 points=61, speed_limit=None, allow_loop=False, start_frames=0,
                 step_time=None):
        """`allow_loop`: accept a LoopBatch component (one Python step per slot), for checking
        a component before it has a batched form; never for a throughput number, and the run
        says so (`self.looped`)."""
        from sim_vehicle_ecu import batch as ecu_batch
        self.c, self.world, self.backend = compiled, world, backend
        self.looped = not getattr(component, "vectorised", False)
        self.component = component if allow_loop else ecu_batch.require_vectorised(component)
        self._ecu = ecu_batch
        if compiled.n != getattr(component, "n", compiled.n):
            raise ValueError(f"component has {component.n} slots, the batch {compiled.n}")
        egos = compiled.of_role("ego")
        if len(egos) != 1:
            raise ValueError(f"{compiled.scenario}: {len(egos)} egos, a batch has one policy")
        self.ego = egos[0]
        self.defaults = compiled.of_role("default")
        self.props = compiled.of_role("prop")
        self.route_length, self.points = route_length, points
        self.speed_limit = speed_limit
        self.vehicles = None
        self.first = None
        self.storyboard = compiled.storyboard
        from . import storyboard as sbmod
        self._sb = sbmod
        self.ctx = sbmod.Context(self)
        self.ctx.storyboard = self.storyboard
        nd = len(self.defaults)
        self.job = torch.zeros(compiled.n, nd, dtype=torch.int8)    # SSv2's LINEAR_ACCELERATION
        self.job_target = torch.zeros(compiled.n, nd, dtype=torch.float64)
        self.obs = None
        self.route = sbmod.RouteGoal(self)
        # SSv2's scenario time is A(S + k) - A(S): A the frames' step summed in a double from
        # the node's start, S the frames before startNpcLogic (SimulationClock; the fork's
        # clock_source:=simulator moves ROS time only). S differs from seats run to seats run;
        # a replay that must hit a SimulationTime boundary on the same frame passes its S.
        if start_frames and step_time is None:
            raise ValueError("start_frames needs the step_time it counts")
        a0 = 0.0
        for _ in range(int(start_frames)):
            a0 += step_time
        self.a0 = a0
        self.clock = torch.full((compiled.n,), a0, dtype=torch.float64)    # A
        self.clock0 = self.clock.clone()                                    # A(S)

    # --- reset ----------------------------------------------------------------------------

    def reset(self, mask=None):
        c = self.c
        n = c.n
        mask = torch.ones(n, dtype=torch.bool) if mask is None else torch.as_tensor(mask)
        e = self.ego
        self.backend.reset(mask, c.x[:, e], c.y[:, e], c.yaw[:, e], c.speed[:, e])
        self.component.reset_batch(mask.numpy())
        self.first = mask.clone() if self.first is None else (self.first | mask)
        if self.defaults:
            self._reset_vehicles(mask)
            self.job = torch.where(mask[:, None], torch.zeros_like(self.job), self.job)
        self.clock = torch.where(mask, torch.full_like(self.clock, self.a0), self.clock)
        self.clock0 = torch.where(mask, torch.full_like(self.clock0, self.a0), self.clock0)
        self.ctx.t = torch.where(mask, torch.zeros_like(self.ctx.t), self.ctx.t)
        self.ctx.standstill = torch.where(mask[:, None], torch.zeros_like(self.ctx.standstill),
                                          self.ctx.standstill)
        if self.storyboard is not None:
            self.storyboard.reset(mask)
        # the storyboard's first evaluation, at time 0, before the first update (SSv2's order)
        prev = self.obs
        self.obs = self._observe(self.backend.state(), None)
        self.ctx.obs = self.obs
        self.route.reset(mask)
        if self.c.goal_waypoints:
            self.route.set(mask, self.c.goal_waypoints)
        self._tick(mask)
        if prev is not None:      # the other slots keep this step's observation (and events)
            self.obs = _merge(prev, self.obs, mask)
            self.ctx.obs = self.obs
        return self.obs

    def _tick(self, mask=None):
        sb = self.storyboard
        if sb is None:
            self.obs["route"] = self.route.update(self.obs)
            return
        if mask is not None:
            keep = sb.outcome.clone()
            sb.outcome = torch.where(mask, sb.outcome, torch.full_like(sb.outcome, -1))
            sb.tick(self.ctx)
            sb.outcome = torch.where(mask, sb.outcome, keep)
        else:
            sb.tick(self.ctx)
        self.obs["route"] = self.route.update(self.obs)
        self.obs["outcome"] = sb.outcome.clone()
        self.obs["storyboard_complete"] = sb.complete.clone()
        self.obs["storyboard_stopped"] = sb.stopped.clone()

    def outcome(self):
        """Each slot's outcome (storyboard.OUTCOMES): running, success or failure (an exit
        action). A slot whose storyboard completed or stopped is still running, as in SSv2
        (`storyboard.complete`, `storyboard.stopped`)."""
        sb = self.storyboard
        if sb is None:
            return ["running"] * self.c.n
        return [self._sb.OUTCOMES[int(k)] for k in sb.outcome]

    def limit_jobs(self, up, down, target):
        """A linear SpeedAction's acceleration (up) or deceleration (down) limit, held until
        the speed passes the target (SSv2's job, POST_UPDATE), then reset."""
        self.job = torch.where(up, torch.ones_like(self.job), torch.where(
            down, torch.full_like(self.job, 2), self.job))
        self.job_target = torch.where(up | down, target, self.job_target)

    def _post_update(self):
        veh = self.vehicles
        if veh is None or not bool((self.job != 0).any()):
            return
        done = (((self.job == 1) & (veh.v >= self.job_target))
                | ((self.job == 2) & (veh.v <= self.job_target)))
        if bool(done.any()):
            self._reset_constraints(done)
            self.job = torch.where(done, torch.zeros_like(self.job), self.job)

    def _reset_constraints(self, m):
        veh = self.vehicles
        for k, val in npc.DEFAULT_CONSTRAINTS.items():
            setattr(veh.c, k, torch.where(m, torch.full_like(getattr(veh.c, k), val),
                                          getattr(veh.c, k)))

    def _reset_vehicles(self, mask):
        c, d = self.c, self.defaults
        x, y, yaw = c.x[:, d], c.y[:, d], c.yaw[:, d]
        pts = self.world.route(x, y, yaw, self.route_length, self.points)   # [N, D, P, 2]
        routes = npc.Routes(pts.to(torch.float64))
        length = torch.tensor([c.entities[i].length for i in d],
                              dtype=torch.float64).expand(c.n, len(d)).clone()
        width = torch.tensor([c.entities[i].width for i in d],
                             dtype=torch.float64).expand(c.n, len(d)).clone()
        limit = (self.speed_limit if self.speed_limit is not None
                 else self.world.speed_limit(x, y)) * torch.ones_like(x)
        fresh = npc.DefaultVehicles(routes, torch.zeros_like(x), c.speed[:, d].clone(), length,
                                    width, limit)
        # an Init SpeedAction is the target too (a step to it, held); without one SSv2 drives
        # at the route's speed limit
        fresh.set_target_speed(c.speed[:, d], c.speed_set[:, d])
        if self.vehicles is None:
            self.vehicles = fresh
            return
        m = mask[:, None].expand_as(x)
        old = self.vehicles
        for name in ("s", "v", "a", "jerk", "target", "length", "width", "speed_limit"):
            setattr(old, name, torch.where(m, getattr(fresh, name), getattr(old, name)))
        self._reset_constraints(m)
        old.routes.points = torch.where(m[..., None, None], fresh.routes.points, old.routes.points)
        old.routes = npc.Routes(old.routes.points)

    # --- step -----------------------------------------------------------------------------

    def _sensors(self, st):
        valid = ~self.first.numpy()
        out = {"wheel_speeds": st["wheel_speeds"].numpy(), "yaw_rate": st["yaw_rate"].numpy(),
               "accel_x": st["accel_x"].numpy(), "accel_y": st["accel_y"].numpy(),
               "valid": valid}
        truth = tuple(getattr(self.component, "truth", ()))
        if truth:
            avail = {"speed": st["v"], "yaw": st["yaw"], "yaw_rate": st["yaw_rate"],
                     "pose": torch.stack([st["x"], st["y"], st["yaw"]], -1)}
            missing = [t for t in truth if t not in avail]
            if missing:
                raise ValueError(f"the component declares truth {missing}, which the batch "
                                 "does not provide yet")
            out["truth"] = {t: avail[t].numpy() for t in truth}
            out["truth_valid"] = {t: valid.copy() for t in truth}
        return out

    def step(self, commands, dt):
        """One step of every slot, the engine stepped here: before_physics, the backend's
        advance, after_physics. Returns after_physics' observation."""
        self.before_physics(commands, dt)
        self.backend.advance(dt)
        return self.after_physics(dt)

    def before_physics(self, commands, dt):
        """The ego's component on what its sensors read, its actuation handed to the backend:
        what a host that owns the physics loop (Isaac Lab's DirectRLEnv) calls before each
        physics step, then steps the engine itself, then calls after_physics."""
        st = self.backend.state()
        act = self._ecu.checked_batch(
            self.component.step_batch(commands, self._sensors(st), dt), self.c.n, "ego component")
        self.first = torch.zeros_like(self.first)
        velocity, gear = request_motion(self.component.commands, commands)
        self.backend.apply(act, velocity=velocity, gear=gear)
        return act

    def after_physics(self, dt):
        """After one physics step: the ego read back, SSv2's default vehicles moved, the
        storyboard evaluated at the new time. Returns the observation: the ego's state, every
        entity's pose [N, E] (x, y, yaw, v) and each slot's outcome."""
        self.obs = self._observe(self.backend.state(), dt)
        self._post_update()
        live = (self.storyboard.outcome == 0 if self.storyboard is not None
                else torch.ones(self.c.n, dtype=torch.bool))
        self.clock = torch.where(live, self.clock + dt, self.clock)
        self.ctx.t = self.clock - self.clock0
        self.ctx.update(self.obs, dt)
        self._tick()
        return self.obs

    def _observe(self, st, dt):
        """Every entity's pose; with `dt`, SSv2's default vehicles moved by it first."""
        c = self.c
        x, y, yaw = c.x.clone(), c.y.clone(), c.yaw.clone()
        v = torch.zeros_like(x)
        x[:, self.ego], y[:, self.ego], yaw[:, self.ego], v[:, self.ego] = (
            st["x"], st["y"], st["yaw"], st["v"])
        if self.defaults:
            others_idx = [self.ego] + self.props
            ego_len = torch.tensor([c.entities[i].length for i in others_idx], dtype=torch.float64)
            ego_wid = torch.tensor([c.entities[i].width for i in others_idx], dtype=torch.float64)
            others = {"x": x[:, others_idx], "y": y[:, others_idx], "yaw": yaw[:, others_idx],
                      "v": v[:, others_idx], "length": ego_len.expand(c.n, -1),
                      "width": ego_wid.expand(c.n, -1)}
            nx, ny, nyaw = (self.vehicles.step(dt, others) if dt is not None
                            else self.vehicles.poses())
            d = self.defaults
            x[:, d], y[:, d], yaw[:, d], v[:, d] = nx, ny, nyaw, self.vehicles.v
        return {"x": x, "y": y, "yaw": yaw, "v": v, "ego": st}



def _merge(old, new, mask):
    """`new` for the slots in `mask`, `old` for the others: tensors whose first dimension is
    the slots, recursively through dicts; anything else is `new`'s."""
    if isinstance(new, dict):
        return {k: (_merge(old[k], v, mask) if isinstance(old, dict) and k in old else v)
                for k, v in new.items()}
    if (torch.is_tensor(new) and torch.is_tensor(old) and new.shape == old.shape
            and new.dim() >= 1 and new.shape[0] == mask.shape[0]):
        m = mask.reshape(-1, *([1] * (new.dim() - 1)))
        return torch.where(m, new, old)
    return new


_ = np
