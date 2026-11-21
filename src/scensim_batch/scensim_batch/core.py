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


class BatchRun:
    def __init__(self, compiled, world, backend, component, route_length=300.0,
                 points=61, speed_limit=None):
        from sim_vehicle_ecu import batch as ecu_batch
        self.c, self.world, self.backend = compiled, world, backend
        self.component = ecu_batch.require_vectorised(component)
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
        """One step of every slot. Returns the observation: the ego's state and every
        entity's pose [N, E] (x, y, yaw, v)."""
        st = self.backend.state()
        act = self._ecu.checked_batch(
            self.component.step_batch(commands, self._sensors(st), dt), self.c.n, "ego component")
        self.first = torch.zeros_like(self.first)
        self.backend.apply(act)
        self.backend.advance(dt)
        st = self.backend.state()
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
            nx, ny, nyaw = self.vehicles.step(dt, others)
            d = self.defaults
            x[:, d], y[:, d], yaw[:, d], v[:, d] = nx, ny, nyaw, self.vehicles.v
        return {"x": x, "y": y, "yaw": yaw, "v": v, "ego": st}


_ = np
