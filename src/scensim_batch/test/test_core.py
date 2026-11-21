"""The batch's step (092 K2.2) on a kinematic stand-in for the engine: the ego through a
vectorised component, SSv2's default vehicles reacting to it, per-slot resets, and a LoopBatch
refused."""
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1] / "scensim_sampler"))
sys.path.insert(0, str(HERE.parents[1] / "sim_vehicle_ecu"))

try:
    import numpy as np
    import torch
    from sim_vehicle_ecu import batch as ecu_batch
except ImportError:   # no torch, or the sim_vehicle_ecu submodule absent
    torch = None

CASES = HERE.parents[1] / "scensim_sampler" / "examples" / "cases"
DT = 0.05
RADIUS, MASS = 0.35, 1800.0


class StraightWorld:
    def lane_pose(self, lanes, s, offset):
        y = torch.tensor([3.5 * (int(l) - 34513) for l in lanes], dtype=torch.float64)
        return s, y + offset, torch.zeros_like(s)

    def route(self, x, y, yaw, length, points):
        k = torch.linspace(0, length, points, dtype=torch.float64)
        px = x[..., None] + k * torch.cos(yaw)[..., None]
        py = y[..., None] + k * torch.sin(yaw)[..., None]
        return torch.stack([px, py], -1)

    def speed_limit(self, x, y):
        return 30.0


class PointMass:
    """The engine's stand-in: a point mass along its heading, wheels at one radius."""

    def __init__(self, n):
        z = torch.zeros(n, dtype=torch.float64)
        self.x, self.y, self.yaw, self.v, self.ax = z.clone(), z.clone(), z.clone(), z.clone(), z.clone()
        self.force = z.clone()

    def reset(self, mask, x, y, yaw, speed):
        for name, val in (("x", x), ("y", y), ("yaw", yaw), ("v", speed)):
            setattr(self, name, torch.where(mask, val, getattr(self, name)))
        self.ax = torch.where(mask, torch.zeros_like(self.ax), self.ax)

    def apply(self, act):
        drive = torch.as_tensor(act["drive"]).sum(-1)
        brake = torch.as_tensor(act["brake"]).sum(-1)
        self.force = (drive - brake * torch.sign(self.v)) / RADIUS

    def advance(self, dt):
        self.ax = self.force / MASS
        self.v = torch.clamp(self.v + self.ax * dt, min=0.0)
        self.x = self.x + self.v * torch.cos(self.yaw) * dt
        self.y = self.y + self.v * torch.sin(self.yaw) * dt

    def state(self):
        w = (self.v / RADIUS)[:, None].expand(-1, 4)
        z = torch.zeros_like(self.v)
        return {"x": self.x, "y": self.y, "yaw": self.yaw, "v": self.v, "yaw_rate": z,
                "accel_x": self.ax, "accel_y": z, "wheel_speeds": w}


class AccelBatch:
    """A vectorised autoware component: the requested acceleration as rear-wheel torque."""
    commands = "autoware"
    truth = ()

    def __init__(self, vehicle, params, seeds):
        self.n = len(seeds)
        self.resets = np.zeros(self.n, int)

    def reset_batch(self, mask):
        self.resets += np.asarray(mask, int)

    def step_batch(self, c, s, dt):
        a = np.where(c["valid"], c["acceleration"], 0.0)
        torque = a * MASS * RADIUS
        drive = np.zeros((self.n, 4))
        brake = np.zeros((self.n, 4))
        drive[:, 2] = drive[:, 3] = np.where(torque > 0, torque / 2, 0.0)
        brake[:, 2] = brake[:, 3] = np.where(torque < 0, -torque / 2, 0.0)
        return {"drive": drive, "brake": brake, "tire_angle": np.zeros(self.n)}


class Accel:   # what load_batch resolves: a class declaring its batch
    batch = AccelBatch


def cmds(n, accel):
    reqs = np.full(n, accel, dtype=float)
    return {"acceleration": reqs, "velocity": np.zeros(n), "tire_angle": np.zeros(n),
            "gear": np.zeros(n, dtype=np.int64), "valid": np.ones(n, bool)}


@unittest.skipIf(torch is None, "no torch or no sim_vehicle_ecu")
class Core(unittest.TestCase):
    def run_of(self, recs):
        from scensim_batch import compile as C, core
        c = C.compile_scenario(CASES / "cs2_lead_brakes_wet_night.xosc", recs, StraightWorld(),
                               provides=("world knob illumination=night",))
        comp = ecu_batch.load_batch(Accel, {}, {}, [7] * len(recs))
        return c, core.BatchRun(c, StraightWorld(), PointMass(c.n), comp)

    def test_the_lead_drives_its_init_speed_and_the_ego_its_component(self):
        recs = [{"values": {"ego_speed": 20.0, "lead_speed": 22.0}}, {"values": {"ego_speed": 10.0}}]
        c, run = self.run_of(recs)
        c.y[:, c.index("ego")] = 3.5   # the next lane: the point mass would drive through the lead
        run.route_length = 1000.0
        run.reset()
        for _ in range(40):   # 2 s
            obs = run.step(cmds(2, 1.0), DT)
        ego, lead = c.index("ego"), c.index("lead")
        # the ego: +1 m/s^2 from its Init speed (the component's first step has no reading)
        self.assertAlmostEqual(obs["v"][0, ego].item(), 22.0, delta=0.06)
        self.assertAlmostEqual(obs["v"][1, ego].item(), 12.0, delta=0.06)
        # the lead: no Init SpeedAction, so SSv2's default behaviour from rest towards the
        # speed limit (30), jerk-limited
        self.assertTrue(math.isnan(run.vehicles.target[0, 0].item()))
        v2 = obs["v"][0, lead].item()
        self.assertGreater(v2, 0.0)
        for _ in range(360):   # to 20 s
            obs = run.step(cmds(2, 0.0), DT)
        self.assertAlmostEqual(obs["v"][0, lead].item(), 30.0, delta=0.02)

    def test_a_lead_reacts_to_an_ego_stopped_ahead_of_it(self):
        recs = [{"values": {"ego_speed": 0.0}}]
        from scensim_batch import compile as C, core
        c = C.compile_scenario(CASES / "cs2_lead_brakes_wet_night.xosc", recs, StraightWorld(),
                               provides=("world knob illumination=night",))
        c.x[0, c.index("ego")] = 90.0     # the ego, stopped 54 m ahead of the lead
        run = core.BatchRun(c, StraightWorld(), PointMass(1), ecu_batch.load_batch(Accel, {}, {}, [1]))
        run.reset()
        for _ in range(400):
            obs = run.step(cmds(1, 0.0), DT)
        lead = c.index("lead")
        self.assertLess(obs["v"][0, lead].item(), 0.05)
        self.assertLess(obs["x"][0, lead].item() + 4.7 / 2, 90.0 - 4.7 / 2)

    def test_a_masked_reset_touches_only_its_slots(self):
        c, run = self.run_of([{"values": {}}, {"values": {}}])
        run.reset()
        for _ in range(20):
            run.step(cmds(2, 2.0), DT)
        lead = c.index("lead")
        before = run.vehicles.s[1, 0].item()
        run.reset(torch.tensor([True, False]))
        self.assertEqual(run.vehicles.s[0, 0].item(), 0.0)
        self.assertEqual(run.vehicles.s[1, 0].item(), before)
        self.assertEqual(run.backend.v[0].item(), 22.0)
        self.assertEqual(run.vehicles.v[0, 0].item(), 0.0)
        self.assertGreater(run.backend.v[1].item(), 22.0)
        self.assertEqual(run.component.resets.tolist(), [2, 1])
        _ = lead

    def test_a_loop_batch_is_refused(self):
        from scensim_batch import compile as C, core

        class Scalar:
            commands, truth = "autoware", ()

            def __init__(self, *a):
                pass

            def reset(self):
                pass

            def step(self, *a):
                pass

        c = C.compile_scenario(CASES / "cs2_lead_brakes_wet_night.xosc", [{"values": {}}],
                               StraightWorld(), provides=("world knob illumination=night",))
        loop = ecu_batch.load_batch(Scalar, {}, {}, [1])
        with self.assertRaisesRegex(ecu_batch.BatchError, "has no batched form"):
            core.BatchRun(c, StraightWorld(), PointMass(1), loop)


if __name__ == "__main__":
    unittest.main()
_ = math
