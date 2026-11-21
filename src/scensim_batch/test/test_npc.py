"""The batched default vehicle behaviour (092 K2.1) against SSv2's logic transcribed for one
vehicle (ssv2_ref.py): the planner and the stop distance equal per element; a follower on a
straight road equals the scalar run slot by slot over N random slots; the front entity found
only on the path, ahead, in the cone."""
import math
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    import torch
except ImportError:  # the package's torch extra
    torch = None

import ssv2_ref as ref  # noqa: E402

if torch is not None:
    from scensim_batch import npc  # noqa: E402

DT = 0.05
F64 = dict(dtype=torch.float64) if torch is not None else {}


def t(x):
    return torch.tensor(x, **F64)


@unittest.skipIf(torch is None, "no torch")
class Planner(unittest.TestCase):
    def test_dynamic_states_and_running_distance_equal_the_scalar_planner(self):
        rng = random.Random(3)
        rows = [(rng.uniform(0, 30), rng.uniform(0, 30), rng.uniform(-5, 5)) for _ in range(400)]
        target, v, a = (t([[r[i] for r in rows]]) for i in range(3))
        c = npc.Constraints.default(v.shape)
        v1, a1, j1 = npc.dynamic_states(target, c, v, a, DT)
        for i, (tg, vv, aa) in enumerate(rows):
            want = ref.dynamic_states(tg, ref.C, vv, aa, DT)
            self.assertEqual((v1[0, i].item(), a1[0, i].item(), j1[0, i].item()), want)
        zero = torch.zeros_like(v)
        d = npc.running_distance(zero, c, v, a, zero, DT)
        for i, (_, vv, aa) in enumerate(rows):
            self.assertAlmostEqual(d[0, i].item(), ref.running_distance(0.0, ref.C, vv, aa, 0.0, DT),
                                   places=9)

    def test_the_limits_hold_on_a_run_to_target(self):
        v = torch.zeros(1, 1, **F64)
        a = torch.zeros_like(v)
        c = npc.Constraints.default(v.shape)
        for _ in range(400):
            v1, a1, j = npc.dynamic_states(t([[15.0]]), c, v, a, DT)
            self.assertLessEqual(a1.item(), 10.0 + 1e-9)
            if v1.item() < 13.0:   # SSv2 caps acceleration at (target - v) / dt on arrival,
                self.assertLessEqual(abs(j.item()), 3.0 + 1e-6)   # past the jerk limit
            v, a = v1, a1
        self.assertAlmostEqual(v.item(), 15.0, places=6)


def straight(n, e, length=600.0):
    xs = torch.linspace(0, length, 61, **F64)
    pts = torch.stack([xs, torch.zeros_like(xs)], -1)
    return npc.Routes(pts.expand(n, e, 61, 2).clone())


def scalar_follow(s, v, target, s_lead, v_lead, steps, L=4.0, L_lead=4.0):
    """One follower behind a lead at constant speed, the tree's speed logic per step."""
    a = j = 0.0
    out = []
    for _ in range(steps):
        horizon = max(min(max(v * 5.0, 20.0), 50.0), ref.running_distance(0.0, ref.C, v, a, j, DT))
        ds = s_lead - s
        gap = ds - L / 2 - L_lead / 2
        d = gap + L / 2 if (ds > 0 and gap >= 0 and ds <= horizon + L_lead) else None
        local = ref.local_target(target, v, a, j, L, d, v_lead, DT)
        v1, a1, j1 = ref.dynamic_states(local, ref.C, v, a, DT)
        s += (v1 + v) / 2.0 * DT
        v, a, j = v1, a1, j1
        s_lead += v_lead * DT
        out.append((s, v))
    return out


@unittest.skipIf(torch is None, "no torch")
class Following(unittest.TestCase):
    def test_n_slots_equal_n_scalar_runs(self):
        rng = random.Random(5)
        n, steps = 64, 300
        cases = [(rng.uniform(0, 20), rng.uniform(0, 15), rng.uniform(5, 20),
                  rng.uniform(15, 80), rng.choice([0.0, rng.uniform(0, 12)])) for _ in range(n)]
        routes = straight(n, 1)
        s0 = t([[c[0]] for c in cases])
        v0 = t([[c[1]] for c in cases])
        car = npc.DefaultVehicles(routes, s0, v0, torch.full_like(s0, 4.0),
                                  torch.full_like(s0, 1.8), t([[c[2]] for c in cases]))
        lead_s = t([[c[0] + c[3]] for c in cases])
        lead_v = t([[c[4]] for c in cases])
        got = []
        for _ in range(steps):
            others = {"x": lead_s, "y": torch.zeros_like(lead_s), "yaw": torch.zeros_like(lead_s),
                      "length": torch.full_like(lead_s, 4.0), "width": torch.full_like(lead_s, 1.8),
                      "v": lead_v}
            car.step(DT, others)
            got.append((car.s.clone(), car.v.clone()))
            lead_s = lead_s + lead_v * DT
        for i, c in enumerate(cases):
            want = scalar_follow(c[0], c[1], c[2], c[0] + c[3], c[4], steps)
            for k in (0, 50, 150, steps - 1):
                self.assertAlmostEqual(got[k][0][i, 0].item(), want[k][0], places=6, msg=(i, k))
                self.assertAlmostEqual(got[k][1][i, 0].item(), want[k][1], places=6, msg=(i, k))

    def test_a_stopped_lead_is_never_reached(self):
        routes = straight(1, 1)
        car = npc.DefaultVehicles(routes, t([[0.0]]), t([[12.0]]), t([[4.0]]), t([[1.8]]),
                                  t([[12.0]]))
        lead = {"x": t([[80.0]]), "y": t([[0.0]]), "yaw": t([[0.0]]), "length": t([[4.0]]),
                "width": t([[1.8]]), "v": t([[0.0]])}
        for _ in range(600):
            car.step(DT, lead)
        self.assertLess(car.v.item(), 0.05)
        self.assertLess(car.s.item() + 2.0, 80.0 - 2.0)   # bumpers apart

    def test_only_an_entity_on_the_path_ahead_counts(self):
        routes = straight(1, 1)
        car = npc.DefaultVehicles(routes, t([[10.0]]), t([[10.0]]), t([[4.0]]), t([[1.8]]),
                                  t([[10.0]]))
        car.dt = DT
        for x, y, seen in ((30.0, 0.0, True), (30.0, 3.5, False), (5.0, 0.0, False),
                           (30.0, 1.5, True)):   # 1.5 m off: its box reaches the path
            d, _, _ = car.front({"x": t([[x]]), "y": t([[y]]), "yaw": t([[0.0]]),
                                 "length": t([[4.0]]), "width": t([[1.8]]), "v": t([[0.0]])})
            self.assertEqual(math.isfinite(d.item()), seen, (x, y))

    def test_vehicles_in_one_slot_see_each_other(self):
        routes = straight(1, 2)
        car = npc.DefaultVehicles(routes, t([[0.0, 40.0]]), t([[12.0, 0.0]]),
                                  torch.full((1, 2), 4.0, **F64), torch.full((1, 2), 1.8, **F64),
                                  t([[12.0, 0.0]]))
        for _ in range(600):
            car.step(DT)
        self.assertLess(car.v[0, 0].item(), 0.05)
        self.assertLess(car.s[0, 0].item(), car.s[0, 1].item() - 4.0)


if __name__ == "__main__":
    unittest.main()
