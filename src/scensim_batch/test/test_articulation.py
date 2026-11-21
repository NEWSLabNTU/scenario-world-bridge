"""Backend B (092 K2.2b) without Isaac: the body file's actuation (steer, spin torques, the
brake), the state read back, the USD stage (where pxr is importable) and the backend's
contract on a stand-in for Isaac Lab's Articulation."""
import copy
import math
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

try:
    import torch
    from scensim_batch import articulation as art
except ImportError:   # no torch
    torch = None

SPEC = HERE.parents[1] / "sim_vehicle_spec" / "generated"
MODEL3 = SPEC / "tesla_model3" / "articulation_carla-0.9.16.yaml"
DT = 1 / 240

import importlib.util

HAVE_PXR = importlib.util.find_spec("pxr") is not None


def body():
    return art.load_body(MODEL3)


@unittest.skipIf(torch is None or not MODEL3.exists(), "no torch, or no generated body file")
class ModelTest(unittest.TestCase):
    def setUp(self):
        self.doc = body()
        self.m = art.ArticulationModel(self.doc, 3)

    def test_not_a_body_file_refused(self):
        with self.assertRaisesRegex(ValueError, "not a sim_vehicle_spec articulation"):
            art.check_body({"name": "x"})
        doc = copy.deepcopy(self.doc)
        del doc["joints"]["front_left_steer"]
        with self.assertRaisesRegex(ValueError, "no joint front_left_steer"):
            art.check_body(doc)

    def test_ackermann_inner_wheel_steers_more(self):
        t = self.m.steer_targets(torch.tensor([0.0, 0.3, -0.3]))
        self.assertEqual(t[0].tolist(), [0.0, 0.0])
        left, right = t[1].tolist()
        self.assertGreater(left, 0.3)          # turning left: the left wheel is inside
        self.assertLess(right, 0.3)
        self.assertGreater(right, 0.0)
        self.assertAlmostEqual(t[2, 0].item(), -right)    # mirrored
        self.assertAlmostEqual(t[2, 1].item(), -left)
        # both wheels' axles meet the rear axle's line at the same point (one turn centre)
        L, T = self.m.L, self.m.T
        self.assertAlmostEqual(L / math.tan(left) + T / 2, L / math.tan(right) - T / 2,
                               places=9)

    def test_steer_clipped_to_the_limit(self):
        t = self.m.steer_targets(torch.tensor([5.0, -5.0, 0.0]))
        self.assertLessEqual(t.abs().max().item(), self.m.max_steer + 1e-12)

    def test_no_ackermann_steers_both_the_same(self):
        doc = copy.deepcopy(self.doc)
        doc["actuation"]["steer"] = "bicycle angle to the front steer joints' targets"
        t = art.ArticulationModel(doc, 1).steer_targets(torch.tensor([0.2], dtype=torch.float64))
        self.assertEqual(t.tolist(), [[0.2, 0.2]])

    def test_drive_without_brake_passes(self):
        drive = torch.tensor([[0.0, 0.0, 500.0, 500.0]] * 3)
        tau = self.m.spin_efforts(drive, torch.zeros(3, 4), torch.zeros(3, 4), DT)
        self.assertEqual(tau.tolist(), drive.tolist())

    def _roll(self, m_road, v0, brake, steps, drive=0.0):
        """One wheel per slot on a road that grips: the wheel and its share of the car turn as
        one, (I + m r^2) alpha = torque, the road's torque on the wheel -m r^2 alpha. Returns
        the wheel's speed per step [steps, 3] (the model's three slots alike)."""
        r, I = self.m.r, self.m.spin_I
        omega = torch.full((3, 4), v0 / r, dtype=torch.float64)
        out = []
        for _ in range(steps):
            tau = self.m.spin_efforts(torch.full((3, 4), drive), torch.full((3, 4), brake),
                                      omega, DT)
            omega = omega + tau / (I + m_road * r * r) * DT
            out.append(omega[:, 0].clone())
        return torch.stack(out)

    def test_brake_full_torque_until_the_wheel_stops(self):
        m_road, v0, B = 440.0, 5.0, 1500.0
        r, I = self.m.r, self.m.spin_I[0].item()
        w = self._roll(m_road, v0, B, 400)
        stop = int(torch.nonzero(w[:, 0] * r <= 1e-3)[0])
        want = (v0 / r) / (B / (I + m_road * r * r)) / DT     # steps at full brake torque
        self.assertLessEqual(abs(stop - want), 2)
        self.assertTrue((w * r >= -1e-3).all())                 # never reversed (1 mm/s)
        self.assertTrue((w[stop + 2:].abs() * r < 1e-3).all())   # and held

    def test_brake_on_a_free_wheel_never_reverses_it(self):
        w = self._roll(0.0, 1.0, 3000.0, 50)
        self.assertTrue((w >= -1e-9).all())
        self.assertTrue((w[-40:].abs() < 1e-9).all())

    def test_brake_holds_against_a_smaller_drive(self):
        w = self._roll(440.0, 0.0, 500.0, 50, drive=300.0)
        self.assertTrue((w.abs() < 1e-9).all())
        w = self._roll(440.0, 0.0, 200.0, 50, drive=300.0)      # a smaller brake: it moves
        self.assertGreater(w[-1, 0].item(), 0.0)

    def _lift_mid_brake(self, use_grip):
        """243's case: braked while gripping (the estimate the car's), then the tyre leaves the
        road at about 0.3 rad/s; the wheel's speed per step after that."""
        r, I = self.m.r, self.m.spin_I
        omega = torch.full((3, 4), 5.0 / r, dtype=torch.float64)
        B = torch.full((3, 4), 3000.0)
        grip = torch.ones(3, 4, dtype=torch.bool)
        while omega[0, 0] > 0.4:                          # gripping
            tau = self.m.spin_efforts(torch.zeros(3, 4), B, omega, DT,
                                      grip=grip if use_grip else None)
            omega = omega + tau / (I + 440.0 * r * r) * DT
        self.assertGreater(self.m._I[0, 0].item(), 10 * I[0].item())
        out = []
        for _ in range(5):                                # lifted: the wheel alone
            tau = self.m.spin_efforts(torch.zeros(3, 4), B, omega, DT,
                                      grip=~grip if use_grip else None)
            omega = omega + tau / I * DT
            out.append(omega[0, 0].item())
        return out

    def test_a_wheel_that_lifts_mid_brake_is_not_reversed(self):
        w = self._lift_mid_brake(True)
        self.assertTrue(all(x >= -1e-9 for x in w), w)
        self.assertLess(abs(w[-1]), 1e-9)

    def test_without_grip_the_lift_reverses_it(self):
        """What the grip check is for: on the estimate alone the wheel reverses for a step."""
        self.assertLess(min(self._lift_mid_brake(False)), -1.0)

    def test_grip_from_load_and_slip(self):
        sus_t = self.m.suspension_target
        r = self.m.r
        q = torch.zeros(3, 4, dtype=torch.float64)       # at design height: loaded
        q[1] = sus_t - 0.002                              # drooped past the free length: lifted
        omega = torch.full((3, 4), 10.0, dtype=torch.float64)
        hub = torch.full((3, 4), 10.0 * r, dtype=torch.float64)
        hub[2, 0] = 10.0 * r + 2.0                        # one wheel sliding
        g = self.m.grip(q, omega, hub)
        self.assertTrue(g[0].all())
        self.assertFalse(g[1].any())
        self.assertEqual(g[2].tolist(), [False, True, True, True])

    def test_reset_forgets_the_step_before(self):
        self._roll(440.0, 5.0, 1500.0, 3)
        self.m.reset(torch.tensor([True, False, False]))
        self.assertEqual(self.m._known[:, 0].tolist(), [False, True, True])

    def test_effort_limit(self):
        lim = self.m.spin_limit[2].item()
        tau = self.m.spin_efforts(torch.full((1, 4), 10 * lim), torch.zeros(1, 4),
                                  torch.zeros(1, 4), DT)
        self.assertEqual(tau[0, 2].item(), lim)

    def test_velocity_spin_drive_refuses_torques(self):
        doc = copy.deepcopy(self.doc)
        for c in art.CORNERS:
            doc["joints"][f"{c}_spin"]["drive"] = {"type": "velocity", "damping": 1000.0}
        m = art.ArticulationModel(doc, 1)
        with self.assertRaisesRegex(ValueError, "apply_joints"):
            m.spin_efforts(torch.zeros(1, 4), torch.zeros(1, 4), torch.zeros(1, 4), DT)

    def test_suspension_targets_are_the_files(self):
        want = [self.doc["joints"][f"{c}_suspension"]["drive"]["target"] for c in art.CORNERS]
        self.assertEqual(self.m.suspension_targets().tolist(), [want] * 3)

    def test_state_in_the_cars_frame(self):
        h = torch.tensor([0.0, math.pi / 2, -0.7])
        quat = torch.stack([torch.zeros(3), torch.zeros(3), torch.sin(h / 2), torch.cos(h / 2)],
                           -1)
        origins = torch.tensor([[0.0, 0.0], [200.0, 0.0], [0.0, 200.0]])
        pos = torch.stack([origins[:, 0] + 1, origins[:, 1] + 2, torch.full((3,), 0.4)], -1)
        speed = torch.tensor([5.0, 3.0, 2.0])
        lin = torch.stack([speed * torch.cos(h), speed * torch.sin(h), torch.zeros(3)], -1)
        ang = torch.tensor([[0, 0, 0.1]] * 3)
        st, vb = self.m.state(pos, quat, lin, ang, torch.zeros(3, 4), None, None, origins)
        for k in range(3):
            self.assertAlmostEqual(st["yaw"][k].item(), h[k].item(), places=6)
            self.assertAlmostEqual(st["v"][k].item(), speed[k].item(), places=6)
            self.assertAlmostEqual(st["x"][k].item(), 1.0)
            self.assertAlmostEqual(st["y"][k].item(), 2.0)
        self.assertEqual(st["accel_x"].tolist(), [0.0] * 3)
        st2, _ = self.m.state(pos, quat, lin * 1.1, ang, torch.zeros(3, 4), vb, DT, origins)
        self.assertAlmostEqual(st2["accel_x"][0].item(), 0.5 / DT, places=4)
        self.assertAlmostEqual(st2["accel_y"][0].item(), 0.0, places=6)

    def test_actuator_groups(self):
        g = art.actuator_groups(self.doc)
        self.assertEqual(set(g), {"suspension", "steer", "spin"})
        self.assertEqual(g["spin"][1], "explicit")      # our torques
        self.assertEqual(g["steer"][1], "implicit")     # PhysX's position drive
        self.assertEqual(g["suspension"][0], [f"{c}_suspension" for c in art.CORNERS])
        k = self.doc["joints"]["rear_left_suspension"]["drive"]["stiffness"]
        self.assertEqual(g["suspension"][2]["rear_left_suspension"]["stiffness"], k)


@unittest.skipIf(torch is None or not HAVE_PXR or not MODEL3.exists(), "no pxr")
class UsdTest(unittest.TestCase):
    def test_stage(self):
        from pxr import Usd, UsdPhysics
        doc = body()
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "car.usda"
            root = art.write_usd(doc, path)
            stage = Usd.Stage.Open(str(path))
            top = stage.GetPrimAtPath(root)
            self.assertTrue(top.HasAPI(UsdPhysics.ArticulationRootAPI))
            total = 0.0
            for name, link in doc["links"].items():
                p = stage.GetPrimAtPath(f"{root}/{name}")
                self.assertTrue(p.HasAPI(UsdPhysics.RigidBodyAPI), name)
                total += UsdPhysics.MassAPI(p).GetMassAttr().Get()
            self.assertAlmostEqual(total, sum(l["mass"] for l in doc["links"].values()),
                                   places=3)
            steer = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(f"{root}/joints/"
                                                                 "front_left_steer"))
            self.assertEqual(steer.GetAxisAttr().Get(), "Z")
            self.assertAlmostEqual(steer.GetUpperLimitAttr().Get(),
                                   math.degrees(doc["actuation"]["max_steer_angle"]), places=3)
            spin = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(f"{root}/joints/rear_left_spin"))
            self.assertEqual(spin.GetAxisAttr().Get(), "Y")
            self.assertFalse(spin.GetLowerLimitAttr().HasAuthoredValue())   # free to spin
            self.assertEqual([str(t) for t in spin.GetBody1Rel().GetTargets()],
                             [f"{root}/rear_left_wheel"])
            sus = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(f"{root}/joints/"
                                                                "front_right_suspension"))
            self.assertEqual(sus.GetAxisAttr().Get(), "Z")
            wheel = stage.GetPrimAtPath(f"{root}/front_left_wheel/collision")
            self.assertTrue(wheel.HasAPI(UsdPhysics.CollisionAPI))
            self.assertEqual(wheel.GetAttribute("axis").Get(), "Y")


# --- the backend on a stand-in for Isaac Lab's Articulation ------------------------------------

class _Proxy:
    def __init__(self, t):
        self.torch = t


class _Data:
    def __init__(self, a):
        self.a = a

    @property
    def root_link_pose_w(self):
        return _Proxy(self.a.pose)

    @property
    def root_link_vel_w(self):
        return _Proxy(self.a.vel)

    @property
    def joint_vel(self):
        return _Proxy(self.a.jvel)

    @property
    def joint_pos(self):
        return _Proxy(self.a.jpos)


class FakeArticulation:
    """The calls ArticulationBackend makes, recorded; joints in an order of its own."""

    def __init__(self, n, names):
        self.num_instances, self.device = n, "cpu"
        self.names = names
        self.num_joints = len(names)
        self.pose = torch.zeros(n, 7)
        self.pose[:, 6] = 1
        self.vel = torch.zeros(n, 6)
        self.jpos = torch.zeros(n, len(names))
        self.jvel = torch.zeros(n, len(names))
        self.pos_target = self.vel_target = self.effort = None
        self.writes = 0
        self.data = _Data(self)

    def find_joints(self, names, preserve_order=False):
        return [self.names.index(n) for n in names], list(names)

    def write_root_pose_to_sim_index(self, root_pose, env_ids):
        self.pose[env_ids] = root_pose

    def write_root_velocity_to_sim_index(self, root_velocity, env_ids):
        self.vel[env_ids] = root_velocity

    def write_joint_position_to_sim_index(self, position, env_ids):
        self.jpos[env_ids] = position

    def write_joint_velocity_to_sim_index(self, velocity, env_ids):
        self.jvel[env_ids] = velocity

    def set_joint_position_target_index(self, target):
        self.pos_target = target.clone()

    def set_joint_velocity_target_index(self, target, joint_ids):
        self.vel_target = (target.clone(), list(joint_ids))

    def set_joint_effort_target_index(self, target, joint_ids):
        self.effort = (target.clone(), list(joint_ids))

    def write_data_to_sim(self):
        self.writes += 1

    def update(self, dt):
        pass


@unittest.skipIf(torch is None or not MODEL3.exists(), "no torch, or no generated body file")
class BackendTest(unittest.TestCase):
    def setUp(self):
        self.doc = body()
        names = list(reversed(art.joint_order(self.doc)))      # not the backend's order
        self.a = FakeArticulation(3, names)
        self.origins = torch.tensor([[0.0, 0.0], [200.0, 0.0], [0.0, 200.0]])
        self.b = art.ArticulationBackend(self.a, self.doc, self.origins, z=0.05)

    def test_vectorised(self):
        self.assertTrue(art.ArticulationBackend.vectorised)

    def test_reset_puts_the_masked_slots_at_their_poses(self):
        self.b.reset(torch.tensor([False, True, True]), torch.tensor([0, 10.0, -5]),
                     torch.tensor([0, 2.0, 3]), torch.tensor([0, math.pi / 2, 0]),
                     torch.tensor([0, 4.0, 0]))
        p = self.a.pose
        self.assertEqual(p[0].tolist(), [0, 0, 0, 0, 0, 0, 1])          # untouched
        self.assertAlmostEqual(p[1, 0].item(), 210.0)
        self.assertAlmostEqual(p[1, 2].item(), 0.05)
        self.assertAlmostEqual(p[1, 5].item(), math.sin(math.pi / 4), places=6)   # qz, xyzw
        self.assertAlmostEqual(p[2, 1].item(), 203.0)
        self.assertAlmostEqual(self.a.vel[1, 1].item(), 4.0, places=5)
        spin = [self.b.ids[f"{c}_spin"] for c in art.CORNERS]
        r = self.doc["actuation"]["wheel_radius"]
        self.assertAlmostEqual(self.a.jvel[1, spin[0]].item(), 4.0 / r, places=5)
        st = self.b.state()
        self.assertAlmostEqual(st["x"][1].item(), 10.0, places=4)
        self.assertAlmostEqual(st["yaw"][1].item(), math.pi / 2, places=6)
        self.assertAlmostEqual(st["v"][1].item(), 4.0, places=5)

    def test_actuate_writes_the_joints_by_name(self):
        n = 3
        act = {"drive": torch.tensor([[0, 0, 400.0, 400.0]] * n),
               "brake": torch.zeros(n, 4), "tire_angle": torch.tensor([0.0, 0.2, -0.2])}
        self.b.apply(act)
        self.b.actuate(DT)
        ids = self.b.ids
        tgt = self.a.pos_target
        self.assertGreater(tgt[1, ids["front_left_steer"]].item(), 0.2)
        self.assertLess(tgt[2, ids["front_right_steer"]].item(), -0.2)
        want = self.doc["joints"]["rear_left_suspension"]["drive"]["target"]
        self.assertAlmostEqual(tgt[0, ids["rear_left_suspension"]].item(), want, places=6)
        effort, jids = self.a.effort
        self.assertEqual(jids, [ids[f"{c}_spin"] for c in art.CORNERS])
        self.assertEqual(effort[0].tolist(), [0, 0, 400.0, 400.0])
        self.assertEqual(self.a.writes, 1)

    def test_joints_bypass_the_component(self):
        steer = torch.tensor([[0.1, 0.1]] * 3)
        spin = torch.full((3, 4), 50.0)
        self.b.apply_joints(steer, spin)
        self.b.actuate(DT)
        self.assertAlmostEqual(self.a.pos_target[0, self.b.ids["front_right_steer"]].item(), 0.1,
                               places=6)
        self.assertEqual(self.a.effort[0][0].tolist(), [50.0] * 4)

    def test_acceleration_from_one_step_to_the_next(self):
        self.b.state()
        self.a.vel[:, 0] = 1.0
        self.b.sample(DT)
        st = self.b.state()
        self.assertAlmostEqual(st["accel_x"][0].item(), 1.0 / DT, places=3)
        st = self.b.state()            # no physics step in between: the same step's
        self.assertAlmostEqual(st["accel_x"][0].item(), 1.0 / DT, places=3)
        self.b.sample(DT)              # a step with no change: none
        self.assertEqual(self.b.state()["accel_x"][0].item(), 0.0)


if __name__ == "__main__":
    unittest.main()
