"""Smoke of backend B (092 K2.2b): N articulation cars from sim_vehicle_spec's body file on a
ground plane, stepped through the backend's contract with a fixed actuation. Three phases per
car, one JSON line each N:

- straight: `--torque` N m on the driven wheels for `--seconds`; distance and speed;
- brake: `--brake` N m on every wheel until the cars stop; the stopping time and distance;
- circle: a constant `--steer` (bicycle rad) at about `--circle-speed`; the radius as
  v / yaw_rate against the bicycle's L / tan(steer).

Every car gets the same actuation, so the spread across cars says they ran as one. Run in Isaac
Lab's Python (scripts/isaaclab_overlay.sh) with sim_vehicle_ecu off the path (not needed),
headless, on the CPU pipeline unless --device says otherwise:

    python smoke_articulation.py --body <generated/<car>/articulation_<body>.yaml> --n 1 4
"""
import argparse
import json
import math
import os
import sys
import tempfile
import time


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--body", required=True)
    ap.add_argument("--n", type=int, nargs="+", default=[1, 4])
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--dt", type=float, default=1 / 240)
    ap.add_argument("--torque", type=float, default=400.0, help="N m per driven wheel")
    ap.add_argument("--brake", type=float, default=1500.0, help="N m per wheel")
    ap.add_argument("--steer", type=float, default=0.15, help="bicycle angle, rad")
    ap.add_argument("--circle-speed", type=float, default=5.0)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--position-iterations", type=int, default=None)
    ap.add_argument("--velocity-iterations", type=int, default=None)
    ap.add_argument("--ground-friction", type=float, default=1.0,
                    help="the ground's; the tyre's is the body file's, combined by multiplying")
    a = ap.parse_args(argv)

    from isaaclab.app import AppLauncher
    launcher = AppLauncher(headless=True, device=a.device)
    app = launcher.app

    import torch

    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab.sim import SimulationCfg, SimulationContext

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from scensim_batch import articulation as art

    doc = art.load_body(a.body)
    driven = torch.tensor([bool(doc["joints"][f"{c}_spin"].get("driven"))
                           for c in art.CORNERS], dtype=torch.float64)
    L = doc["actuation"]["wheel_base"]
    tmp = tempfile.mkdtemp(prefix="smoke_articulation_")
    usd = os.path.join(tmp, "car.usda")
    art.write_usd(doc, usd)
    print(json.dumps({"body": doc["name"], "spec_hash": doc["spec_hash"],
                      "physics_hash": doc["physics_hash"], "usd": usd}), flush=True)

    for n in a.n:
        sim = SimulationContext(SimulationCfg(dt=a.dt, device=a.device))
        g = sim_utils.GroundPlaneCfg(size=(2000.0, 2000.0), physics_material=(
            sim_utils.RigidBodyMaterialCfg(static_friction=a.ground_friction,
                                               dynamic_friction=a.ground_friction)))
        g.func("/World/ground", g)
        spacing = 60.0
        side = max(1, math.ceil(math.sqrt(n)))
        origins = [((i % side) * spacing, (i // side) * spacing) for i in range(n)]
        for i, (ox, oy) in enumerate(origins):
            sim_utils.create_prim(f"/World/envs/env_{i}", "Xform", translation=(ox, oy, 0.0))
        asset = Articulation(art.articulation_cfg(
            doc, usd, z=0.02, position_iterations=a.position_iterations,
            velocity_iterations=a.velocity_iterations))
        sim.reset()
        be = art.ArticulationBackend(asset, doc, torch.tensor(origins), sim=sim, z=0.02)
        z = torch.zeros(n, dtype=torch.float64)
        be.reset(torch.ones(n, dtype=torch.bool), z, z, z, z)
        for _ in range(int(0.5 / a.dt)):     # settle on the springs
            be.apply({"drive": torch.zeros(n, 4), "brake": torch.full((n, 4), a.brake),
                      "tire_angle": z})
            be.advance(a.dt)
        row = {"n": n, "device": a.device, "ground_friction": a.ground_friction,
               "iterations": [a.position_iterations, a.velocity_iterations]}
        jp = lambda: asset.data.joint_pos.torch.detach().cpu().double()   # noqa: E731
        jv = lambda: asset.data.joint_vel.torch.detach().cpu().double()   # noqa: E731
        sus = jp()[:, be.sus_ids]
        row["suspension_q_at_rest"] = [float(sus.min()), float(sus.max())]
        settled = be.state()
        row["settle_drift_m"] = float(settled["x"].abs().max())

        def run(act, steps):
            be.timing = {"actuate": 0.0, "physics": 0.0, "sample": 0.0, "steps": 0}
            t0 = time.perf_counter()
            for _ in range(steps):
                be.apply(act)
                be.advance(a.dt)
            return (time.perf_counter() - t0) / steps

        # straight
        drive = (driven * a.torque).expand(n, 4).clone()
        per = run({"drive": drive, "brake": torch.zeros(n, 4), "tire_angle": z},
                  int(round(a.seconds / a.dt)))
        st = be.state()
        row.update({"ms_per_step": per * 1e3, "us_per_car": per * 1e6 / n,
                    "straight_x_min": float(st["x"].min()), "straight_x_max": float(st["x"].max()),
                    "straight_v_mean": float(st["v"].mean()),
                    "straight_y_abs_max": float(st["y"].abs().max()),
                    "straight_yaw_abs_max": float(st["yaw"].abs().max()),
                    # the actions read back from the tensors: the driven wheels' spin against
                    # the car's speed (slip), the torque the explicit actuator applied
                    "straight_wheel_speed_x_r_over_v": (
                        jv()[:, be.spin_ids] * doc["actuation"]["wheel_radius"]
                        / st["v"][:, None]).mean(0).tolist(),
                    "straight_applied_spin_torque": asset.data.applied_torque.torch[
                        0, be.spin_ids].detach().cpu().tolist()})
        # brake to a stop
        x0, v0 = st["x"].clone(), float(st["v"].mean())
        t, steps = 0.0, 0
        brake = {"drive": torch.zeros(n, 4), "brake": torch.full((n, 4), a.brake),
                 "tire_angle": z}
        while steps < int(20 / a.dt):
            run(brake, 1)
            t, steps = t + a.dt, steps + 1
            if float(be.state()["v"].abs().max()) < 0.05:
                break
        st = be.state()
        row.update({"brake_from_v": v0, "brake_time_s": t,
                    "brake_dist_m": float((st["x"] - x0).mean()),
                    "brake_decel": v0 / t if t else None,
                    "stopped": bool(float(st["v"].abs().max()) < 0.05)})
        # a circle at a constant steer: speed held by a proportional torque on the driven wheels
        steer = torch.full((n,), a.steer, dtype=torch.float64)
        for _ in range(int(8 / a.dt)):
            v = be.state()["v"]
            tq = (a.torque * (a.circle_speed - v) / max(a.circle_speed, 1e-3)).clamp(
                -a.torque, a.torque)
            be.apply({"drive": driven * tq[:, None], "brake": torch.zeros(n, 4),
                      "tire_angle": steer})
            be.advance(a.dt)
        st = be.state()
        want = be.model.steer_targets(steer)
        got = jp()[:, be.steer_ids]
        tgt = asset.data.joint_pos_target.torch.detach().cpu().double()[:, be.steer_ids]
        row.update({"steer_target_written": tgt.mean(0).tolist(),
                    "steer_target_wanted": want.mean(0).tolist(),
                    "steer_joint_pos": got.mean(0).tolist()})
        radius = st["v"] / st["yaw_rate"].clamp(min=1e-6)
        row.update({"circle_v": float(st["v"].mean()), "circle_yaw_rate": float(
            st["yaw_rate"].mean()), "circle_radius_m": float(radius.mean()),
            "circle_radius_spread_m": float(radius.max() - radius.min()),
            "bicycle_radius_m": L / math.tan(a.steer)})
        print(json.dumps(row), flush=True)
        sim.clear_instance()
        sim_utils.create_new_stage()
    app.close()


if __name__ == "__main__":
    main()
