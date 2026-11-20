"""Roadmap 091 W6/W7: open-loop tests of a small car (the RoboRacer example) in Isaac, at a
chosen physics rate. aib.steptest's harness, with tests sized for a 1/10 car.

    python smallcar.py --ego EGO.yaml --out DIR --frame 0.05 --sub 3 [--set k=v ...]
        [--only test,...] [--solver TGS|PGS]

In Isaac's environment with aib on the path (aib's scripts/isaac_env.sh, then
PYTHONPATH=<aib>/src/aib). EGO.yaml is an aib ego config, `rig: roboracer.rig.yaml` beside it;
make the rig with `python3 -m aib.gen --spec ../vehicles/roboracer.yaml -o roboracer.rig.yaml`.
`--set` takes a field of aib's VehicleParams or of sim_vehicle_ecu's Calibration by name.
Physics rate = sub / frame. Per test a CSV of every physics step, and summary.json.

The tests: rest, launch, cruise at 3 m/s, braking from 3 m/s, full lock at 1.5 and 4 m/s on
the open plane; in their own lanes, four 10 mm cleats at 3 m/s (`bumps`), creeping into a wall
and pushing on it (`wall`), and a 0.1 kg cone hit at 2 m/s (`cone`).
"""
import argparse
import csv
import json
import math
import os
import statistics as st
import sys
import time
from pathlib import Path


def hold(target, v, gain=2.0, limit=4.0):
    return max(-limit, min(limit, gain * (target - v)))


# name: (length s, command(t, v, memo) -> (accel, velocity, tire_angle))
def rest(t, v, m):
    return (0.0, 0.0, 0.0)


def launch(t, v, m):
    return (8.0, 8.0, 0.0)


def cruise(t, v, m):
    return (hold(3.0, v), 3.0, 0.0)


def brake(t, v, m):
    if "t_b" not in m and t > 2.5:
        m["t_b"] = t
    if "t_b" not in m:
        return (hold(3.0, v), 3.0, 0.0)
    return (-6.0, 0.0, 0.0)


def lock(speed, angle):
    def cmd(t, v, m):
        return (hold(speed, v), speed, angle if t > 2.0 else 0.0)
    return cmd


def bumps(t, v, m):
    return (hold(3.0, v), 3.0, 0.0)


def wall(t, v, m):
    # creep up to the wall at 0.5 m/s, then keep asking for 1 m/s: the car pushes on it
    return (hold(0.5, v), 0.5, 0.0) if t < 3.0 else (hold(1.0, v), 1.0, 0.0)


def cone(t, v, m):
    return (hold(2.0, v), 2.0, 0.0)


TESTS = {
    "rest": (3.0, rest),
    "launch": (3.0, launch),
    "cruise": (8.0, cruise),
    "brake": (5.0, brake),
    "lock_slow": (8.0, lock(1.5, 0.36)),
    "lock_fast": (6.0, lock(4.0, 0.36)),
    "bumps": (3.0, bumps),
    "wall": (6.0, wall),
    "cone": (3.0, cone),
}
# each test's lane: the start's y, m (the obstacles of one lane stay out of the others)
LANES = {"bumps": 5.0, "wall": 10.0, "cone": 15.0}
CLEATS = (1.5, 2.5, 3.5, 4.5)  # x of each cleat's centre, m
CLEAT = (0.03, 1.0, 0.010)  # its size along x, y and its height, m
WALL_X = 1.0  # the wall's face, m ahead of the start
CONE_X = 1.5  # the cone's centre, m
CONE = (0.05, 0.15, 0.1)  # radius, height m, mass kg


def sd(xs):
    return st.pstdev(xs) if len(xs) > 1 else 0.0


def measure(test, rows, p):
    out = {}
    if test == "rest":
        out["drift_xy_mm"] = 1000 * math.hypot(rows[-1]["x"] - rows[0]["x"], rows[-1]["y"] - rows[0]["y"])
        out["z_sd_mm"] = 1000 * sd([r["z"] for r in rows])
        out["v_max"] = max(abs(r["v"]) for r in rows)
    elif test == "launch":
        for lv in (1.0, 3.0, 5.0):
            t = next((r["t"] for r in rows if r["v"] >= lv), None)
            out[f"t_0_{lv:g}"] = t
        out["v_end"] = rows[-1]["v"]
        out["slip_max"] = max(max(abs(r[f"slip_{w}"]) for w in ("fl", "fr", "rl", "rr")) for r in rows)
    elif test == "cruise":
        s = [r for r in rows if r["t"] > 3.0]
        out["v_mean"] = st.mean(r["v"] for r in s)
        out["v_sd"] = sd([r["v"] for r in s])
        out["z_sd_mm"] = 1000 * sd([r["z"] for r in s])
        out["yaw_rate_sd"] = sd([r["yaw_rate"] for r in s])
        out["lat_acc_sd"] = sd([r["ay"] for r in s])
        out["lon_acc_sd"] = sd([r["ax"] for r in s])
        out["wheel_slip_sd"] = sd([r["slip_rl"] for r in s])
        out["wheel_slip_mean"] = st.mean(r["slip_rl"] for r in s)
        out["y_end_mm"] = 1000 * rows[-1]["y"]
    elif test == "brake":
        i0 = next(i for i, r in enumerate(rows) if r["cmd_a"] < -1)
        v0 = rows[i0]["v"]
        stop = next((i for i in range(i0, len(rows)) if abs(rows[i]["v"]) < 0.02), None)
        out["v0"] = v0
        if stop is not None:
            out["dist"] = rows[stop]["x"] - rows[i0]["x"]
            out["time"] = rows[stop]["t"] - rows[i0]["t"]
            out["decel_mean"] = v0 / out["time"] if out["time"] > 0 else None
            tail = rows[stop:]
            out["after_stop_drift_mm"] = 1000 * (tail[-1]["x"] - tail[0]["x"])
            out["after_stop_vmax"] = max(abs(r["v"]) for r in tail)
        out["slip_min"] = min(min(r[f"slip_{w}"] for w in ("fl", "fr", "rl", "rr")) for r in rows[i0:])
        out["pitch_z_min_mm"] = 1000 * (min(r["z"] for r in rows[i0:]) - rows[i0]["z"])
    elif test == "bumps":
        out["z_range_mm"] = 1000 * (max(r["z"] for r in rows) - min(r["z"] for r in rows))
        out["pitch_rate_max"] = max(abs(r["pitch_rate"]) for r in rows)
        out["roll_rate_max"] = max(abs(r["roll_rate"]) for r in rows)
        out["yaw_rate_max"] = max(abs(r["yaw_rate"]) for r in rows)
        out["airborne_share"] = sum(r["airborne"] for r in rows) / (4 * len(rows))
        out["v_min"] = min(r["v"] for r in rows if r["t"] > 0.8)
        out["y_end_mm"] = 1000 * rows[-1]["y"]
        tail = [r for r in rows if r["x"] > CLEATS[-1] + 0.6]  # past the last cleat's rear wheels
        out["z_sd_after_mm"] = 1000 * sd([r["z"] for r in tail]) if tail else None
    elif test == "wall":
        s = [r for r in rows if r["t"] > rows[-1]["t"] - 2.0]
        out["x_mean"] = st.mean(r["x"] for r in s)
        out["x_sd_mm"] = 1000 * sd([r["x"] for r in s])
        out["v_sd"] = sd([r["v"] for r in s])
        out["z_sd_mm"] = 1000 * sd([r["z"] for r in s])
        out["yaw_rate_sd"] = sd([r["yaw_rate"] for r in s])
        out["pitch_rate_sd"] = sd([r["pitch_rate"] for r in s])
        out["ax_sd"] = sd([r["ax"] for r in s])
        out["slip_rl_mean"] = st.mean(r["slip_rl"] for r in s)
        out["y_end_mm"] = 1000 * rows[-1]["y"]
    elif test == "cone":
        out["cone_moved_m"] = rows[-1]["cone_d"]
        out["cone_z_end"] = rows[-1]["cone_z"]
        hit = next((r for r in rows if r["cone_d"] > 0.005), None)
        out["hit_t"] = hit["t"] if hit else None
        if hit:
            after = [r for r in rows if r["t"] >= hit["t"]]
            out["v_loss"] = hit["v"] - min(r["v"] for r in after[: max(1, len(after) // 3)])
            out["yaw_rate_max"] = max(abs(r["yaw_rate"]) for r in after)
        out["cone_rest_jitter_mm"] = 1000 * max(r["cone_d"] for r in rows[: max(1, len(rows) // 10)])
    else:
        s = [r for r in rows if r["t"] > rows[-1]["t"] - 2.0]
        v = st.mean(r["v"] for r in s)
        yr = st.mean(r["yaw_rate"] for r in s)
        d = st.mean(r["tire_angle"] for r in s)
        out["v"] = v
        out["yaw_rate"] = yr
        out["yaw_rate_kinematic"] = v * math.tan(d) / p.wheel_base
        out["radius"] = v / yr if abs(yr) > 1e-6 else None
        out["lat_acc"] = st.mean(r["ay"] for r in s)
        out["lat_acc_v_r"] = v * yr
        out["yaw_rate_sd"] = sd([r["yaw_rate"] for r in s])
        out["z_sd_mm"] = 1000 * sd([r["z"] for r in s])
        out["roll_max_deg"] = max(abs(r["roll"]) for r in rows)
    return {k: (round(v, 5) if isinstance(v, float) else v) for k, v in out.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ego", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frame", type=float, default=0.05)
    ap.add_argument("--sub", type=int, default=3)
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--label", default="")
    ap.add_argument("--only", default="")
    ap.add_argument("--solver", default="", help="TGS or PGS (default: Isaac's)")
    args = ap.parse_args()
    from dataclasses import fields

    from aib.control import GEAR_DRIVE, Command, State
    from aib.params import VehicleParams
    from aib.rig import EgoConfig

    cfg = EgoConfig.load(args.ego)
    vehicle, ecu = dict(cfg.vehicle), dict(cfg.ecu)
    physx_fields = {f.name for f in fields(VehicleParams)}
    for kv in args.set:
        k, v = kv.split("=", 1)
        (vehicle if k in physx_fields else ecu)[k] = (
            json.loads(v) if v[:1] in "-0123456789tf[" else v)
    cfg.vehicle, cfg.ecu = vehicle, ecu
    rig = cfg.load_rig()
    p = rig.vehicle
    rate = args.sub / args.frame
    label = args.label or f"{rate:.0f}hz"
    out = Path(args.out) / label
    out.mkdir(parents=True, exist_ok=True)
    os.environ.pop("DISPLAY", None)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "extra_args": ["--/crashreporter/url="]})
    import omni.physx
    import omni.usd
    from isaacsim.core.api import World

    from aib.isaac.vehicle import Vehicle, ensure_vehicle_scene

    dt = args.frame / args.sub
    world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt)
    stage = omni.usd.get_context().get_stage()
    world.scene.add_default_ground_plane()
    ensure_vehicle_scene(stage)
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    def box(path, center, size, dynamic_mass=None):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        xf = UsdGeom.XformCommonAPI(cube.GetPrim())
        xf.SetTranslate(Gf.Vec3d(*center))
        xf.SetScale(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        return cube

    for i, x in enumerate(CLEATS):
        box(f"/World/obstacles/cleat{i}", (x, LANES["bumps"], CLEAT[2] / 2),
            (CLEAT[0], CLEAT[1], CLEAT[2]))
    box("/World/obstacles/wall", (WALL_X + 0.1, LANES["wall"], 0.25), (0.2, 2.0, 0.5))
    cyl = UsdGeom.Cylinder.Define(stage, "/World/obstacles/cone")
    cyl.CreateRadiusAttr(CONE[0])
    cyl.CreateHeightAttr(CONE[1])
    cyl.CreateAxisAttr("Z")
    UsdGeom.XformCommonAPI(cyl.GetPrim()).SetTranslate(
        Gf.Vec3d(CONE_X, LANES["cone"], CONE[1] / 2))
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(cyl.GetPrim())
    UsdPhysics.MassAPI.Apply(cyl.GetPrim()).CreateMassAttr(CONE[2])
    scene_prim = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
    sapi = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    if args.solver:
        sapi.CreateSolverTypeAttr(args.solver)
    world.reset()
    world.play()
    physx = omni.physx.get_physx_interface()
    pose = ((0.0, 0.0, 0.02), (1.0, 0.0, 0.0, 0.0))
    veh = Vehicle(stage, "/World/ego", p, rig.make_component(), rig.box_center,
                  rig.box_dims, pose, truth_settings=rig.component.get("truth"))
    solver = sapi.GetSolverTypeAttr().Get()
    print("solver", solver, flush=True)
    summary = {"rate_hz": rate, "solver": solver, "frame": args.frame, "sub": args.sub,
               "set": args.set, "ecu_slip_offset": rig.ecu.est_slip_offset, "tests": {}}
    only = [t for t in args.only.split(",") if t]
    try:
        for test, (length, command) in TESTS.items():
            if only and test not in only:
                continue
            y = LANES.get(test, 0.0)
            start = ((0.0, y, pose[0][2]), pose[1])
            veh.teleport(start, physx)
            state = State()
            idle = Command(received=True, gear=GEAR_DRIVE)
            for _ in range(int(2.0 / args.frame)):
                for _ in range(args.sub):
                    veh.apply(idle, dt, state)
                    world.step(render=False)
                    veh.sample(physx, dt, state)
            x0, y0 = state.position[0], state.position[1]
            cmd = Command(received=True, gear=GEAR_DRIVE)
            rows, memo, t = [], {}, 0.0
            wall = time.monotonic()
            while t < length - 1e-9:
                c = command(t, state.velocity[0], memo)
                cmd.acceleration, cmd.velocity, cmd.tire_angle = c
                for _ in range(args.sub):
                    veh.apply(cmd, dt, state)
                    world.step(render=False)
                    veh.sample(physx, dt, state)
                    t += dt
                    w, qx, qy, qz = state.quat
                    roll = math.degrees(math.atan2(2 * (w * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)))
                    ws = veh.wheel_states or [None] * 4
                    row = {"t": round(t, 5), "v": state.velocity[0], "vy": state.velocity[1],
                           "x": state.position[0] - x0, "y": state.position[1] - y0,
                           "z": state.position[2], "roll": roll,
                           "yaw_rate": state.angular_velocity[2],
                           "ax": state.acceleration[0], "ay": state.acceleration[1],
                           "cmd_a": cmd.acceleration, "tire_angle": state.tire_angle,
                           "drive_sum": sum(veh.torques.drive), "brake_sum": sum(veh.torques.brake)}
                    for name, wst in zip(("fl", "fr", "rl", "rr"), ws):
                        row[f"slip_{name}"] = wst.slip if wst else 0.0
                        row[f"omega_{name}"] = wst.rotation_speed if wst else 0.0
                    row["pitch_rate"] = state.angular_velocity[1]
                    row["roll_rate"] = state.angular_velocity[0]
                    row["airborne"] = sum(1 for wst in ws if wst and not wst.on_ground)
                    if test == "cone":
                        tf = physx.get_rigidbody_transformation("/World/obstacles/cone")
                        cp = tf["position"]
                        row["cone_d"] = math.hypot(cp[0] - CONE_X, cp[1] - LANES["cone"])
                        row["cone_z"] = cp[2]
                    rows.append(row)
            with open(out / f"{test}.csv", "w", newline="") as f:
                wr = csv.DictWriter(f, fieldnames=list(rows[0]))
                wr.writeheader()
                wr.writerows(rows)
            m = measure(test, rows, p)
            m["wall_s"] = round(time.monotonic() - wall, 2)
            summary["tests"][test] = m
            print(test, json.dumps(m), flush=True)
    finally:
        (out / "summary.json").write_text(json.dumps(summary, indent=2))
        app.close()


if __name__ == "__main__":
    main()
