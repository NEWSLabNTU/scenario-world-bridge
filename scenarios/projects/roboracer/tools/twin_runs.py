"""Roadmap 091 W23: the twin driven through the measurement protocol, with the stand-in drive
unit, on Isaac, no ROS. It makes two kinds of logs, one per run:
- the synthetic stand-in for the real car's logs: a perturbed copy, with sensor noise;
- the twin's own runs, to compare with them.

    python twin_runs.py --ego EGO.yaml --out LOGS [--perturb P.yaml --sealed DIR]
        [--only M1,M5] [--repetitions N] [--hz 240] [--seed S]

Each run writes, under LOGS/<run>/:
- `streams.npz`: the real car's topics as arrays, each with its own stamps:
  - `cmd_*`: what the protocol driver sent, at its command rate: the motor mode's index in
    protocol.MOTORS, the value in that mode's unit, and the servo position;
  - `core_*`: `sensors/core`, at 50 Hz: ERPM, motor current, duty, input voltage and
    tachometer, from the drive unit's telemetry;
  - `imu_*`: `sensors/imu/raw`, at 50 Hz: the body's specific force and angular rate;
  - `ref_*`: the external reference, at 100 Hz: x, y, z and yaw in the start's frame.
- `session.yaml`: the manoeuvre, its condition and repetition, and the session conditions (the
  battery's voltage, the floor, the tyres' age), the reference's kind and noise, and the streams'
  rates. With a perturbation it also holds the sealed truth's sha256, never the truth itself.

`--perturb` (yaml) is the deliberate perturbation:
- `vehicle`: aib VehicleParams fields (`mass`, `axle_tires`, ...);
- `component`: the drive unit's params;
- `noise`: the sensors' standard deviations (`ref_pos`, `ref_yaw`, `imu_accel`, `imu_gyro`,
  `erpm`, `current`).

It is copied, with the master seed, to `--sealed`/truth.yaml, a directory the fitting tool is
never given. Without `--perturb` the runs are the twin's own: the spec as it is, the component's
defaults (or the ego config's `component` params), and no noise. With `--fitted` (fit_twin.py's
result) they are the fitted twin's: its `vehicle` and `component`, and no noise.

Run it in Isaac's environment with aib on the path (aib's scripts/isaac_env.sh, then
PYTHONPATH=<aib>/src/aib), from any directory.
"""
import argparse
import hashlib
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "protocol"))
sys.path.insert(0, str(ROOT / "component" / "roboracer_vesc"))
import protocol as protomod  # noqa: E402

CORE_HZ, IMU_HZ, REF_HZ = 50.0, 50.0, 100.0
SETTLE_S = 1.0
G = 9.81
NOISE_KEYS = ("ref_pos", "ref_yaw", "imu_accel", "imu_gyro", "erpm", "current")


def run_seed(master, name):
    """A run's own seed: the master seed and the run's name, so a repetition's noise does not
    depend on which other runs were made."""
    return int.from_bytes(hashlib.sha256(f"{master}:{name}".encode()).digest()[:4], "little")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ego", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--perturb", default="")
    ap.add_argument("--fitted", default="",
                    help="a fit_twin.py result: its vehicle and component, no noise, nothing sealed")
    ap.add_argument("--sealed", default="")
    ap.add_argument("--protocol", default=str(ROOT / "protocol" / "protocol.yaml"))
    ap.add_argument("--only", default="")
    ap.add_argument("--repetitions", type=int, default=None)
    ap.add_argument("--hz", type=float, default=240.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--session", default="", help="session conditions (yaml) for every run")
    args = ap.parse_args()
    if bool(args.perturb) != bool(args.sealed):
        sys.exit("--perturb and --sealed go together: the truth is sealed apart from the logs")
    if args.perturb and args.fitted:
        sys.exit("--perturb or --fitted, not both")
    perturb = yaml.safe_load(Path(args.perturb).read_text()) if args.perturb else {}
    unknown = set(perturb) - {"vehicle", "component", "noise"}
    if unknown:
        sys.exit(f"--perturb: unknown sections {sorted(unknown)}")
    fitted = yaml.safe_load(Path(args.fitted).read_text()) if args.fitted else {}
    changes = perturb or {k: fitted.get(k) or {} for k in ("vehicle", "component")}
    noise = {k: float((perturb.get("noise") or {}).get(k, 0.0)) for k in NOISE_KEYS}
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    truth_sha = None
    if perturb:
        sealed = Path(args.sealed)
        if sealed.resolve() == out.resolve() or out.resolve() in sealed.resolve().parents:
            sys.exit("--sealed must not be inside --out: the fitting tool reads --out")
        sealed.mkdir(parents=True, exist_ok=True)
        text = yaml.safe_dump({"perturbation": perturb, "seed": args.seed}, sort_keys=True)
        (sealed / "truth.yaml").write_text(text)
        truth_sha = hashlib.sha256(text.encode()).hexdigest()
    session = yaml.safe_load(Path(args.session).read_text()) if args.session else {
        "battery_v": 7.9, "floor": "synthetic: the twin's floor", "tyre_age": "synthetic"}

    from dataclasses import fields

    from aib.params import VehicleParams
    from aib.rig import EgoConfig

    cfg = EgoConfig.load(args.ego)
    vehicle = dict(cfg.vehicle)
    vehicle.update(changes.get("vehicle") or {})
    known = {f.name for f in fields(VehicleParams)}
    bad = set(changes.get("vehicle") or {}) - known
    if bad:
        sys.exit(f"--perturb vehicle: unknown fields {sorted(bad)}")
    cfg.vehicle = vehicle
    rig = cfg.load_rig()
    p = rig.vehicle
    comp_params = dict((getattr(cfg, "component", None) or {}).get("params") or {})
    comp_params.update(changes.get("component") or {})

    doc = protomod.load(args.protocol)
    only = [m for m in args.only.split(",") if m]
    runs = protomod.runs(doc, only=only or None, repetitions=args.repetitions)
    cmd_period = 1.0 / float(doc.get("command_rate_hz", 50))

    os.environ.pop("DISPLAY", None)
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "extra_args": ["--/crashreporter/url="]})
    import omni.physx
    import omni.usd
    from isaacsim.core.api import World

    from aib.control import State
    from aib.isaac.vehicle import Vehicle, ensure_vehicle_scene
    from roboracer_vesc.component import RoboRacerVesc
    from sim_vehicle_ecu.signals import VescCommand

    dt = 1.0 / args.hz
    world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt)
    stage = omni.usd.get_context().get_stage()
    world.scene.add_default_ground_plane()
    ensure_vehicle_scene(stage)
    world.reset()
    world.play()
    physx = omni.physx.get_physx_interface()
    pose = ((0.0, 0.0, 0.02), (1.0, 0.0, 0.0, 0.0))
    component = RoboRacerVesc(rig.spec, comp_params, args.seed)
    veh = Vehicle(stage, "/World/ego", p, component, rig.box_center, rig.box_dims, pose,
                  interface="vesc")
    wall0 = time.monotonic()
    try:
        for run in runs:
            seed = run_seed(args.seed, run.name)
            rng = np.random.default_rng(seed)
            component.seed = seed  # the jitter's draws are the run's own
            veh.teleport(pose, physx)
            component.reset()
            state = State()
            idle = VescCommand("current", 0.0, None, 0.0)
            for _ in range(int(round(SETTLE_S * args.hz))):
                veh.apply(idle._replace(age=math.inf), dt, state)
                world.step(render=False)
                veh.sample(physx, dt, state)
            x0, y0, z0 = state.position
            yaw0 = _yaw(state.quat)
            streams = {k: [] for k in ("cmd", "core", "imu", "ref")}
            t, next_cmd, next_core, next_imu, next_ref = 0.0, 0.0, 0.0, 0.0, 0.0
            last_cmd, cmd_t = None, 0.0
            n = int(round(run.length * args.hz))
            for i in range(n):
                if t >= next_cmd - 1e-9:
                    motor, value, servo = protomod.command(run, t, p.max_steer_angle)
                    last_cmd, cmd_t = (motor, value, servo), t
                    streams["cmd"].append((t, protomod.MOTORS.index(motor), value, servo))
                    next_cmd += cmd_period
                motor, value, servo = last_cmd
                veh.apply(VescCommand(motor, value, servo, t - cmd_t), dt, state)
                world.step(render=False)
                veh.sample(physx, dt, state)
                t = (i + 1) * dt
                tel = veh.torques.telemetry or {}
                if t >= next_core - 1e-9:
                    streams["core"].append((
                        t, tel.get("erpm", 0.0) + rng.normal(0.0, noise["erpm"]) * (noise["erpm"] > 0),
                        tel.get("current_motor", 0.0) + rng.normal(0.0, noise["current"]) * (noise["current"] > 0),
                        tel.get("duty", 0.0), tel.get("voltage_input", 0.0),
                        tel.get("tachometer", 0)))
                    next_core += 1.0 / CORE_HZ
                if t >= next_imu - 1e-9:
                    g_body = _rot_inv(state.quat, (0.0, 0.0, -G))
                    f = [a - gb for a, gb in zip(state.acceleration, g_body)]
                    f = [v + (rng.normal(0.0, noise["imu_accel"]) if noise["imu_accel"] > 0 else 0.0)
                         for v in f]
                    w = [v + (rng.normal(0.0, noise["imu_gyro"]) if noise["imu_gyro"] > 0 else 0.0)
                         for v in state.angular_velocity]
                    streams["imu"].append((t, *f, *w))
                    next_imu += 1.0 / IMU_HZ
                if t >= next_ref - 1e-9:
                    # in the start's frame: x ahead, y left, yaw from the start's heading
                    dx, dy = state.position[0] - x0, state.position[1] - y0
                    c, s = math.cos(yaw0), math.sin(yaw0)
                    rx, ry = c * dx + s * dy, -s * dx + c * dy
                    yaw = math.atan2(math.sin(_yaw(state.quat) - yaw0),
                                     math.cos(_yaw(state.quat) - yaw0))
                    if noise["ref_pos"] > 0:
                        rx += rng.normal(0.0, noise["ref_pos"])
                        ry += rng.normal(0.0, noise["ref_pos"])
                    if noise["ref_yaw"] > 0:
                        yaw += rng.normal(0.0, noise["ref_yaw"])
                    streams["ref"].append((t, rx, ry, state.position[2] - z0, yaw))
                    next_ref += 1.0 / REF_HZ
            d = out / run.name
            d.mkdir(parents=True, exist_ok=True)
            arrays = {}
            for k, cols in (("cmd", ("t", "mode", "value", "servo")),
                            ("core", ("t", "erpm", "current_motor", "duty", "voltage_input",
                                      "tachometer")),
                            ("imu", ("t", "ax", "ay", "az", "wx", "wy", "wz")),
                            ("ref", ("t", "x", "y", "z", "yaw"))):
                a = np.asarray(streams[k], dtype=float)
                for j, col in enumerate(cols):
                    arrays[f"{k}_{col}"] = a[:, j]
            np.savez_compressed(d / "streams.npz", **arrays)
            sess = {
                "format": "scensim 091 W23 run, version 1",
                "manoeuvre": run.manoeuvre, "condition": run.condition,
                "repetition": run.repetition, "session": session,
                "reference": {"kind": "synthetic motion capture" if perturb else "twin truth",
                              "noise_m": noise["ref_pos"], "noise_rad": noise["ref_yaw"]},
                "rates_hz": {"cmd": 1.0 / cmd_period, "core": CORE_HZ, "imu": IMU_HZ,
                             "ref": REF_HZ},
                "source": ("synthetic: a perturbed twin" if perturb else
                           f"the twin, fitted ({args.fitted})" if fitted else "the twin"),
                "physics_hz": args.hz, "protocol_version": doc.get("version"),
            }
            if truth_sha:
                sess["truth_sha256"] = truth_sha
            (d / "session.yaml").write_text(yaml.safe_dump(sess, sort_keys=False))
            print(f"run {run.name} {run.length:.1f} s", flush=True)
    finally:
        print(json.dumps({"runs": len(runs), "wall_s": round(time.monotonic() - wall0, 1)}),
              flush=True)
        app.close()


def _yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _rot_inv(q, v):
    """v (world) in the body frame of quaternion q (w, x, y, z)."""
    w, x, y, z = q
    # the rotation matrix's transpose times v
    r = ((1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)),
         (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)),
         (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)))
    return tuple(sum(r[i][j] * v[j] for j in range(3)) for i in range(3))


if __name__ == "__main__":
    main()
