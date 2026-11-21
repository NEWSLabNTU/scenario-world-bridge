"""Smoke and cost of backend A (092 K2.2): N copies of aib's PhysX vehicle on a ground plane,
each driven by the same actuation, stepped for a few seconds. Prints that every car moved as
one, and the time per step and per car. Run in Isaac's Python with aib and sim_vehicle_ecu on
the path, headless:

    python3 smoke_physx_vehicle.py --ego <aib ego.yaml> --n 1 4 16 64 --seconds 4
"""
import argparse
import json
import os
import sys
import time


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--ego", required=True)
    ap.add_argument("--n", type=int, nargs="+", default=[1, 4, 16])
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--dt", type=float, default=1 / 240)
    ap.add_argument("--torque", type=float, default=150.0, help="N m per driven wheel")
    a = ap.parse_args(argv)

    from aib.rig import EgoConfig
    rig = EgoConfig.load(a.ego).load_rig()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "extra_args": ["--/crashreporter/url="]
                         + os.environ.get("ISAAC_KIT_ARGS", "").split()})
    import numpy as np
    import omni.physx
    import omni.usd
    from isaacsim.core.api import World

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from scensim_batch.physx_vehicle import PhysxVehicleBackend

    rows = []
    for n in a.n:
        world = World(stage_units_in_meters=1.0, physics_dt=a.dt, rendering_dt=a.dt)
        world.scene.add_default_ground_plane()
        stage = omni.usd.get_context().get_stage()
        physx = omni.physx.get_physx_interface()
        be = PhysxVehicleBackend(world, stage, physx, rig, n, spacing=60.0)
        world.reset()
        world.play()
        z = np.zeros(n)
        be.reset(np.ones(n, bool), z, z, z, z)
        drive = np.zeros((n, 4))
        drive[:, :] = a.torque
        act = {"drive": drive, "brake": np.zeros((n, 4)), "tire_angle": np.zeros(n)}
        steps = int(round(a.seconds / a.dt))
        for _ in range(10):   # warm
            be.apply(act)
            be.advance(a.dt)
        t0 = time.perf_counter()
        for _ in range(steps):
            be.apply(act)
            be.advance(a.dt)
        per_step = (time.perf_counter() - t0) / steps
        st = be.state()
        x = st["x"].numpy()
        rows.append({"n": n, "ms_per_step": per_step * 1e3, "us_per_car": per_step * 1e6 / n,
                     "x_min": float(x.min()), "x_max": float(x.max()),
                     "v_mean": float(st["v"].numpy().mean())})
        print(json.dumps(rows[-1]), flush=True)
        world.stop()
        world.clear_instance()
        omni.usd.get_context().new_stage()
    app.close()


if __name__ == "__main__":
    main()
