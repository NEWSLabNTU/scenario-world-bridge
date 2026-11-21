"""Smoke of the Isaac Lab environment (092 K2.2): CS2's ego and lead on a straight road over a
ground plane, N slots from N records, backend A, the shipped `ideal` component as a LoopBatch
(allow_loop: a check, not a bench). Steps Isaac Lab's loop with a constant acceleration
request and prints whether every slot's ego sped up and every lead drove off. Run in the
Isaac Lab overlay venv (scripts/isaaclab_overlay.sh), headless:

    python3 smoke_env.py --ego <aib ego.yaml> --vehicle <spec .yaml> --n 4 --steps 120
"""
import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
CASES = HERE.parents[1] / "scensim_sampler" / "examples" / "cases"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--ego", required=True)
    ap.add_argument("--vehicle", required=True, help="the car's spec (sim_vehicle_spec)")
    ap.add_argument("--n", type=int, default=4)
    ap.add_argument("--steps", type=int, default=120)
    a = ap.parse_args(argv)

    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=True).app

    import torch
    import yaml
    from aib.rig import EgoConfig

    sys.path.insert(0, str(HERE.parent))
    from scensim_batch.env import ScensimEnv, ScensimEnvCfg

    class StraightWorld:
        def lane_pose(self, lanes, s, offset):
            y = torch.tensor([3.5 * (int(l) - 34513) for l in lanes], dtype=torch.float64)
            return s, y + offset, torch.zeros_like(s)

        def route(self, x, y, yaw, length, points):
            k = torch.linspace(0, length, points, dtype=torch.float64)
            return torch.stack([x[..., None] + k * torch.cos(yaw)[..., None],
                                y[..., None] + k * torch.sin(yaw)[..., None]], -1)

        def speed_limit(self, x, y):
            return 15.0

    rig = EgoConfig.load(a.ego).load_rig()
    cfg = ScensimEnvCfg()
    cfg.scenario = str(CASES / "cs2_lead_brakes_wet_night.xosc")
    cfg.records = [{"values": {"ego_speed": 5.0 + i}, "seeds": {"scenario": i}}
                   for i in range(a.n)]
    cfg.world_factory = StraightWorld
    cfg.rig = rig
    cfg.vehicle = yaml.safe_load(Path(a.vehicle).read_text())
    cfg.component = "ideal"
    cfg.allow_loop = True
    cfg.provides = ("world knob illumination=night",)
    cfg.scene.num_envs = a.n
    env = ScensimEnv(cfg)
    env.reset()
    act = torch.zeros(a.n, 3)
    act[:, 0] = 1.0     # acceleration, m/s^2
    act[:, 1] = 20.0    # velocity limit
    v0 = None
    for k in range(a.steps):
        obs, *_ = env.step(act)
        sc = env.extras["scensim"]
        if v0 is None:
            v0 = sc["ego"]["v"].clone()
    lead = env.compiled.index("lead")
    out = {"n": a.n, "ego_v0": v0.tolist(), "ego_v": sc["ego"]["v"].tolist(),
           "lead_x": sc["x"][:, lead].tolist(), "lead_v": sc["v"][:, lead].tolist(),
           "ego_sped_up": bool((sc["ego"]["v"] > v0 + 0.5).all()),
           "lead_moved": bool((sc["x"][:, lead] > 36.5).all())}
    print(json.dumps(out), flush=True)
    env.close()
    app.close()


if __name__ == "__main__":
    main()
