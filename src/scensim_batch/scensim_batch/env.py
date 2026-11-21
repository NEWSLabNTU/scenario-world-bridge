"""The batch as an Isaac Lab environment (roadmap 092 K2.2; batch-executor.md section 1): a
DirectRLEnv over N slots of one compiled scenario, each slot one of its sample records, the
ego through its vectorised control component on an engine backend, SSv2's default vehicles
around it. Rewards and terminations are the user's: subclass and override `_get_rewards` and
`_get_dones`; the observation and `extras["scensim"]` carry what they need.

    cfg = ScensimEnvCfg(scenario="cs2.xosc", records=records, world_factory=..., rig=rig,
                        component="roboracer_vesc", ...)
    env = ScensimEnv(cfg)

Isaac Lab owns the physics loop: before each physics step `_apply_action` reads the cars of the
step before (`sample`) and actuates them through the component (`BatchRun.before_physics`);
after the last, `_get_observations` reads them and moves the NPCs (`after_physics`). Isaac Lab
v3.0.0-EA (scripts/isaaclab_overlay.sh). Not exercised yet on 139's GPU: K2.2's smoke.
"""
import torch

from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from . import compile as compilemod
from . import core

# the action, per command type, as the component reads it (S3's batch commands)
ACTIONS = {"autoware": ("acceleration", "velocity", "tire_angle"),
           "ackermann": ("speed", "acceleration", "steering_angle"),
           "vesc": ("value", "servo")}


@configclass
class ScensimEnvCfg(DirectRLEnvCfg):
    scenario: str = ""
    records: list = []
    world_factory: object = None     # callable() -> the world adapter (lane_pose, route, speed_limit)
    rig: object = None               # aib's rig: the car
    component: str = "ideal"         # sim_vehicle_ecu component name (vectorised, S3)
    component_params: object = None  # one dict, or a list of N (per slot)
    vehicle: dict = {}               # the spec document the component is built for
    provides: tuple = ()             # the verdict's conditions this env meets
    vesc_mode: str = "speed"         # a vesc component's motor command mode
    allow_loop: bool = False         # accept a LoopBatch component (checks only, never a bench)
    decimation: int = 4
    episode_length_s: float = 30.0
    # backend A's PhysX vehicles need PhysX's CPU pipeline: the direct-GPU one refuses their
    # constraints and the CPU velocity setters (as aib's runner, which uses the CPU). Scene
    # queries on: a PhysX vehicle finds the ground by raycasting each wheel, and Isaac Lab turns
    # them off by default; without them every wheel was off the ground (K2.2's env smoke: the
    # steer and the wheel spin reached PhysX, the tire force stayed 0, every ego coasted to
    # rest on its body). Fabric stays Isaac Lab's default: on and off measured the same.
    sim: SimulationCfg = SimulationCfg(dt=1 / 240, device="cpu",
                                       enable_scene_query_support=True)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=1, env_spacing=200.0,
                                                     replicate_physics=False)
    action_space: int = 3
    observation_space: int = 8


class ScensimEnv(DirectRLEnv):
    cfg: ScensimEnvCfg

    def __init__(self, cfg: ScensimEnvCfg, render_mode=None, **kwargs):
        n = len(cfg.records)
        if n != cfg.scene.num_envs:
            raise ValueError(f"{n} records for {cfg.scene.num_envs} environments: one per slot")
        super().__init__(cfg, render_mode, **kwargs)

    # --- the scene ----------------------------------------------------------------------------

    def _setup_scene(self):
        import omni.physx
        import omni.usd
        from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
        from sim_vehicle_ecu import batch as ecu_batch

        from .physx_vehicle import PhysxVehicleBackend

        cfg = self.cfg
        spawn_ground_plane("/World/ground", GroundPlaneCfg())
        world = cfg.world_factory()
        self.compiled = compilemod.compile_scenario(cfg.scenario, cfg.records, world,
                                                    provides=cfg.provides)
        n = self.compiled.n
        seeds = [int(s or 0) for s in self.compiled.seeds]
        component = ecu_batch.load_batch(cfg.component, cfg.vehicle, cfg.component_params, seeds)
        self.kind = component.commands
        self.backend = PhysxVehicleBackend(None, omni.usd.get_context().get_stage(),
                                           omni.physx.get_physx_interface(), cfg.rig, n,
                                           origins=self.scene.env_origins.cpu().numpy())
        self.run = core.BatchRun(self.compiled, world, self.backend, component,
                                 allow_loop=cfg.allow_loop)
        self.commands = None
        self.stepped = False
        self.obs = None

    # --- the step -----------------------------------------------------------------------------

    def _pre_physics_step(self, actions: torch.Tensor):
        """The policy's actions as the component's commands (ACTIONS[kind], in order)."""
        a = actions.detach().to("cpu", torch.float64).numpy()
        n = a.shape[0]
        import numpy as np
        fields = ACTIONS[self.kind]
        c = {f: a[:, i].copy() for i, f in enumerate(fields)}
        c["valid"] = np.ones(n, bool)
        if self.kind == "autoware":
            c["gear"] = np.full(n, 2, dtype=np.int64)   # drive
        elif self.kind == "ackermann":
            for f in ("jerk", "steering_angle_velocity", "age"):
                c[f] = np.zeros(n)
        elif self.kind == "vesc":
            from sim_vehicle_ecu.signals import MOTOR_MODES
            c["motor"] = np.full(n, MOTOR_MODES.index(self.cfg.vesc_mode), dtype=np.int64)
            c["age"] = np.zeros(n)
        self.commands = c

    def _apply_action(self):
        dt = self.physics_dt
        if self.stepped:          # the step before this one: the cars read back
            self.backend.sample(dt)
        self.run.before_physics(self.commands, dt)
        self.backend.actuate(dt)
        self.stepped = True

    def _get_observations(self):
        dt = self.physics_dt
        if self.stepped:
            self.backend.sample(dt)
            self.stepped = False
        self.obs = self.run.after_physics(self.step_dt)
        ego = self.obs["ego"]
        o = torch.stack([ego["v"], ego["yaw_rate"], ego["accel_x"], ego["accel_y"],
                         ego["x"], ego["y"], torch.cos(ego["yaw"]), torch.sin(ego["yaw"])], -1)
        self.extras["scensim"] = self.obs
        return {"policy": o.to(self.device, torch.float32)}

    # --- the user's ---------------------------------------------------------------------------

    def _get_rewards(self):
        return torch.zeros(self.num_envs, device=self.device)

    def _get_dones(self):
        """Terminated: the storyboard's exitSuccess or exitFailure (extras['scensim']
        ['outcome']: 1 success, 2 failure); truncated: the episode length, which is SSv2's
        test runner timeout's role. A slot whose storyboard completed or stopped without an
        exit runs on to it, as on seats."""
        timeout = self.episode_length_buf >= self.max_episode_length - 1
        out = self.run.storyboard.outcome if self.run.storyboard is not None else None
        term = (out != 0).to(timeout.device) if out is not None else torch.zeros_like(timeout)
        return term, timeout

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        mask = torch.zeros(self.num_envs, dtype=torch.bool)
        mask[torch.as_tensor(env_ids, dtype=torch.long).cpu()] = True
        self.run.reset(mask)
        self.stepped = False
