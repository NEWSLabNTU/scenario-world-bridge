"""Backend A of the batch (roadmap 092 K2.2, batch-executor.md section 5): N copies of the
same PhysX vehicle seats run (aib's `Vehicle`, built from the car's rig), one per slot, each in
its own place on the stage. It is the same car as seats, physics and tire model alike, so a
batch run on it can be checked against a seats run (K2.4). It is not vectorised: Isaac Sim's
physics tensor API has no vehicle view, so each car's actuation (four drive and brake
torques, the steer angles) is written and its wheel states read one car at a time. It says so
(`vectorised = False`), and a throughput bench refuses it as it refuses a LoopBatch; backend B
(an articulation car) is the fast one.

The backend's contract is core.BatchRun's: reset(mask, x, y, yaw, speed), apply(actuation),
advance(dt), state(). Import after Isaac Sim's `SimulationApp`.

Slots stand apart on a grid (`spacing`, m), so cars of different slots never touch. A slot's
poses are the scenario's, offset by its origin; state() takes the origin off again.
"""
import math

import numpy as np
import torch


class PhysxVehicleBackend:
    vectorised = False   # one car at a time through PhysX's vehicle interface (no vehicle view)

    def __init__(self, world, stage, physx, rig, n, spacing=200.0, root="/World/envs",
                 z=0.0, velocity_set=True, origins=None):
        """`world`: what `advance` steps (None for a host that owns the physics loop and calls
        actuate, its own step, then sample); `origins` [N, 2]: each slot's place (Isaac Lab's
        env origins), else a grid at `spacing`."""
        from aib.isaac.vehicle import Vehicle, ensure_vehicle_scene
        from aib.control import State

        self.world, self.stage, self.physx, self.rig, self.n = world, stage, physx, rig, n
        self.z, self.velocity_set = z, velocity_set
        side = max(1, math.ceil(math.sqrt(n)))
        self.origins = (np.asarray(origins, dtype=np.float64)[:, :2] if origins is not None else
                        np.array([((i % side) * spacing, (i // side) * spacing)
                                  for i in range(n)], dtype=np.float64))
        ensure_vehicle_scene(stage)
        self.cars, self.states = [], []
        for i in range(n):
            ox, oy = self.origins[i]
            path = f"{root}/env_{i}/ego"
            self.cars.append(Vehicle(stage, path, rig.vehicle, rig.make_component(),
                                     rig.box_center, rig.box_dims, ((ox, oy, z), (1, 0, 0, 0)),
                                     ground_clearance=rig.ground_clearance))
            self.states.append(State())
        self.act = None
        self.timing = {"actuate": 0.0, "physics": 0.0, "sample": 0.0, "steps": 0}

    # --- the contract ------------------------------------------------------------------------

    def reset(self, mask, x, y, yaw, speed):
        from pxr import Gf, UsdPhysics

        for i in np.flatnonzero(np.asarray(mask, bool)):
            ox, oy = self.origins[i]
            h = float(yaw[i])
            pose = ((ox + float(x[i]), oy + float(y[i]), self.z),
                    (math.cos(h / 2), 0.0, 0.0, math.sin(h / 2)))
            car = self.cars[i]
            car.teleport(pose, self.physx)
            v = float(speed[i])
            if v and self.velocity_set:
                # the Init speed: the body along its heading, the wheels rolling with it
                body = UsdPhysics.RigidBodyAPI(self.stage.GetPrimAtPath(car.path))
                body.GetVelocityAttr().Set(Gf.Vec3f(v * math.cos(h), v * math.sin(h), 0.0))
                spin = v / self.rig.vehicle.wheel_radius
                for wp in car.wheel_paths:
                    try:
                        self.physx.set_wheel_rotation_speed(wp, spin)
                    except Exception:   # noqa: BLE001 - not simulated yet
                        pass
            self.states[i].__init__()

    def apply(self, act, velocity=None, gear=None):
        """The coming step's actuation, with each request's velocity and gear for aib's
        sticky-tire release (core.request_motion); without them, 0 and drive, which holds a
        car whose request is to stop."""
        from sim_vehicle_ecu.signals import GEAR_DRIVE
        self.act = act
        self.velocity = np.zeros(self.n) if velocity is None else np.asarray(velocity, float)
        self.gear = (np.full(self.n, GEAR_DRIVE) if gear is None else np.asarray(gear))

    def advance(self, dt):
        """actuate, the world's step, sample: for a host that leaves the physics to us."""
        import time
        t0 = time.perf_counter()
        self.actuate(dt)
        t1 = time.perf_counter()
        self.world.step(render=False)
        t2 = time.perf_counter()
        self.sample(dt)
        t3 = time.perf_counter()
        tm = self.timing   # where a step's time goes (K2.5's split)
        tm["actuate"] += t1 - t0
        tm["physics"] += t2 - t1
        tm["sample"] += t3 - t2
        tm["steps"] += 1

    def actuate(self, dt):
        """Every car's actuation for the coming physics step (a host that owns the loop calls
        this, steps the engine, then sample)."""
        from sim_vehicle_ecu.signals import Actuation
        a = self.act
        drive, brake = np.asarray(a["drive"]), np.asarray(a["brake"])
        angle = np.asarray(a["tire_angle"])
        for i, (car, st) in enumerate(zip(self.cars, self.states)):
            car.actuate(Actuation(tuple(drive[i].tolist()), tuple(brake[i].tolist()),
                                  float(angle[i]), None), dt, st,
                        float(self.velocity[i]), int(self.gear[i]))

    def sample(self, dt):
        """Every car read back after a physics step."""
        for car, st in zip(self.cars, self.states):
            car.sample(self.physx, dt, st)

    def state(self):
        n = self.n
        x, y, yaw, v = (np.zeros(n) for _ in range(4))
        yr, ax, ay = (np.zeros(n) for _ in range(3))
        ws = np.zeros((n, 4))
        for i, (car, st) in enumerate(zip(self.cars, self.states)):
            px, py, _ = st.position
            w, qx, qy, qz = st.quat
            x[i], y[i] = px - self.origins[i][0], py - self.origins[i][1]
            yaw[i] = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            v[i] = st.velocity[0]
            yr[i], ax[i], ay[i] = st.angular_velocity[2], st.acceleration[0], st.acceleration[1]
            if car.wheel_states is not None:
                ws[i] = [w_.rotation_speed for w_ in car.wheel_states]
        t = lambda a: torch.as_tensor(a, dtype=torch.float64)   # noqa: E731
        return {"x": t(x), "y": t(y), "yaw": t(yaw), "v": t(v), "yaw_rate": t(yr),
                "accel_x": t(ax), "accel_y": t(ay), "wheel_speeds": t(ws)}
