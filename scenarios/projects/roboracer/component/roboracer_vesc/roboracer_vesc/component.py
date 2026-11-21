"""A stand-in RoboRacer drive unit: a VESC driving a sensored brushless motor through the
Slash 4x4's spur, centre and axle gears, and a steering servo (scensim roadmap 091 W23).

It is an example of a user's control component (sim_vehicle_ecu's component interface), made so
that the synthetic stand-in for real logs (W23) has parameters to perturb and the fitting tool
(W10) has parameters to recover. It is not a model of any particular car, and scensim does not
ship it: 091 leaves the car's model to its user. Pure: no simulator, no I/O.

It is written once, in array form over N cars (`RoboRacerVescBatch`, the batched form of
sim_vehicle_ecu.batch, scensim 092 S3), and `RoboRacerVesc` is that at one car
(`batch.ScalarOf`): the scalar and the batched car are one code. The arithmetic is elementwise
+ - * /, abs, min/max and where, in the order the scalar form had, so N cars equal N single
cars bit for bit; the wheels' mean is written out, ((w0 + w1) + w2) + w3, not `sum()` (Python
3.12's float `sum()` is compensated; roboracer-indoor-twin.md 2.6).

Commands are the ``vesc`` ones (``VescCommand``):
- `speed` (ERPM): the VESC's speed loop. It is a PI on the ERPM error, its output a motor
  current within the limits.
- `current` (A): the motor current, within the limits.
- `brake` (A): a braking current, as a brake torque on every wheel.
- `duty` (-1 to 1): the motor's voltage over the battery's, against its back-EMF through its
  resistance.
- `position`: not modelled; it gives no current.

A motor command older than `timeout` releases the motor (a real VESC's timeout). The servo
position becomes the bicycle tire angle through the driver's gain and offset (f1tenth's
`steering_angle_to_servo_gain`, `_offset`), and the servo moves at most `servo_rate` rad/s.
Commands take effect `latency` seconds after they arrive. `latency_jitter` (s, the standard
deviation) draws each command's own extra delay from the seed. That is a perturbation the
fitting tool's model holds as a single latency, so W23 can see how a fit degrades on it.

Each car keeps its own queue of pending commands (a ring buffer per car, drained in arrival
order) and its own `random.Random(seed)`, drawn in the same order as a single car draws, so car
i of a batch is the single car with seed i.

The motor's speed is the wheels' mean, as a sensored VESC sees it through the differentials.
The telemetry is what `sensors/core` carries: `erpm`, `current_motor`, `duty`, `voltage_input`
and `tachometer`.
"""
import random

import numpy as np

from sim_vehicle_ecu import batch as ecu_batch
from sim_vehicle_ecu.signals import MOTOR_MODES

# The defaults: a 1/10 car on f1tenth_system's humble-devel `vesc.yaml` and a 3500 kV motor
DEFAULTS = {
    "erpm_per_mps": 4614.0,         # f1tenth's speed_to_erpm_gain: ERPM per m/s at the wheels
    "erpm_per_motor_rad_s": 9.549,  # ERPM per rad/s of the motor: pole pairs x 60 / (2 pi), 1 pair
    "kt": 0.00273,                  # N m per A at the motor: 60 / (2 pi kV), kV 3500
    "current_max": 40.0,            # A, the motor current's limit, both ways
    "brake_max": 40.0,              # A, the braking current's limit
    "speed_kp": 0.0045,             # A per ERPM of error
    "speed_ki": 0.012,              # A per ERPM s
    "resistance": 0.03,             # ohm, for duty commands
    "battery": 7.9,                 # V, a 2S pack mid-charge
    "servo_gain": -1.2135,          # servo position per rad of tire angle
    "servo_offset": 0.5304,         # servo position at a straight tire
    "servo_rate": 5.0,              # rad/s at the wheels
    "latency": 0.02,                # s, from the command's arrival to its effect
    "latency_jitter": 0.0,          # s, standard deviation of each command's extra delay
    "timeout": 0.5,                 # s, a motor command older than this releases the motor
}
QUEUE = 256   # pending commands per car: latency / dt of them, with room
SPEED, CURRENT, BRAKE, DUTY = (MOTOR_MODES.index(m) for m in ("speed", "current", "brake", "duty"))
NONE = -1


class RoboRacerVescBatch:
    commands = "vesc"
    driven = (True,) * 4  # the Slash 4x4: the centre gear drives both axles
    truth = ()
    array_api, dtype = "numpy", np.float64
    per_slot_params = True   # every param an array [N]: a fit's candidates, a varied car

    def __init__(self, vehicle: dict, params: dict, seeds):
        unknown = set(params) - set(DEFAULTS)
        if unknown:
            raise ValueError(f"unknown params {sorted(unknown)}; known: {sorted(DEFAULTS)}")
        n = len(seeds)
        # each param per car, [N]: a number (one car's, or every car's) or an array [N]
        self.p = {k: np.broadcast_to(np.asarray(params.get(k, d), dtype=np.float64), (n,)).copy()
                  for k, d in DEFAULTS.items()}
        g = vehicle["geometry"]
        self.r = float(g["wheel_radius"])
        self.max_steer = float(g["max_steer_angle"])
        # the gearing: wheel rad/s per motor rad/s, from the ERPM per m/s
        motor_rad_s_per_mps = self.p["erpm_per_mps"] / self.p["erpm_per_motor_rad_s"]
        self.gear = motor_rad_s_per_mps * self.r  # motor rad/s per wheel rad/s
        self.seeds = [int(s) for s in seeds]
        n, q = len(self.seeds), QUEUE
        self.n = n
        self.rngs = [random.Random(s) for s in self.seeds]
        # the queue: (effective time, arrival, command) per car, as a ring buffer
        self.q_eff, self.q_arr = np.zeros((n, q)), np.zeros((n, q))
        self.q_motor = np.full((n, q), NONE, np.int64)
        self.q_value, self.q_servo = np.zeros((n, q)), np.full((n, q), np.nan)
        self.head, self.size = np.zeros(n, np.int64), np.zeros(n, np.int64)
        self.motor, self.last_motor = np.full(n, NONE, np.int64), np.full(n, NONE, np.int64)
        self.has_last = np.zeros(n, bool)
        for name in ("t", "integral", "tire_angle", "tachometer", "current", "value",
                     "last_value", "last_age"):
            setattr(self, name, np.zeros(n))
        for name in ("servo", "arrived", "last_servo"):
            setattr(self, name, np.full(n, np.nan))
        self.debug = {}

    def reset_batch(self, mask):
        m = np.asarray(mask, bool)
        for name in ("t", "integral", "tire_angle", "tachometer", "current", "value",
                     "last_value", "last_age"):
            setattr(self, name, np.where(m, 0.0, getattr(self, name)))
        for name in ("servo", "arrived", "last_servo"):
            setattr(self, name, np.where(m, np.nan, getattr(self, name)))
        self.motor = np.where(m, NONE, self.motor)
        self.last_motor = np.where(m, NONE, self.last_motor)
        self.has_last = np.where(m, False, self.has_last)
        self.head, self.size = np.where(m, 0, self.head), np.where(m, 0, self.size)
        for i in np.flatnonzero(m):
            self.rngs[i] = random.Random(self.seeds[i])

    # --- commands ------------------------------------------------------------------------

    def _arrive(self, c):
        """Commands the vehicle interface hands over, queued until their latency has passed. A
        command is a new one when its motor command or servo position changed, or its age fell
        (a new message with the same values)."""
        valid = c.get("valid", np.ones(self.n, bool))
        motor, value, servo, age = c["motor"], c["value"], c["servo"], c["age"]
        same_servo = (servo == self.last_servo) | (np.isnan(servo) & np.isnan(self.last_servo))
        same = (self.has_last & (motor == self.last_motor) & (value == self.last_value)
                & same_servo & (age >= self.last_age))
        self.last_motor = np.where(valid, motor, self.last_motor)
        self.last_value = np.where(valid, value, self.last_value)
        self.last_servo = np.where(valid, servo, self.last_servo)
        self.last_age = np.where(valid, age, self.last_age)
        self.has_last = self.has_last | valid
        new = valid & ~same
        if not new.any():
            return
        arrived = self.t - np.where(np.isinf(age), 0.0, age)
        delay = self.p["latency"].copy()
        lat, jit = self.p["latency"], self.p["latency_jitter"]
        for i in np.flatnonzero(new & (jit > 0)):   # each car's own generator, in its order
            delay[i] = max(0.0, float(lat[i]) + self.rngs[i].gauss(0.0, float(jit[i])))
        idx = np.flatnonzero(new)
        if np.any(self.size[idx] >= QUEUE):
            raise RuntimeError(f"roboracer_vesc: more than {QUEUE} commands pending in a car")
        slot = (self.head[idx] + self.size[idx]) % QUEUE
        self.q_eff[idx, slot] = arrived[idx] + delay[idx]
        self.q_arr[idx, slot] = arrived[idx]
        self.q_motor[idx, slot] = motor[idx]
        self.q_value[idx, slot] = value[idx]
        self.q_servo[idx, slot] = servo[idx]
        self.size[idx] += 1

    def _due(self):
        rows = np.arange(self.n)
        for _ in range(QUEUE):
            h = self.head
            due = (self.size > 0) & (self.q_eff[rows, h] <= self.t + 1e-12)
            if not due.any():
                return
            self.arrived = np.where(due, self.q_arr[rows, h], self.arrived)
            self.motor = np.where(due, self.q_motor[rows, h], self.motor)
            self.value = np.where(due, self.q_value[rows, h], self.value)
            self.servo = np.where(due, self.q_servo[rows, h], self.servo)
            self.head = np.where(due, (h + 1) % QUEUE, h)
            self.size = np.where(due, self.size - 1, self.size)

    # --- the step ------------------------------------------------------------------------

    def step_batch(self, commands, sensors, dt):
        p = self.p
        self.t = self.t + dt
        self._arrive(commands)
        self._due()
        w = np.where(sensors["valid"][:, None], sensors["wheel_speeds"], 0.0)
        wheel = (((w[:, 0] + w[:, 1]) + w[:, 2]) + w[:, 3]) / 4.0  # rad/s, written out (2.6)
        motor = wheel * self.gear  # rad/s
        erpm = motor * p["erpm_per_motor_rad_s"]
        mode = self.motor
        age = np.where(mode != NONE, self.t - self.arrived, np.inf)
        released = (mode == NONE) | (age > p["timeout"])
        zero = np.zeros(self.n)
        # the speed loop
        err = self.value - erpm
        speed = ~released & (mode == SPEED)
        integral = np.where(speed, self.integral + err * dt, self.integral)
        # anti-windup: the integral holds what keeps the output within the limit
        imax = p["current_max"] / np.maximum(p["speed_ki"], 1e-12)
        integral = np.where(speed, np.maximum(-imax, np.minimum(imax, integral)), integral)
        self.integral = np.where(released, 0.0, integral)
        emf = motor * p["kt"]  # V: the back-EMF, kt w
        current = np.where(speed, p["speed_kp"] * err + p["speed_ki"] * self.integral, zero)
        current = np.where(~released & (mode == CURRENT), self.value, current)
        current = np.where(~released & (mode == DUTY),
                           (self.value * p["battery"] - emf) / p["resistance"], current)
        brake_current = np.where(~released & (mode == BRAKE),
                                 np.minimum(np.abs(self.value), p["brake_max"]), zero)
        current = np.maximum(-p["current_max"], np.minimum(p["current_max"], current))
        self.current = current
        torque = current * p["kt"] * self.gear  # N m at the wheels, all four
        drive = np.repeat((torque / 4.0)[:, None], 4, axis=1)
        brake = np.repeat((brake_current * p["kt"] * self.gear / 4.0)[:, None], 4, axis=1)
        # the servo
        has = ~np.isnan(self.servo)
        target = (self.servo - p["servo_offset"]) / p["servo_gain"]
        target = np.maximum(-self.max_steer, np.minimum(self.max_steer, target))
        step = p["servo_rate"] * dt
        move = np.maximum(-step, np.minimum(step, target - self.tire_angle))
        self.tire_angle = np.where(has, self.tire_angle + move, self.tire_angle)
        self.tachometer = self.tachometer + np.abs(erpm) * dt / 60.0
        duty = (motor * p["kt"] + current * p["resistance"]) / p["battery"]
        telemetry = {"erpm": erpm, "current_motor": current,
                     "duty": np.maximum(-1.0, np.minimum(1.0, duty)),
                     "voltage_input": p["battery"].copy(),
                     "tachometer": np.floor(self.tachometer).astype(np.int64)}
        self.debug = {"erpm": erpm, "current": current, "brake_current": brake_current,
                      "mode": mode}
        return {"drive": drive, "brake": brake, "tire_angle": self.tire_angle.copy(),
                "telemetry": telemetry}


class RoboRacerVesc(ecu_batch.ScalarOf):
    """The single car: the batch at one slot. The constructor is (car, params, seed), as
    before; `p`, `gear`, `r` and `debug` read the one car's, as floats."""
    batch = RoboRacerVescBatch

    @property
    def p(self):
        return {k: float(v[0]) for k, v in self._b.p.items()}

    @property
    def gear(self):
        return float(self._b.gear[0])

    @property
    def r(self):
        return self._b.r

    @property
    def debug(self):
        d = self._b.debug
        if not d:
            return {}
        m = int(d["mode"][0])
        return {"erpm": float(d["erpm"][0]), "current": float(d["current"][0]),
                "brake_current": float(d["brake_current"][0]),
                "mode": None if m == NONE else MOTOR_MODES[m]}
