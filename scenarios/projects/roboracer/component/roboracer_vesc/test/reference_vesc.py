"""118's scalar RoboRacerVesc as W23 landed it (af34e2a3): its code verbatim, the module
docstring dropped and the class renamed. The reference the batched form is tested against."""
import math
import random
from collections import deque

from sim_vehicle_ecu.signals import Actuation, VescCommand

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


class ReferenceVesc:
    commands = "vesc"
    driven = (True,) * 4  # the Slash 4x4: the centre gear drives both axles

    def __init__(self, vehicle: dict, params: dict, seed: int):
        unknown = set(params) - set(DEFAULTS)
        if unknown:
            raise ValueError(f"unknown params {sorted(unknown)}; known: {sorted(DEFAULTS)}")
        self.p = {**DEFAULTS, **{k: float(v) for k, v in params.items()}}
        g = vehicle["geometry"]
        self.r = float(g["wheel_radius"])
        self.max_steer = float(g["max_steer_angle"])
        # the gearing: wheel rad/s per motor rad/s, from the ERPM per m/s
        motor_rad_s_per_mps = self.p["erpm_per_mps"] / self.p["erpm_per_motor_rad_s"]
        self.gear = motor_rad_s_per_mps * self.r  # motor rad/s per wheel rad/s
        self.seed = int(seed)
        self.debug = {}
        self.reset()

    def reset(self):
        self.rng = random.Random(self.seed)
        self.t = 0.0
        self.queue = deque()  # (effective time, command) in arrival order
        self.last_arrival = None
        self.active = VescCommand()
        self.active_arrived = None
        self.integral = 0.0
        self.tire_angle = 0.0
        self.tachometer = 0.0
        self.current = 0.0

    # --- commands ------------------------------------------------------------------------

    def _arrive(self, command):
        """A command the vehicle interface hands over, queued until its latency has passed. It
        is a new one when its motor command or servo position changed, or its age fell (a new
        message with the same values)."""
        key = (command.motor, command.value, command.servo)
        last = self.last_arrival
        self.last_arrival = (key, command.age)
        if last is not None and key == last[0] and command.age >= last[1]:
            return
        arrived = self.t - (0.0 if math.isinf(command.age) else command.age)
        delay = self.p["latency"]
        if self.p["latency_jitter"] > 0:
            delay = max(0.0, delay + self.rng.gauss(0.0, self.p["latency_jitter"]))
        self.queue.append((arrived + delay, arrived, command))

    def _due(self):
        while self.queue and self.queue[0][0] <= self.t + 1e-12:
            _, self.active_arrived, self.active = self.queue.popleft()

    # --- the step ------------------------------------------------------------------------

    def step(self, command, sensors, dt):
        p = self.p
        self.t += dt
        if command is not None:
            self._arrive(command)
        self._due()
        speeds = sensors.wheel_speeds if sensors is not None else (0.0,) * 4
        wheel = sum(speeds) / 4.0  # rad/s
        motor = wheel * self.gear  # rad/s
        erpm = motor * p["erpm_per_motor_rad_s"]
        cmd = self.active
        age = self.t - self.active_arrived if cmd.motor else math.inf
        current, brake_current = 0.0, 0.0
        if cmd.motor is None or age > p["timeout"]:
            self.integral = 0.0
        elif cmd.motor == "speed":
            err = cmd.value - erpm
            self.integral += err * dt
            # anti-windup: the integral holds what keeps the output within the limit
            imax = p["current_max"] / max(p["speed_ki"], 1e-12)
            self.integral = max(-imax, min(imax, self.integral))
            current = p["speed_kp"] * err + p["speed_ki"] * self.integral
        elif cmd.motor == "current":
            current = cmd.value
        elif cmd.motor == "brake":
            brake_current = min(abs(cmd.value), p["brake_max"])
        elif cmd.motor == "duty":
            emf = motor * p["kt"]  # V: the back-EMF, kt w
            current = (cmd.value * p["battery"] - emf) / p["resistance"]
        current = max(-p["current_max"], min(p["current_max"], current))
        self.current = current
        torque = current * p["kt"] * self.gear  # N m at the wheels, all four
        drive = (torque / 4.0,) * 4
        brake = (brake_current * p["kt"] * self.gear / 4.0,) * 4
        # the servo
        if cmd.servo is not None:
            target = (cmd.servo - p["servo_offset"]) / p["servo_gain"]
            target = max(-self.max_steer, min(self.max_steer, target))
            step = p["servo_rate"] * dt
            self.tire_angle += max(-step, min(step, target - self.tire_angle))
        self.tachometer += abs(erpm) * dt / 60.0
        duty = (motor * p["kt"] + current * p["resistance"]) / p["battery"]
        telemetry = {"erpm": erpm, "current_motor": current, "duty": max(-1.0, min(1.0, duty)),
                     "voltage_input": p["battery"], "tachometer": int(self.tachometer)}
        self.debug = {"erpm": erpm, "current": current, "brake_current": brake_current,
                      "mode": cmd.motor}
        return Actuation(drive, brake, self.tire_angle, telemetry)
