"""The measurement protocol (protocol.yaml) as runs and commands: scensim roadmap 091 W23.
Pure: the real car's driver node and the synthetic stand-in both read it.

    runs(doc) -> [Run]        every manoeuvre, condition and repetition
    command(run, t, max_steer) -> (motor, value, servo) at time t: the VESC's command
"""
import itertools
from dataclasses import dataclass, field
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
MOTORS = ("speed", "current", "brake")


def load(path=HERE / "protocol.yaml"):
    return yaml.safe_load(Path(path).read_text())


@dataclass(frozen=True)
class Run:
    manoeuvre: str
    condition: dict
    repetition: int
    segments: tuple = field(repr=False)
    conversion: dict = field(repr=False)

    @property
    def name(self):
        """A stable name: M5_angle0.2_side-1_speed2_r3."""
        cond = "_".join(f"{k}{_short(v)}" for k, v in sorted(self.condition.items()))
        return f"{self.manoeuvre}_{cond}_r{self.repetition}" if cond else \
            f"{self.manoeuvre}_r{self.repetition}"

    @property
    def length(self):
        return sum(float(s["t"]) for s in self.segments)


def _short(v):
    return f"{v:g}" if isinstance(v, (int, float)) else str(v)


def runs(doc, only=None, repetitions=None):
    """Every run of the protocol, in order: manoeuvre, condition, repetition."""
    out = []
    reps = int(repetitions if repetitions is not None else doc.get("repetitions", 1))
    for name, m in doc["manoeuvres"].items():
        if only and name not in only:
            continue
        conds = m.get("conditions") or {}
        keys = sorted(conds)
        for combo in itertools.product(*(conds[k] for k in keys)):
            cond = dict(zip(keys, combo))
            for r in range(reps):
                out.append(Run(name, cond, r, tuple(m["segments"]), dict(doc["conversion"])))
    return out


def _value(v, cond):
    return cond[v] if isinstance(v, str) and v in cond else v


def target(run, t, max_steer):
    """(motor, value in m/s or A, tire angle rad) at time t into the run."""
    t0 = 0.0
    seg = run.segments[-1]
    for s in run.segments:
        if t < t0 + float(s["t"]):
            seg = s
            break
        t0 += float(s["t"])
    else:
        t0 = run.length - float(seg["t"])
    cond = run.condition
    motor = seg["motor"]  # a mode, never a condition's name
    if motor not in MOTORS:
        raise ValueError(f"{run.name}: motor {motor!r} is not one of {MOTORS}")
    value = float(_value(seg["value"], cond))
    if "to" in seg:
        frac = min(1.0, max(0.0, (t - t0) / float(seg["t"])))
        value += (float(_value(seg["to"], cond)) - value) * frac
    angle = _value(seg.get("angle", 0.0), cond)
    side = float(cond.get("side", 1))
    angle = max_steer if angle == "full" else float(angle)
    return motor, value, angle * side


def command(run, t, max_steer):
    """The VESC's command at time t: (motor mode, value in its unit, servo position)."""
    motor, value, angle = target(run, t, max_steer)
    c = run.conversion
    if motor == "speed":
        value = c["speed_to_erpm_gain"] * value + c["speed_to_erpm_offset"]
    servo = c["steering_angle_to_servo_gain"] * angle + c["steering_angle_to_servo_offset"]
    return motor, value, servo
