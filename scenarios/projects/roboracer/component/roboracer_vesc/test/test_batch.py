"""The batched VESC (scensim 092 S3 for 091 W23): N cars equal N single cars of 118's scalar
RoboRacerVesc as W23 landed it (reference_vesc.py), bit for bit, over random command
sequences in every mode, with the latency queue, its jitter per seed, stale commands, missing
readings and resets; and the single car is the batch at one slot."""
import math
import random
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from reference_vesc import ReferenceVesc  # noqa: E402
from roboracer_vesc.component import RoboRacerVesc, RoboRacerVescBatch  # noqa: E402
from sim_vehicle_ecu import batch as B  # noqa: E402
from sim_vehicle_ecu.signals import Sensors, VescCommand  # noqa: E402

CAR = {"geometry": {"wheel_radius": 0.055, "max_steer_angle": 0.36}}
DT = 1 / 240


# Bit for bit before Python 3.12. From 3.12, float sum() is compensated: the reference's
# `sum(speeds) / 4.0` then differs in the last bit from the left-to-right order, which the
# batched form writes out (the same on every Python). So the reference differs from itself
# across Pythons, and the comparison there allows 1e-12 relative.
EXACT = sys.version_info < (3, 12)


def same(a, b):
    if EXACT:
        pack = lambda v: np.array(v, dtype=np.float64).tobytes()   # noqa: E731
        return (pack(a.drive) == pack(b.drive) and pack(a.brake) == pack(b.brake)
                and pack([a.tire_angle]) == pack([b.tire_angle]) and a.telemetry == b.telemetry)
    close = lambda x, y: np.allclose(np.array(x, float), np.array(y, float), rtol=1e-12,  # noqa
                                     atol=1e-12)
    return (close(a.drive, b.drive) and close(a.brake, b.brake)
            and close([a.tire_angle], [b.tire_angle]) and a.telemetry.keys() == b.telemetry.keys()
            and all(close([a.telemetry[k]], [b.telemetry[k]]) for k in a.telemetry))


def sequence(n, steps, seed):
    """Per step: a command per car (held for a while, then a new one, sometimes the same values
    again with a fresh age, sometimes None), its sensors, and which cars reset."""
    rng = random.Random(seed)
    held = [VescCommand() for _ in range(n)]
    ages = [0.0] * n
    out = []
    for k in range(steps):
        cmds, sens = [], []
        for i in range(n):
            r = rng.random()
            if r < 0.04:
                mode = rng.choice([None, "speed", "current", "brake", "duty", "position"])
                value = {"speed": rng.uniform(-8000, 30000), "current": rng.uniform(-60, 60),
                         "brake": rng.uniform(-50, 50), "duty": rng.uniform(-1.2, 1.2)}.get(
                             mode, rng.uniform(-1, 1))
                held[i] = VescCommand(mode, value, None if rng.random() < 0.2 else rng.random(),
                                      0.0)
                ages[i] = 0.0
            elif r < 0.06:
                ages[i] = 0.0                       # the same values, a new message
            else:
                ages[i] += DT
            if rng.random() < 0.01:
                ages[i] = math.inf
            cmds.append(None if rng.random() < 0.03 else held[i]._replace(age=ages[i]))
            sens.append(None if k == 0 or rng.random() < 0.02 else
                        Sensors(tuple(rng.uniform(0, 400) for _ in range(4))))
        out.append((cmds, sens, np.array([rng.random() < 0.005 for _ in range(n)])))
    return out


@pytest.mark.parametrize("n,jitter", [(1, 0.0), (8, 0.0), (32, 0.004), (16, 0.02)])
def test_n_cars_equal_n_reference_cars_bit_for_bit(n, jitter):
    params = {"latency_jitter": jitter} if jitter else {}
    seeds = [11 + i for i in range(n)]
    b = B.load_batch(RoboRacerVesc, CAR, params, seeds)
    assert b.vectorised
    refs = [ReferenceVesc(CAR, params, s) for s in seeds]
    for cmds, sens, resets in sequence(n, 1500, seed=n):
        if resets.any():
            b.reset_batch(resets)
            for r, m in zip(refs, resets):
                if m:
                    r.reset()
        act = B.checked_batch(b.step_batch(B.commands_to_batch("vesc", cmds),
                                           B.sensors_to_batch(sens), DT), n)
        for i, (g, r) in enumerate(zip(B.batch_to_actuations(act), refs)):
            w = r.step(cmds[i], sens[i], DT)
            assert same(g, w), (i, g, w)


def test_the_single_car_keeps_its_interface():
    c = RoboRacerVesc(CAR, {"latency": 0.01}, 3)
    assert c.p["latency"] == 0.01 and c.gear == ReferenceVesc(CAR, {}, 0).gear
    a = c.step(VescCommand("current", 10.0, 0.6, 0.0), Sensors((100.0,) * 4), DT)
    assert c.debug == {"erpm": c.debug["erpm"], "current": 0.0, "brake_current": 0.0,
                       "mode": None}     # still within its latency
    assert isinstance(a.telemetry["tachometer"], int)
    with pytest.raises(ValueError, match="unknown params"):
        RoboRacerVesc(CAR, {"nope": 1}, 0)


def test_the_batch_is_its_own_class_and_refuses_nothing_a_single_car_takes():
    b = RoboRacerVescBatch(CAR, {}, [1, 2])
    assert B.check_batch(b) is b


def test_per_car_params_equal_reference_cars_each_with_its_own():
    """Per-car params (sim_vehicle_ecu's list of N dicts): a fit's candidate sets, or a varied
    car. Car i is the reference car built with dict i."""
    rng = random.Random(4)
    n = 12
    per = [{"speed_kp": rng.uniform(0.002, 0.008), "speed_ki": rng.uniform(0.005, 0.02),
            "kt": rng.uniform(0.002, 0.004), "latency": rng.choice([0.0, 0.01, 0.03]),
            "latency_jitter": rng.choice([0.0, 0.005]), "servo_rate": rng.uniform(2, 8),
            "erpm_per_mps": rng.uniform(4000, 5000)} for _ in range(n)]
    seeds = [3 * i for i in range(n)]
    b = B.load_batch(RoboRacerVesc, CAR, per, seeds)
    refs = [ReferenceVesc(CAR, p, s) for p, s in zip(per, seeds)]
    for cmds, sens, resets in sequence(n, 1200, seed=77):
        act = b.step_batch(B.commands_to_batch("vesc", cmds), B.sensors_to_batch(sens), DT)
        for i, g in enumerate(B.batch_to_actuations(B.checked_batch(act, n))):
            w = refs[i].step(cmds[i], sens[i], DT)
            assert same(g, w), (i, g, w)
    one = RoboRacerVesc(CAR, per[0], 0)
    assert one.p["speed_kp"] == per[0]["speed_kp"] and isinstance(one.gear, float)
