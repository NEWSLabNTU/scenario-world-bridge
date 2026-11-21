"""The stand-in drive unit (scensim roadmap 091 W23): its commands, latency, limits and
telemetry, without a simulator."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from roboracer_vesc.component import DEFAULTS, RoboRacerVesc  # noqa: E402
from sim_vehicle_ecu.component import check  # noqa: E402
from sim_vehicle_ecu.signals import Sensors, VescCommand  # noqa: E402

CAR = {"geometry": {"wheel_radius": 0.055, "max_steer_angle": 0.36}}
DT = 1 / 240


def make(**params):
    return RoboRacerVesc(CAR, params, 0)


def at(speed_mps):
    w = speed_mps / CAR["geometry"]["wheel_radius"]
    return Sensors(wheel_speeds=(w,) * 4)


def run(c, command, speed, seconds):
    out = None
    for i in range(int(round(seconds / DT))):
        out = c.step(command._replace(age=i * DT) if command.motor else command, at(speed), DT)
    return out


def test_it_is_a_component():
    check(make())
    with pytest.raises(ValueError, match="unknown params"):
        make(gain=1.0)


def test_speed_mode_pushes_toward_the_commanded_erpm_within_the_current_limit():
    c = make()
    a = run(c, VescCommand("speed", 2.0 * DEFAULTS["erpm_per_mps"]), 0.0, 0.2)
    assert a.telemetry["current_motor"] == pytest.approx(DEFAULTS["current_max"])
    assert all(d > 0 for d in a.drive)
    c = make()
    a = run(c, VescCommand("speed", 2.0 * DEFAULTS["erpm_per_mps"]), 2.0, 0.2)
    assert abs(a.telemetry["current_motor"]) < 1.0  # at the speed: nothing to push
    assert a.telemetry["erpm"] == pytest.approx(2.0 * DEFAULTS["erpm_per_mps"])


def test_the_latency_delays_every_command():
    c = make(latency=0.05)
    cmd = VescCommand("current", 10.0)
    outs = [c.step(cmd._replace(age=i * DT), at(0.0), DT) for i in range(30)]
    first = next(i for i, a in enumerate(outs) if a.telemetry["current_motor"] > 0)
    assert (first + 1) * DT == pytest.approx(0.05, abs=DT)


def test_brake_mode_brakes_every_wheel_and_a_stale_command_releases_the_motor():
    c = make()
    a = run(c, VescCommand("brake", 20.0), 2.0, 0.1)
    assert all(b > 0 for b in a.brake) and a.drive == (0.0,) * 4
    c = make(timeout=0.1)
    cmd = VescCommand("current", 10.0)
    for i in range(int(0.3 / DT)):  # the same message, never renewed: it ages
        a = c.step(cmd._replace(age=i * DT), at(0.0), DT)
    assert a.telemetry["current_motor"] == 0.0


def test_the_servo_maps_and_rate_limits_the_tire_angle():
    c = make(latency=0.0)
    servo = DEFAULTS["servo_offset"] + DEFAULTS["servo_gain"] * 0.2  # 0.2 rad, left
    a = c.step(VescCommand(servo=servo), at(0.0), DT)
    assert a.tire_angle == pytest.approx(DEFAULTS["servo_rate"] * DT)
    for _ in range(240):
        a = c.step(VescCommand(servo=servo), at(0.0), DT)
    assert a.tire_angle == pytest.approx(0.2)
    a = run(c, VescCommand(servo=0.0), 0.0, 1.0)  # past the stop
    assert a.tire_angle == pytest.approx(0.36)


def test_jitter_is_the_seeds():
    def trace(seed):
        c = RoboRacerVesc(CAR, {"latency_jitter": 0.01}, seed)
        out = []
        for k in range(20):
            cmd = VescCommand("current", float(k))
            for i in range(12):
                out.append(c.step(cmd._replace(age=i * DT), at(0.0), DT).telemetry["current_motor"])
        return out
    assert trace(1) == trace(1)
    assert trace(1) != trace(2)
    c = make()
    c.reset()
    assert c.step(VescCommand(), None, DT).drive == (0.0,) * 4
    assert not math.isnan(c.step(VescCommand("duty", 0.1), at(1.0), DT).telemetry["duty"])
