"""Roadmap 091 W23 and W10, the pure parts: the protocol, the quantities and 5.3's bands, the
replay fit, the mass from steady holds, and the blind recovery check. No simulator."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "protocol"), str(ROOT / "tools"), str(ROOT / "component" / "roboracer_vesc")]
pytest.importorskip("sim_vehicle_ecu")
import check_recovery  # noqa: E402
import fit  # noqa: E402
import protocol as protomod  # noqa: E402
import quantities as qmod  # noqa: E402
from roboracer_vesc.component import RoboRacerVesc  # noqa: E402

DOC = protomod.load()
CAR = yaml.safe_load((ROOT / "vehicles" / "roboracer.yaml").read_text())


def test_the_protocol_covers_m1_to_m6_with_five_repetitions():
    runs = protomod.runs(DOC)
    assert {r.manoeuvre for r in runs} >= {"M1", "M2", "M3", "M4", "M5", "M6"}
    assert all(sum(1 for q in runs if q.name[:-1] == r.name[:-1]) == 5 for r in runs)
    assert len({r.name for r in runs}) == len(runs)


def test_a_command_is_the_vesc_units():
    m6 = next(r for r in protomod.runs(DOC, only=["M6"]) if r.condition["side"] == -1)
    motor, erpm, servo = protomod.command(m6, 11.0, 0.36)  # half-way up the ramp
    assert motor == "speed" and erpm == pytest.approx(4.0 * 4614.0)
    assert servo == pytest.approx(-1.2135 * -0.2 + 0.5304)
    b = protomod.runs(DOC, only=["M3b"])[0]
    assert protomod.command(b, 5.0, 0.36)[:2] == ("brake", 20.0)


def synthetic_m1(speed=2.0, tau=0.3, dead=0.04, noise=0.0, seed=0):
    """An M1 run's streams with a first-order response: (session, streams)."""
    rng = np.random.default_rng(seed)
    t = np.arange(0.02, 7.0, 0.02)
    v = np.where(t > 1.0 + dead, speed * (1 - np.exp(-(t - 1.0 - dead) / tau)), 0.0)
    v = np.where(t > 5.0, speed * np.exp(-(t - 5.0) / tau), v)
    erpm = 4614.0 * v + rng.normal(0.0, noise, len(t))
    rt = np.arange(0.01, 7.0, 0.01)
    streams = {"core_t": t, "core_erpm": erpm, "core_current_motor": np.zeros_like(t),
               "ref_t": rt, "ref_x": np.cumsum(np.interp(rt, t, v)) * 0.01, "ref_y": 0 * rt}
    sess = {"manoeuvre": "M1", "condition": {"speed": speed}, "repetition": 0,
            "reference": {"noise_m": 0.002}}
    return sess, streams


def test_m1s_quantities_are_the_response():
    s, st = synthetic_m1(tau=0.3, dead=0.04)
    q = qmod.quantities(s, st, DOC)
    assert q["steady_speed"] == pytest.approx(2.0, abs=1e-3)
    assert q["rise_time"] == pytest.approx(0.3 * math.log(9), abs=0.02)  # 10 to 90 %
    assert q["dead_time"] == pytest.approx(0.04 + 0.3 * -math.log(0.95), abs=0.02)


def test_the_band_is_the_spread_widened_by_the_resolution():
    assert qmod.band([1.0, 1.2], 0.05) == pytest.approx((0.95, 1.25))
    assert qmod.band([1.0, 1.01], 0.05) == pytest.approx((1.005 - 0.1, 1.005 + 0.1))  # below res
    assert qmod.band([None, float("nan")], 0.05) is None


def test_verdicts_pass_the_same_car_and_fail_a_slower_one():
    real = [synthetic_m1(noise=20.0, seed=k) for k in range(5)]
    same = qmod.verdicts(real, [synthetic_m1()], DOC)
    assert same and all(v.passed for v in same)
    slow = qmod.verdicts(real, [synthetic_m1(tau=0.5)], DOC)
    assert {v.quantity for v in slow if not v.passed} >= {"rise_time", "speed_trace_rms"}


def test_the_replay_recovers_the_components_latency_and_gains():
    """Streams the component itself made, at known params: the replay fit finds them."""
    from sim_vehicle_ecu.signals import Sensors, VescCommand
    # the current limit at its default: without M1c the fit keeps the default, and says so
    truth = {"speed_kp": 0.006, "speed_ki": 0.012, "current_max": 40.0, "latency": 0.04}
    run = protomod.runs(DOC, only=["M1"], repetitions=1)[1]  # 2 m/s
    c = RoboRacerVesc(CAR, truth, 0)
    dt = fit.REPLAY_DT
    erpm_per_wheel = c.gear * c.p["erpm_per_motor_rad_s"]
    v, t, cmd_t = 0.0, 0.0, 0.0
    rows, cmds, last = [], [], None
    for i in range(int(run.length / dt)):
        if i % 12 == 0:  # 50 Hz commands at 240 Hz steps
            last, cmd_t = protomod.command(run, t, 0.36), t
            cmds.append((t, protomod.MOTORS.index(last[0]), last[1], last[2]))
        a = c.step(VescCommand(last[0], last[1], last[2], t - cmd_t),
                   Sensors(wheel_speeds=(v / 0.055,) * 4), dt)
        v += (sum(a.drive) / 0.055 / 4.0 - 0.5 * v) * dt  # a simple car of 4 kg
        t = (i + 1) * dt
        if i % 12 == 11:
            rows.append((t, v / 0.055 * erpm_per_wheel, a.telemetry["current_motor"]))
    cm, rw = np.array(cmds), np.array(rows)
    st = {"cmd_t": cm[:, 0], "cmd_mode": cm[:, 1], "cmd_value": cm[:, 2], "cmd_servo": cm[:, 3],
          "core_t": rw[:, 0], "core_erpm": rw[:, 1], "core_current_motor": rw[:, 2]}
    sess = {"manoeuvre": "M1", "condition": run.condition, "repetition": 0}
    got, rms, notes = fit.fit_component([(sess, st)], RoboRacerVesc, CAR, protomod.MOTORS, DOC,
                                    latencies=np.arange(0.024, 0.0561, 0.008))
    assert got["latency"] == pytest.approx(0.04, abs=0.002)
    assert got["speed_kp"] == pytest.approx(0.006, rel=0.05)
    assert rms < 0.1
    assert any("current_max: not excited" in n for n in notes)


def test_the_mass_from_steady_holds():
    k, m, c = 1.3, 4.0, (0.2, 0.05, 0.01)
    holds = [(v, (m + 0.2) * (c[0] + c[1] * v + c[2] * v * v) / k) for v in (1.0, 2.0, 4.0)]
    est = [k * i / (c[0] + c[1] * v + c[2] * v * v) - 0.2 for v, i in holds]
    assert np.median(est) == pytest.approx(m)


def test_recovery_reports_misses_and_what_the_logs_could_not_identify(tmp_path):
    truth = {"perturbation": {"vehicle": {"mass": 4.0, "axle_tires": {"rear": {"tire_friction": 1.2}}},
                              "component": {"latency": 0.04, "latency_jitter": 0.01}},
             "seed": 1}
    fitted = {"vehicle": {"mass": 4.5, "axle_tires": {"rear": {"tire_friction": 0.9}}},
              "component": {"latency": 0.041},
              "not_identified": ["vehicle.axle_tires.rear.tire_friction"]}
    t, f = tmp_path / "truth.yaml", tmp_path / "fitted.yaml"
    t.write_text(yaml.safe_dump(truth))
    f.write_text(yaml.safe_dump(fitted))
    assert check_recovery.main([str(t), str(f)]) == 1  # the mass is 12 % off
    fitted["vehicle"]["mass"] = 4.05
    f.write_text(yaml.safe_dump(fitted))
    assert check_recovery.main([str(t), str(f)]) == 0  # jitter and the rear: reported only
