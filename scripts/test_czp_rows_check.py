"""czp_rows_check's car model (roadmap 078): the probe's watch is a stop, as the node makes it.

The two contexts below failed the impatient-adversary check (patience 8 s) once the probe's stop margin became
0.5 m + 1.2 s x v: the creep turned into the watch 0.09 m after it began, and the checker's watch rolled on at
probe_watch_speed to 0.3 m from the zone, so the go at the watch's end met the adversary's restart. The node
DEACTIVATEs for the watch, and Autoware stops 0.119 + 0.954 v after the request (the 078 creep-stop set).
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "coop"))
import czp_rows_check as R  # noqa: E402

X = R.X
FAILING = [(6.0, 0.7), (14.0, -0.1)]  # (v, dt): the contexts B:impatient p8 failed in, A reading B "+e", 0.79 s late


@pytest.fixture
def p2g(monkeypatch):
    monkeypatch.setattr(X, "REST_LAG", 0.8)
    monkeypatch.setattr(X, "AUTOWARE_LAUNCH", (0.6, 0.68, 3.0))


def run(v, dt, model, monkeypatch):
    monkeypatch.setattr(R, "WATCH_MODEL", model)
    ctx = (v, 0.0, {"A": ("A", X.T_MEAN + dt / 2), "B": ("B", X.T_MEAN - dt / 2)}, {})
    tr = []
    r = R.run_passage("cross", ctx, {("A", "B"): ("+e", R.DELAYS[1])}, 0.55, X.car_lags(), trace=tr,
                      adversary=("B", "impatient", 8.0))
    return r, tr


@pytest.mark.parametrize("v0", [0.1, 0.3, 0.5, 0.7, 1.0])
def test_watch_stop_travel_is_the_measured_one(v0, p2g):
    class C:
        pass
    c = C()
    c.u, c.v, c.watch_to = -5.0, v0, None
    c.act = X.Actuator(R.czp_lags(X.car_lags()))
    for _ in range(300):
        a = c.act.step(R.watch_accel(c), c.v)
        c.v = max(0.0, c.v + a * X.DT)
        c.u += c.v * X.DT
    travel = c.u + 5.0
    fit = R.WATCH_STOP[0] + R.WATCH_STOP[1] * v0
    assert c.v == 0.0
    assert abs(travel - fit) <= 0.02


@pytest.mark.parametrize("v,dt", FAILING)
def test_failing_context_safe_with_the_watch_a_stop(v, dt, p2g, monkeypatch):
    r, tr = run(v, dt, "stop", monkeypatch)
    assert r["unsafe"] is None and r["live"]
    # the watch brought A to rest before the zone, beyond the stop margin's reach
    watch = [row for row in tr if row[1] == "A" and row[5] == "probe:watch"]
    assert watch and max(row[2] for row in watch) < -0.3


@pytest.mark.parametrize("v,dt", FAILING)
def test_failing_context_unsafe_with_the_rolling_watch(v, dt, p2g, monkeypatch):
    """The reproduction: 078 V2b's rolling watch, the model the failure came from."""
    r, _ = run(v, dt, "roll", monkeypatch)
    assert r["unsafe"] is not None
