"""Roadmap 091 W10, the pure parts: the stage intervals (a jackknife over the repetitions), the
anchor and the drift rule. No simulator."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "protocol"), str(ROOT / "tools")]
pytest.importorskip("scipy")
import fit  # noqa: E402
import refine_twin  # noqa: E402

FRICTION = "vehicle.axle_tires.rear.tire_friction"


def test_a_fit_flattens_to_its_vehicle_and_component_parameters():
    f = {"vehicle": {"mass": 3.5, "axle_tires": {"rear": {"tire_friction": 1.0}}},
         "component": {"latency": 0.03}, "noise": {"erpm": 40.0}, "not_identified": []}
    assert fit.flatten(f) == {"vehicle.mass": 3.5, FRICTION: 1.0, "component.latency": 0.03}


def test_the_interval_is_twice_the_jackknife_error():
    # leave-one-out values 1, 2, 3, 4, 5: the jackknife's error is sqrt(4/5 * 10) = 2.828...
    leave = [{"vehicle.mass": v} for v in (1.0, 2.0, 3.0, 4.0, 5.0)]
    lo, hi = fit.intervals({"vehicle.mass": 3.0}, leave)["vehicle.mass"]
    assert hi - 3.0 == pytest.approx(2 * (4 / 5 * 10) ** 0.5, rel=1e-5)
    assert 3.0 - lo == pytest.approx(hi - 3.0, rel=1e-5)


def test_an_interval_is_at_least_the_stages_resolution_and_a_lower_bound_is_one_sided():
    leave = [{"component.latency": 0.036, FRICTION: 1.1} for _ in range(5)]
    iv = fit.intervals({"component.latency": 0.036, FRICTION: 1.11}, leave, {FRICTION: 0.84})
    assert iv["component.latency"] == pytest.approx([0.035, 0.037])
    assert iv[FRICTION] == [0.84, None]


def test_the_anchor_costs_half_widths_and_nothing_above_a_lower_bound():
    ivs = {"vehicle.mass": [3.0, 4.0], FRICTION: [0.8, None]}
    stage = {"vehicle.mass": 3.5, FRICTION: 1.1}
    a = refine_twin.anchor_terms({"vehicle.mass": 4.5, FRICTION: 1.3}, stage, ivs)
    assert a == pytest.approx([2.0, 0.0])
    a = refine_twin.anchor_terms({"vehicle.mass": 3.5, FRICTION: 0.76}, stage, ivs)
    assert a == pytest.approx([0.0, 1.0])  # 0.04 below the bound: 5 % of 0.8


def test_the_drift_rule_flags_what_left_its_interval():
    """W23's p2: the refinement took the rear friction below what the logs already show."""
    ivs = {FRICTION: [0.843, None], "component.latency": [0.035, 0.037]}
    stage = {FRICTION: 1.11, "component.latency": 0.036, "vehicle.mass": 3.47}
    d = refine_twin.drift(stage, {FRICTION: 0.72, "component.latency": 0.0362,
                                  "vehicle.mass": 3.6}, ivs)
    assert d[FRICTION]["outside"] is True and d[FRICTION]["change"] == pytest.approx(-0.3514)
    assert d["component.latency"]["outside"] is False
    assert d["vehicle.mass"]["outside"] is None  # no interval: reported, not judged
