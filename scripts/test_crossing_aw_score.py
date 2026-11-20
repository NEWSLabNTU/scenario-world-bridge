"""crossing_aw's score (074 U3): a run where A crossed and the ego (B) never entered Z is an error, not a verdict.

On 139's stage 1 one such run was scored ok: Autoware's velocity limit went to 0 after the ego had stopped at its
line (issue 082), A crossed alone, and neither the contact nor the deadlock check sees that.
"""

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def C():
    sys.path.insert(0, str(ROOT / "scripts" / "coop"))
    try:
        # crossing_aw imports the arena (carla, sim_vehicle_spec): skip where those are absent
        return pytest.importorskip("crossing_aw")
    finally:
        sys.path.pop(0)


def write_log(rd, a_moves, b_moves):
    """A from the west along x, B from the south along y, each 20 m before Z (u < 0 before it), 10 s at 20 Hz.
    A car that moves goes at 8 m/s from 1 s after the lock; one that does not rests 5 m before Z."""
    ext = {"A": [4.8, 2.2], "B": [4.8, 2.2]}
    ev = [{"event": "setup", "rule": "stop1y_c1.5", "dt": 0.0, "rest_offset": 0.0, "v": 8.0, "ext": ext, "exit": 8.3,
           "physics_off": False, "stops": {"A": -7.6, "B": -7.6}},
          {"event": "lock", "t": 1.0, "ua": -20.0, "va": 8.0, "ub": -20.0, "vb": 8.0, "ab": 0.0, "dt_lock": 0.0,
           "dt_const": 0.0}]
    rows = []
    for i in range(200):
        t = 1.0 + i * 0.05
        go = max(0.0, t - 2.0)
        ua, va = (-20.0 + 8.0 * go, 8.0 if go else 0.0) if a_moves else (-5.0, 0.0)
        ub, vb = (-20.0 + 8.0 * go, 8.0 if go else 0.0) if b_moves else (-5.0, 0.0)
        # B passes 3 s after A, so the two never meet
        if b_moves:
            ub, vb = (-20.0 + 8.0 * max(0.0, t - 5.0), 8.0 if t > 5.0 else 0.0)
        rows.append({"t": t, "phase": "run", "A": [ua, 0.0, 0.0, va, 0.0], "B": [0.0, ub - 50.0, 1.5708, 0.0, vb],
                     "ua": ua, "ub": ub, "va": va, "vb": vb})
    (rd / "coop_a.jsonl").write_text("".join(json.dumps(r) + "\n" for r in ev + rows))


def test_ego_never_entered_is_an_error(C, tmp_path):
    write_log(tmp_path, a_moves=True, b_moves=False)
    r = C.score(tmp_path, 0.0)
    assert r["error"] == "ego never entered"
    assert r["tB_in"] is None and r["tA_in"] is not None
    assert r["deadlock"] == 0 and r["contact"] == 0  # what the old scoring saw: an ok run


def test_both_entered_is_no_error(C, tmp_path):
    write_log(tmp_path, a_moves=True, b_moves=True)
    r = C.score(tmp_path, 0.0)
    assert "error" not in r
    assert r["tA_in"] is not None and r["tB_in"] is not None and r["tA_in"] < r["tB_in"]


def test_neither_entered_stays_a_deadlock(C, tmp_path):
    write_log(tmp_path, a_moves=False, b_moves=False)
    r = C.score(tmp_path, 0.0)
    assert "error" not in r
    assert r["deadlock"] == 1
