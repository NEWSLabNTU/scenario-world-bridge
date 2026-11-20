"""crossing_aw's score (074 U3): a run where A crossed and the ego (B) never entered Z is "starved", a counted
verdict like a deadlock, unless its bag shows issue 082 -- then it is the error "ego never entered", retried.

On 139's stage 1 one such run was scored ok: Autoware's velocity limit went to 0 after the ego had stopped at its
line (issue 082), A crossed alone, and neither the contact nor the deadlock check saw it. A strategy that holds the
ego is its own liveness failure (243): counted, so it cannot drop out of the grid through retries.
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


def test_ego_never_entered_is_starved(C, tmp_path):
    write_log(tmp_path, a_moves=True, b_moves=False)
    r = C.score(tmp_path, 0.0)
    assert r["starved"] == 1 and "error" not in r
    assert r["tB_in"] is None and r["tA_in"] is not None
    assert r["deadlock"] == 0 and r["contact"] == 0  # what the old scoring saw: an ok run


def test_both_entered_is_neither(C, tmp_path):
    write_log(tmp_path, a_moves=True, b_moves=True)
    r = C.score(tmp_path, 0.0)
    assert "error" not in r and r["starved"] == 0
    assert r["tA_in"] is not None and r["tB_in"] is not None and r["tA_in"] < r["tB_in"]


def test_neither_entered_stays_a_deadlock(C, tmp_path):
    write_log(tmp_path, a_moves=False, b_moves=False)
    r = C.score(tmp_path, 0.0)
    assert "error" not in r and r["starved"] == 0
    assert r["deadlock"] == 1


@pytest.mark.parametrize("sig, error", [((1, 68.5), "ego never entered"), ((0, None), None), ((None, None), None)])
def test_finish_082_makes_starved_an_error(C, tmp_path, monkeypatch, sig, error):
    """082's signature in the bag: a stack fault, an error; none, or a bag that cannot be read: the verdict stays."""
    write_log(tmp_path, a_moves=True, b_moves=False)
    (tmp_path / "bag").mkdir()
    monkeypatch.setattr(C, "bag_limit0", lambda bag: sig)
    r = C.finish(tmp_path, {"dt": 0.0})
    assert r.get("error") == error and r["starved"] == 1 and (r["sig082"], r["sig082_t"]) == sig


def test_finish_reads_no_bag_for_a_good_run(C, tmp_path, monkeypatch):
    write_log(tmp_path, a_moves=True, b_moves=True)
    (tmp_path / "bag").mkdir()
    monkeypatch.setattr(C, "bag_limit0", lambda bag: pytest.fail("read a bag for a good run"))
    assert "sig082" not in C.finish(tmp_path, {"dt": 0.0})


def test_limit0(C):
    assert C.limit0([(0.0, 8.0, ""), (52.0, 0.0, "")]) == 52.0
    assert C.limit0([(0.0, 8.0, ""), (40.0, 0.0, "interaction_strategy"), (41.0, 8.0, "")]) is None  # czp's own stop
    assert C.limit0([(0.0, 0.0, "")]) is None  # nothing real before it
    assert C.limit0([]) is None


def test_summary_keeps_failed_attempts_apart(C, tmp_path):
    rows = [
        # 082 twice, then a good run: the final is ok, the two attempts are listed with their signature
        {"tag": "a", "arm": "on", "attempt": 0, "starved": 1, "error": "ego never entered", "sig082": 1, "sig082_t": 52.0},
        {"tag": "a", "arm": "on", "attempt": 1, "error": "no lock logged"},
        {"tag": "a", "arm": "on", "attempt": 2, "contact": 0, "deadlock": 0, "starved": 0},
        # starved without 082 under load: the quiet rerun counts, starved again
        {"tag": "b", "arm": "on", "attempt": 0, "starved": 1, "sig082": 0, "counted": 0},
        {"tag": "b", "arm": "on", "attempt": 0, "starved": 1, "sig082": 0, "quiet_rerun": 1},
        # errors only: no verdict
        {"tag": "c", "arm": "off", "attempt": 0, "error": "no setup logged"},
        {"tag": "c", "arm": "off", "attempt": 1, "error": "no setup logged"},
    ]
    (tmp_path / "runs.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    s = C.summary(tmp_path / "runs.jsonl")
    assert (s["tags"], s["final_ok"], s["starved"], s["final_errors"]) == (3, 2, 1, {"no setup logged": 1})
    assert s["failed_attempts"] == {"ego never entered": 1, "no lock logged": 1, "no setup logged": 1}
    assert [(x["tag"], x["attempt"], x["sig082"], x["final"]) for x in s["ego_never_entered"]] == \
        [("a", 0, 1, "ok"), ("b", 0, 0, "starved"), ("b", 0, 0, "this row")]
    assert s["no_verdict"] == ["c"]


def test_finish_counts_the_watchdog_clears_on_a_czp_arm(C, tmp_path, monkeypatch):
    """243: with czp's 082 fix in, 082's signature on a czp arm is a regression; the row says how many clears czp's
    watchdog sent (from the ego's play_log named in ego.log). Not looked up on the stock arm."""
    write_log(tmp_path, a_moves=True, b_moves=False)
    (tmp_path / "bag").mkdir()
    wd = tmp_path / "work" / "play_log" / "ego" / "2026-11-21_01-02-03" / "node" / "interaction_strategy_watchdog"
    wd.mkdir(parents=True)
    (wd / "err").write_text("[WARN] no heartbeat for 1.35 s (it held none in manual mode): modules [] back to auto "
                            "mode, velocity limit cleared\n[WARN] no heartbeat for 2.1 s (...): no velocity limit held, "
                            "none cleared\n")
    (tmp_path / "ego.log").write_text(f"log dir {wd.parents[1]}\n")
    monkeypatch.setattr(C, "bag_limit0", lambda bag: (1, 52.0))
    assert C.finish(tmp_path, {"dt": 0.0, "arm": "on"})["wd_clears"] == 1
    assert "wd_clears" not in C.finish(tmp_path, {"dt": 0.0, "arm": "off"})
    (tmp_path / "ego.log").write_text("no play_log here\n")
    assert C.finish(tmp_path, {"dt": 0.0, "arm": "on"})["wd_clears"] is None


def test_summary_lists_every_czp_082_attempt(C, tmp_path):
    """One 082 hit, then a clean rerun: no_verdict misses it, czp_082 does not."""
    rows = [{"tag": "a", "arm": "on", "attempt": 0, "error": "no lock logged", "sig082": 1, "sig082_t": 52.0, "wd_clears": 1},
            {"tag": "a", "arm": "on", "attempt": 1, "contact": 0, "starved": 0},
            {"tag": "b", "arm": "off", "attempt": 0, "error": "no lock logged", "sig082": 1}]
    (tmp_path / "runs.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    s = C.summary(tmp_path / "runs.jsonl")
    assert s["no_verdict"] == ["b"]
    assert [(x["tag"], x["attempt"], x["error"], x["wd_clears"], x["final"]) for x in s["czp_082"]] == \
        [("a", 0, "no lock logged", 1, "ok")]


def test_rescore_touches_only_each_tags_last_row(C, tmp_path, monkeypatch):
    """243: a tag's directory holds its last attempt only, so the earlier rows keep their live scoring."""
    rows = [{"tag": "a", "dt": 0.0, "attempt": 0, "error": "ego never entered", "starved": 1, "sig082": 1},
            {"tag": "a", "dt": 0.0, "attempt": 0, "counted": 0, "starved": 1, "sig082": 0},
            {"tag": "a", "dt": 0.0, "attempt": 0, "quiet_rerun": 1, "starved": 0},
            {"tag": "b", "dt": 0.0, "attempt": 0, "starved": 0}]
    (tmp_path / "runs.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    monkeypatch.setattr(C, "finish", lambda rd, res: {**res, "rescored": Path(rd).name})
    monkeypatch.setattr(sys, "argv", ["crossing_aw.py", "--out", str(tmp_path), "--rescore"])
    assert C.main() == 0
    out = [json.loads(line) for line in open(tmp_path / "runs.jsonl")]
    assert [r.get("rescored") for r in out] == [None, None, "a", "b"]
    assert out[:2] == rows[:2] and out[2]["quiet_rerun"] == 1
