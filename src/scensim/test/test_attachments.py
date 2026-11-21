"""scensim.attachments (roadmap 092 K4): processes tied to a run, which read the truth stream."""
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

from scensim import attachments as at

WATCH = """import json, os, signal, sys, time
out = os.path.join(os.environ["SCENSIM_ATTACHMENT_DIR"], "env.json")
json.dump({k: v for k, v in os.environ.items() if k.startswith(("SCENSIM_", "ROS_"))}, open(out, "w"))
mode = sys.argv[1] if len(sys.argv) > 1 else "wait"
if mode == "die":
    sys.exit(3)
signal.signal(signal.SIGINT, lambda *a: sys.exit(0 if mode == "wait" else 5))
while True:
    time.sleep(0.05)
"""


def att(tmp_path, mode="wait", **kw):
    (tmp_path / "watch.py").write_text(WATCH)
    return {"name": "netsim", "run": [sys.executable, str(tmp_path / "watch.py"), mode], **kw}


def test_problems():
    assert at.problems(None) == []
    assert at.problems({"x": 1}) == [("attachments", "is not a list")]
    p = dict(at.problems([{"name": "a", "run": ["x"], "lockstep": "yes", "colour": 1},
                          {"name": "a", "run": []},
                          {"name": "b/c", "run": ["x"], "stop_timeout": 0},
                          {"name": "d", "run": ["x"], "lifetime": "lab"}]))
    msgs = " | ".join(m for _, m in at.problems([{"name": "a", "run": ["x"], "lockstep": "yes",
                                                  "colour": 1}]))
    assert "unknown key 'colour'" in msgs and "lockstep: 'yes' is not true or false" in msgs
    assert "given twice" in p["attachments[1]"] or "run:" in p["attachments[1]"]
    allm = " | ".join(m for _, m in at.problems([{"name": "a", "run": []}, {"name": "a", "run": ["x"]},
                                                 {"name": "b/c", "run": ["x"], "stop_timeout": 0},
                                                 {"name": "d", "run": ["x"], "lifetime": "lab"}]))
    for want in ("run: a command", "given twice", "a plain name", "stop_timeout", "question 6"):
        assert want in allm, want
    # lockstep follows CARLA's tick gate; Isaac has none yet
    assert at.problems([{"name": "a", "run": ["x"], "lockstep": True}], "carla") == []
    (w, m), = at.problems([{"name": "a", "run": ["x"], "lockstep": True}], "isaac")
    assert "no tick gate yet" in m
    assert at.lockstep([{"lockstep": True}]) and not at.lockstep([{"name": "a"}, None])


def test_an_attachment_gets_its_env_is_stopped_and_recorded(tmp_path):
    run_dir = tmp_path / "run"
    a = att(tmp_path, lockstep=True)
    roles = at.roles([SimpleNamespace(name="ego", ego=True), SimpleNamespace(name="npc", ego=False),
                      SimpleNamespace(name="lead", ego=False)], [{"entity": "lead", "kind": "carla_basic"}])
    assert roles == {"ego": "ego", "npc": "scenario", "lead": "carla_basic"}
    env = at.env_for(a, {"ROS_DOMAIN_ID": "9", "PATH": "/usr/bin:/bin"}, run_dir, 42, roles,
                     tick_gate="127.0.0.1:5561")
    r = at.start(a, env, run_dir, tmp_path)
    deadline = time.time() + 10
    seen = run_dir / "attachments" / "netsim" / "env.json"
    while not seen.is_file() and time.time() < deadline:
        time.sleep(0.05)
    time.sleep(0.2)
    at.stop(r)
    e = json.loads(seen.read_text())
    assert e["SCENSIM_RUN_DIR"] == str(run_dir) and e["SCENSIM_RUN_SEED"] == "42"
    assert e["SCENSIM_ATTACHMENT_DIR"].endswith("attachments/netsim")
    assert json.loads(e["SCENSIM_ROLES"]) == roles
    assert e["SCENSIM_TRUTH_TOPIC"] == "/scensim/truth" and e["ROS_DOMAIN_ID"] == "9"
    assert e["SCENSIM_TICK_GATE"] == "127.0.0.1:5561"
    rec = r.record()
    assert rec["exit"] == 0 and not rec["ended_early"] and rec["stopped"] >= rec["started"]
    assert r.failed() == ""
    # without lockstep, no gate
    assert "SCENSIM_TICK_GATE" not in at.env_for(att(tmp_path), {}, run_dir, 1, {}, "127.0.0.1:1")


def test_an_early_exit_and_a_bad_exit_are_recorded_and_fail_only_if_required(tmp_path):
    run_dir = tmp_path / "run"
    r = at.start(att(tmp_path, "die"), at.env_for(att(tmp_path), {}, run_dir, None, {}), run_dir, tmp_path)
    time.sleep(1.0)
    at.stop(r)
    assert r.early and r.code == 3 and "ended before the run did" in r.failed()
    assert r.record()["ended_early"] and not r.record()["required"]
    r = at.start(att(tmp_path, "bad"), at.env_for(att(tmp_path), {}, run_dir, None, {}), run_dir, tmp_path)
    time.sleep(1.0)
    at.stop(r)
    assert not r.early and r.code == 5 and "exited 5 when stopped" in r.failed()
    # a command that does not exist
    r = at.start({"name": "x", "run": [str(tmp_path / "nope")]}, {}, run_dir, tmp_path)
    assert "did not start" in r.failed() and at.stop(r).record()["error"]


def test_one_that_ignores_sigint_is_terminated(tmp_path):
    (tmp_path / "stubborn.py").write_text(
        "import signal, time\nsignal.signal(signal.SIGINT, signal.SIG_IGN)\n"
        "open(__import__('os').environ['SCENSIM_ATTACHMENT_DIR'] + '/up', 'w')\n"
        "while True: time.sleep(0.05)\n")
    run_dir = tmp_path / "run"
    a = {"name": "s", "run": [sys.executable, str(tmp_path / "stubborn.py")], "stop_timeout": 0.5}
    r = at.start(a, at.env_for(a, {}, run_dir, None, {}), run_dir, tmp_path)
    deadline = time.time() + 10
    while not (run_dir / "attachments" / "s" / "up").exists() and time.time() < deadline:
        time.sleep(0.05)
    t = time.time()
    at.stop(r)
    assert r.code == -15 and time.time() - t < 5
