"""scripts/t3.py: a suite's results against its reference (roadmap 077, T3)."""
import importlib.util
import os
from pathlib import Path

spec = importlib.util.spec_from_file_location("t3", Path(__file__).with_name("t3.py"))
t3 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t3)

HEAD = "run_id,scenario,params,repetition,seed,verdict,reason,duration,judges,path\n"


def row(scen, verdict, judges="physical_verdict=passed", dur="100", rep="0", reason=""):
    return f"x,/a/{scen},,{rep},,{verdict},{reason},{dur},{judges},/p\n"


def test_a_retry_counts_by_its_last_try(tmp_path):
    p = tmp_path / "summary.csv"
    p.write_text(HEAD + row("i7.xosc", "error") + row("i7.xosc", "failed"))
    rows = t3.read_summary(p)
    assert [(r["scenario"], r["verdict"]) for r in rows] == [("i7.xosc", "failed")]


def test_a_changed_verdict_or_judge_is_a_problem_a_slow_run_a_note(tmp_path):
    ref, got = tmp_path / "ref.csv", tmp_path / "got.csv"
    ref.write_text(HEAD + row("a.xosc", "passed") + row("b.xosc", "passed") + row("c.xosc", "passed"))
    got.write_text(HEAD + row("a.xosc", "failed", reason="contact")
                   + row("b.xosc", "passed", judges="physical_verdict=failed")
                   + row("c.xosc", "passed", dur="200") + row("d.xosc", "passed"))
    problems, notes = t3.compare_rows(t3.read_summary(ref), t3.read_summary(got))
    assert problems == ["a.xosc r0: failed, the reference passed (contact)",
                        "b.xosc r0: judge physical_verdict failed, the reference passed"]
    assert notes == ["c.xosc r0: 200 s, the reference 100 s",
                     "d.xosc r0: not in the reference (passed)"]


def test_a_missing_run_is_a_problem(tmp_path):
    ref, got = tmp_path / "ref.csv", tmp_path / "got.csv"
    ref.write_text(HEAD + row("a.xosc", "passed") + row("a.xosc", "passed", rep="1"))
    got.write_text(HEAD + row("a.xosc", "passed"))
    problems, _ = t3.compare_rows(t3.read_summary(ref), t3.read_summary(got))
    assert problems == ["a.xosc r1: no result (the reference has passed)"]


def test_the_manifest_names_existing_suites():
    doc = t3.load()
    for e in doc["suites"]:
        assert (t3.ROOT / e["project"] / "suites" / f"{e['suite']}.yaml").is_file(), e["key"]
    assert len({e["key"] for e in doc["suites"]}) == len(doc["suites"])


def test_parts_cover_every_suite_once_keeping_projects_whole():
    doc = t3.load()
    parts = [t3.selected(doc, [], f"{i}/3") for i in (1, 2, 3)]
    keys = [e["key"] for p in parts for e in p]
    assert sorted(keys) == sorted(e["key"] for e in doc["suites"])
    owners = {}
    for i, p in enumerate(parts):
        for e in p:
            assert owners.setdefault(e["project"], i) == i
    assert all(parts)


def test_a_busy_gpu_is_one_at_half_its_memory_or_load():
    quiet = [{"gpu": 0, "used_mib": 3000, "total_mib": 32000, "util_pct": 10}]
    assert not t3.busy(quiet)
    assert t3.busy([dict(quiet[0], util_pct=97)])
    assert t3.busy([dict(quiet[0], used_mib=25000)])
    assert t3.busy(quiet, load=330.0, cores=48)
    assert not t3.busy(quiet, load=11.0, cores=48)


def test_list_runs_on_the_shipped_manifest(capsys):
    """main() end to end on scenarios/t3.yaml (a local once shadowed load(): list crashed)."""
    assert t3.main(["list", "--part", "2/3"]) == 0
    assert "isaac_road_physics/road" in capsys.readouterr().out


# --- record: the stack's GPU gated, the results' own commit (077, 118's T3 part 1) ----------

TWO = [{"gpu": 0, "used_mib": 3000, "total_mib": 32000, "util_pct": 5},
       {"gpu": 1, "used_mib": 30000, "total_mib": 32000, "util_pct": 90}]   # another user's


def test_only_the_stacks_gpu_is_gated_the_load_host_wide():
    assert not t3.busy(TWO, gated=[0])           # the busy card is not the stack's
    assert t3.busy(TWO, gated=[1])               # the stack's card is busy
    assert t3.busy(TWO)                          # no setting: every card
    assert t3.busy(TWO, load=40.0, cores=32, gated=[0])   # the load is the host's


def test_the_stacks_gpu_sources_in_swbs_order():
    seats = {"seats": {"count": 2, "gpus": [1, 0]}}
    env = {"CUDA_VISIBLE_DEVICES": "1"}
    # 1. host gpu, over everything
    assert t3.stack_gpus(dict(seats, gpu=0), environ=env)[0] == [0]
    # 2. the seat's seats.gpus entry: --seat, host seat, auto as swb run resolves it
    assert t3.stack_gpus(seats, seat_arg="0", environ=env) == ([1], "seats.gpus[0] (seat 0)")
    assert t3.stack_gpus(dict(seats, seat=1), environ=env)[0] == [0]
    assert t3.stack_gpus(seats, environ=env, resolve=lambda h: 1) == \
        ([0], "seats.gpus[1] (seat auto)")
    assert t3.stack_gpus(seats, environ=env, resolve=lambda h: None)[0] == [0, 1]
    # 3. CUDA_VISIBLE_DEVICES: indices in its order, UUIDs mapped, an unknown one refused
    assert t3.stack_gpus({}, environ={"CUDA_VISIBLE_DEVICES": "1,0"})[0] == [1, 0]
    uuids = lambda: {"GPU-aaaa-1111": 0, "GPU-bbbb-2222": 1}   # noqa: E731
    assert t3.stack_gpus({}, environ={"CUDA_VISIBLE_DEVICES": "GPU-bbbb-2222"},
                         uuids=uuids)[0] == [1]
    try:
        t3.stack_gpus({}, environ={"CUDA_VISIBLE_DEVICES": "GPU-cccc"}, uuids=uuids)
        raise AssertionError("an unknown UUID must be refused")
    except t3.GpuRefused as e:
        assert "GPU-cccc is not a GPU nvidia-smi lists" in str(e)
    # 4. nothing set: every card
    assert t3.stack_gpus({}, environ={})[0] is None


def _results(tmp_path, commits, dirty=False, subs=None):
    """A project's suite results on a host: summary.csv and each run's result.json."""
    import json
    logs = tmp_path / "logs"
    suite = logs / "zz_t3test" / "suites" / "s"
    suite.mkdir(parents=True)
    lines = [HEAD]
    for i, c in enumerate(commits):
        rid = f"2026-11-17T00-00-0{i}_a{i}_r0"
        run = logs / "zz_t3test" / "runs" / rid
        run.mkdir(parents=True)
        run.joinpath("result.json").write_text(json.dumps({"provenance": {
            "swb": c + ("+dirty" if dirty else ""),
            "submodules": subs or {"src/swb_isaac_world": "f00"}}}))
        lines.append(f"{rid},/a/a{i}.xosc,,0,,passed,,100,,/p\n")
    (suite / "summary.csv").write_text("".join(lines))
    m = tmp_path / "t3.yaml"
    m.write_text(f"references: {tmp_path / 'refs'}\n"
                 "suites:\n  - {project: scenarios/projects/zz_t3test, suite: s, sim: carla}\n")
    return logs, m


def _quiet(monkeypatch, gpus=TWO, host=None):
    monkeypatch.setattr(t3, "commit", lambda: "bbbbbbb")            # the script's checkout: B
    monkeypatch.setattr(t3, "gpu_state", lambda: gpus)
    monkeypatch.setattr(t3, "host_load", lambda: (2.0, 32))
    monkeypatch.setattr(t3, "host_settings", lambda: host if host is not None else {"gpu": 0})


def test_record_names_the_results_commit_not_the_scripts(tmp_path, monkeypatch, capsys):
    import json
    logs, m = _results(tmp_path, ["aaaaaaa", "aaaaaaa"])
    _quiet(monkeypatch)
    assert t3.main(["record", "--manifest", str(m), "--logs", str(logs)]) == 0
    meta = json.loads((tmp_path / "refs" / "zz_t3test" / "s.json").read_text())
    assert meta["commit"] == "aaaaaaa" and meta["recorded_with"] == "bbbbbbb"
    assert meta["submodules"] == {"src/swb_isaac_world": "f00"}
    assert meta["gpu_gated"] == {"indices": [0], "source": "host gpu: 0"}
    assert meta["gpu_at_record"] == TWO and meta["notes"] == []
    # compare reads the same field
    assert t3.main(["compare", "--manifest", str(m), "--logs", str(logs)]) == 0
    assert "runs at aaaaaaa; the reference at aaaaaaa" in capsys.readouterr().out


def test_record_refuses_a_mix_of_commits_and_the_stacks_busy_gpu(tmp_path, monkeypatch, capsys):
    logs, m = _results(tmp_path, ["aaaaaaa", "ccccccc"])
    _quiet(monkeypatch)
    assert t3.main(["record", "--manifest", str(m), "--logs", str(logs)]) == 1
    assert "results from more than one commit" in capsys.readouterr().out
    logs2, m2 = _results(tmp_path / "b", ["aaaaaaa"])
    _quiet(monkeypatch, host={"gpu": 1})
    assert t3.main(["record", "--manifest", str(m2), "--logs", str(logs2)]) == 1
    assert "gated GPU [1] from host gpu: 1" in capsys.readouterr().out


def test_a_dirty_checkout_is_recorded_with_a_note(tmp_path, monkeypatch, capsys):
    """`just build` dirties the tracked Cargo.lock on every fresh RC checkout (G79), and
    provenance holds only the flag: a warning and a note, not a refusal, for this RC."""
    import json
    logs, m = _results(tmp_path, ["aaaaaaa"], dirty=True)
    _quiet(monkeypatch)
    assert t3.main(["record", "--manifest", str(m), "--logs", str(logs)]) == 0
    meta = json.loads((tmp_path / "refs" / "zz_t3test" / "s.json").read_text())
    assert meta["commit"] == "aaaaaaa+dirty" and "G79" in meta["notes"][0]
    assert "warning: results' checkout was dirty" in capsys.readouterr().out


def test_record_refuses_submodules_off_their_pins(tmp_path, monkeypatch, capsys):
    """243's must-fix: a stale submodule after a pull (provenance `sha+`) or one not
    initialised (`sha-`) means the binaries are not the commit's code."""
    logs, m = _results(tmp_path, ["aaaaaaa"], subs={"src/swb_isaac_world": "f2d7e1a+",
                                                    "src/color_names": "c6a3227-",
                                                    "src/sim_vehicle_spec": "9b7e48d"})
    _quiet(monkeypatch)
    assert t3.main(["record", "--manifest", str(m), "--logs", str(logs)]) == 1
    assert ("not recorded, submodules off the commit's pins: src/color_names (c6a3227-), "
            "src/swb_isaac_world (f2d7e1a+); git submodule update --init --recursive, "
            "rebuild, run again") in capsys.readouterr().out
    assert not (tmp_path / "refs" / "zz_t3test" / "s.json").exists()


def test_a_seat_past_the_table_is_refused_not_an_index_error():
    try:
        t3.stack_gpus({"seats": {"gpus": [0, 1]}}, seat_arg="5", environ={})
        raise AssertionError("seat 5 of 2 must be refused")
    except t3.GpuRefused as e:
        assert "seat 5: seats.gpus lists 2 seats" in str(e)


# t3 run's kill path (077: a `&` job ignored its group's SIGINT; a SIGTERM to t3 left the
# stack its child had just started up). A fake `ros2` on PATH: `assets fetch` exits 0; `run`
# starts a "stack" in its own session (as swb run does), says so as swb run does, and on
# SIGINT exits 130 without stopping it (a suite without --stop-stack leaves it up).
FAKE_ROS2 = r"""#!/usr/bin/env python3
import os, signal, subprocess, sys, time
if sys.argv[1:4] == ["run", "swb", "assets"]:
    sys.exit(0)
stack = subprocess.Popen(["sleep", "300"], start_new_session=True)
open(os.environ["T3_FAKE_STACK"], "w").write(str(stack.pid))
print(f"stack    starting on seat 0 (pid {stack.pid}, console x)", flush=True)
def on_int(*a):
    time.sleep(float(os.environ.get("T3_FAKE_STOP_S", "0")))   # a stack slow to stop
    sys.exit(130)
signal.signal(signal.SIGINT, on_int)
for i in range(int(os.environ.get("T3_FAKE_LINES", "600"))):
    print(f"line {i}", flush=True)
    time.sleep(0.05)
"""


def _t3_child(tmp_path, env_extra=None, **popen):
    import os
    import signal
    import subprocess
    import sys
    import yaml
    bin_ = tmp_path / "bin"
    bin_.mkdir()
    (bin_ / "ros2").write_text(FAKE_ROS2)
    (bin_ / "ros2").chmod(0o755)
    proj = tmp_path / "proj"
    proj.mkdir()
    manifest = tmp_path / "t3.yaml"
    manifest.write_text(yaml.safe_dump({"references": str(tmp_path / "refs"), "suites": [
        {"project": str(proj), "suite": "s", "sim": "carla"}]}))
    env = dict(os.environ, PATH=f"{bin_}{os.pathsep}{os.environ['PATH']}",
               T3_FAKE_STACK=str(tmp_path / "stack.pid"), **(env_extra or {}))
    ignore = lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)   # a script's `&` job
    return subprocess.Popen([sys.executable, str(Path(__file__).with_name("t3.py")), "run",
                             "--manifest", str(manifest), "--work", str(tmp_path / "work")],
                            env=env, preexec_fn=ignore, text=True, **popen)


def _alive(pid):
    import os
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    with open(f"/proc/{pid}/stat") as f:
        return f.read().split(")")[-1].split()[0] != "Z"


def _wait_for(cond, timeout=20):
    import time
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.1)
    return False


def test_a_sigterm_to_t3_stops_its_step_and_the_stack_it_started(tmp_path):
    import signal
    import subprocess
    p = _t3_child(tmp_path, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        pidfile = tmp_path / "stack.pid"
        assert _wait_for(lambda: pidfile.is_file() and pidfile.read_text())
        stack = int(pidfile.read_text())
        with open(f"/proc/{p.pid}/status") as f:   # SIGINT taken back, though inherited ignored
            sigign = int(next(x for x in f if x.startswith("SigIgn:")).split()[1], 16)
        assert not sigign & (1 << (signal.SIGINT - 1))
        p.send_signal(signal.SIGTERM)              # t3 alone, as 139's abort did
        code = p.wait(timeout=30)
    finally:
        if p.poll() is None:
            p.kill()
    assert code == 143                             # SIGTERM's
    assert not _alive(stack), "the stack the step started is still up"
    assert "t3: interrupted; what this run started is stopped" in p.stderr.read()


def test_t3s_stdout_gone_does_not_break_the_step(tmp_path):
    import subprocess
    p = _t3_child(tmp_path, {"T3_FAKE_LINES": "40"}, stdout=subprocess.PIPE,
                  stderr=subprocess.DEVNULL)
    try:
        assert p.stdout.readline().startswith("[t3]")
        p.stdout.close()                         # t3's reader is gone: its writes get EPIPE
        code = p.wait(timeout=30)
    finally:
        if p.poll() is None:
            p.kill()
    assert code == 0
    logs = list((tmp_path / "work" / "logs" / "proj").glob("*_s.log"))
    assert len(logs) == 1 and "line 39" in logs[0].read_text()   # the step wrote to the end
    stack = int((tmp_path / "stack.pid").read_text())
    assert not _alive(stack)                     # the project's stack is stopped after it


def test_a_second_sigint_hurries_the_stop_and_never_abandons_it(tmp_path):
    import signal
    import subprocess
    import time
    p = _t3_child(tmp_path, {"T3_FAKE_STOP_S": "60"}, stdout=subprocess.DEVNULL,
                  stderr=subprocess.PIPE)
    try:
        pidfile = tmp_path / "stack.pid"
        assert _wait_for(lambda: pidfile.is_file() and pidfile.read_text())
        stack = int(pidfile.read_text())
        t0 = time.time()
        p.send_signal(signal.SIGINT)               # the step takes 60 s to stop on SIGINT
        time.sleep(1)
        p.send_signal(signal.SIGINT)               # Ctrl-C again: on to SIGTERM, not abandoned
        code = p.wait(timeout=30)
    finally:
        if p.poll() is None:
            p.kill()
    assert code == 130 and time.time() - t0 < 30
    assert not _alive(stack), "the stack the step started is still up"


def test_a_sigterm_with_t3s_stdout_and_stderr_gone_still_stops_the_stack(tmp_path):
    import signal
    import subprocess
    r, w = os.pipe()
    p = _t3_child(tmp_path, stdout=w, stderr=w)    # `t3.py run ... 2>&1 | head -1`
    os.close(w)
    try:
        with os.fdopen(r) as reader:
            assert reader.readline().startswith("[t3]")
        pidfile = tmp_path / "stack.pid"           # the reader is gone, both streams with it
        assert _wait_for(lambda: pidfile.is_file() and pidfile.read_text())
        stack = int(pidfile.read_text())
        p.send_signal(signal.SIGTERM)
        code = p.wait(timeout=30)
    finally:
        if p.poll() is None:
            p.kill()
    assert code == 143
    assert not _alive(stack), "the stack the step started is still up"
