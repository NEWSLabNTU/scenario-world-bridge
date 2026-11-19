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
    # 2. the seat's seats.gpus entry: --seat, host seat, auto as scensim run resolves it
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


def _results(tmp_path, commits, dirty=False, subs=None, checkout=None):
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
            **({"checkout": checkout} if checkout else {}),
            "submodules": subs or {"src/scensim_isaac_world": "f00"}}}))
        lines.append(f"{rid},/a/a{i}.xosc,,0,,passed,,100,,/p\n")
    (suite / "summary.csv").write_text("".join(lines))
    stamps = tmp_path / "work" / "stamps"           # this host's `t3.py run` of it (G82)
    stamps.mkdir(parents=True)
    (stamps / "zz_t3test__s.json").write_text(json.dumps(
        {"key": "zz_t3test/s", "started": "2026-11-17T00-00-00"}))
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
    assert t3.main(["record", "--manifest", str(m), "--logs", str(logs), "--work", str(logs.parent / "work")]) == 0
    meta = json.loads((tmp_path / "refs" / "zz_t3test" / "s.json").read_text())
    assert meta["commit"] == "aaaaaaa" and meta["recorded_with"] == "bbbbbbb"
    assert meta["summary"] == "<host logs>/zz_t3test/suites/s/summary.csv"   # no host path
    assert meta["submodules"] == {"src/scensim_isaac_world": "f00"}
    assert meta["gpu_gated"] == {"indices": [0], "source": "host gpu: 0"}
    assert meta["gpu_at_record"] == TWO and meta["notes"] == []
    # compare reads the same field
    assert t3.main(["compare", "--manifest", str(m), "--logs", str(logs), "--work", str(logs.parent / "work")]) == 0
    assert "runs at aaaaaaa; the reference at aaaaaaa" in capsys.readouterr().out



def test_a_build_older_than_the_checkout_names_both(tmp_path, monkeypatch, capsys):
    """083 T3 on 243: runs from an install built at one commit, Python from a checkout at
    another (symlink-install), were all named by the build's commit alone."""
    import json
    logs, m = _results(tmp_path, ["aaaaaaa", "aaaaaaa"], dirty=True, checkout="ddddddd")
    _quiet(monkeypatch)
    w = ["--manifest", str(m), "--logs", str(logs), "--work", str(logs.parent / "work")]
    assert t3.main(["record", *w]) == 0
    meta = json.loads((tmp_path / "refs" / "zz_t3test" / "s.json").read_text())
    assert meta["commit"] == "aaaaaaa+dirty (checkout ddddddd)"
    assert "G79" in meta["notes"][0]          # still noted dirty
    assert t3.main(["compare", *w]) == 0
    assert "runs at aaaaaaa+dirty (checkout ddddddd)" in capsys.readouterr().out
    prov = t3.results_provenance([{"run_id": r.name, "scenario": ""} for r in
                                  (logs / "zz_t3test" / "runs").iterdir()],
                                 logs / "zz_t3test" / "runs")
    assert list(prov["commits"]) == ["aaaaaaa+dirty (checkout ddddddd)"]

def test_record_refuses_a_mix_of_commits_and_the_stacks_busy_gpu(tmp_path, monkeypatch, capsys):
    logs, m = _results(tmp_path, ["aaaaaaa", "ccccccc"])
    _quiet(monkeypatch)
    assert t3.main(["record", "--manifest", str(m), "--logs", str(logs), "--work", str(logs.parent / "work")]) == 1
    assert "results from more than one commit" in capsys.readouterr().out
    logs2, m2 = _results(tmp_path / "b", ["aaaaaaa"])
    _quiet(monkeypatch, host={"gpu": 1})
    assert t3.main(["record", "--manifest", str(m2), "--logs", str(logs2), "--work", str(logs2.parent / "work")]) == 1
    assert "gated GPU [1] from host gpu: 1" in capsys.readouterr().out


def test_a_dirty_checkout_is_recorded_with_a_note(tmp_path, monkeypatch, capsys):
    """`just build` dirties the tracked Cargo.lock on every fresh RC checkout (G79), and
    provenance holds only the flag: a warning and a note, not a refusal, for this RC."""
    import json
    logs, m = _results(tmp_path, ["aaaaaaa"], dirty=True)
    _quiet(monkeypatch)
    assert t3.main(["record", "--manifest", str(m), "--logs", str(logs), "--work", str(logs.parent / "work")]) == 0
    meta = json.loads((tmp_path / "refs" / "zz_t3test" / "s.json").read_text())
    assert meta["commit"] == "aaaaaaa+dirty" and "G79" in meta["notes"][0]
    assert "warning: results' checkout was dirty" in capsys.readouterr().out


def test_record_refuses_submodules_off_their_pins(tmp_path, monkeypatch, capsys):
    """243's must-fix: a stale submodule after a pull (provenance `sha+`) or one not
    initialised (`sha-`) means the binaries are not the commit's code."""
    logs, m = _results(tmp_path, ["aaaaaaa"], subs={"src/scensim_isaac_world": "f2d7e1a+",
                                                    "src/color_names": "c6a3227-",
                                                    "src/sim_vehicle_spec": "9b7e48d"})
    _quiet(monkeypatch)
    assert t3.main(["record", "--manifest", str(m), "--logs", str(logs), "--work", str(logs.parent / "work")]) == 1
    assert ("not recorded, submodules off the commit's pins: src/color_names (c6a3227-), "
            "src/scensim_isaac_world (f2d7e1a+); git submodule update --init --recursive, "
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
# starts a "stack" in its own session (as scensim run does), says so as scensim run does, and on
# SIGINT exits 130 without stopping it (a suite without --stop-stack leaves it up).
FAKE_ROS2 = r"""#!/usr/bin/env python3
import os, signal, subprocess, sys, time
if sys.argv[1:4] == ["run", "scensim", "assets"]:
    sys.exit(0)
log = os.environ.get("T3_FAKE_LOG")
if log:   # was the previous suite's stack gone when this one started? (G82)
    try:
        prev = int(open(os.environ["T3_FAKE_STACK"]).read())
        os.kill(prev, 0)
        alive = open(f"/proc/{prev}/stat").read().split(")")[-1].split()[0] != "Z"
    except (OSError, ValueError):
        alive = False
    open(log, "a").write(f"{sys.argv[4]} previous_stack_alive={alive}\n")
stack = subprocess.Popen(["sleep", "300"], start_new_session=True)
open(os.environ["T3_FAKE_STACK"], "w").write(str(stack.pid))
open(os.environ["T3_FAKE_STACK"] + ".all", "a").write(f"{stack.pid}\n")
print(f"stack    proj on seat 0: starting (pid {stack.pid}, console x)", flush=True)
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
                             "--manifest", str(manifest), "--work", str(tmp_path / "work"),
                             "--logs", str(tmp_path / "logs")],
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


def _two_suites(tmp_path, monkeypatch, rec):
    """t3 run, in process, of one project's two suites (s1 on Town02, s2 on Town01) with the
    fake ros2; the stack's record is ``rec``. Returns the fake's start log."""
    bin_ = tmp_path / "bin"
    bin_.mkdir()
    (bin_ / "ros2").write_text(FAKE_ROS2)
    (bin_ / "ros2").chmod(0o755)
    proj = tmp_path / "proj"
    (proj / "assets" / "maps" / "Town01").mkdir(parents=True)
    monkeypatch.setenv("PATH", f"{bin_}{os.pathsep}{os.environ['PATH']}")
    monkeypatch.setenv("T3_FAKE_STACK", str(tmp_path / "stack.pid"))
    monkeypatch.setenv("T3_FAKE_LOG", str(tmp_path / "starts.log"))
    monkeypatch.setenv("T3_FAKE_LINES", "5")
    monkeypatch.setattr(t3, "suite_maps", lambda d, s: {"s1": ["Town02"], "s2": ["Town01"]}[s])
    monkeypatch.setattr(t3, "stack_record", lambda logs, d, seat: dict(rec, project=str(proj)))
    entries = [{"project": str(proj), "suite": s, "sim": "isaac", "key": f"proj/{s}"}
               for s in ("s1", "s2")]
    try:
        assert t3.run(entries, tmp_path / "work", False, tmp_path / "logs") == 0
    finally:
        for pid in (tmp_path / "stack.pid.all").read_text().split():
            try:
                os.kill(int(pid), 9)
            except ProcessLookupError:
                pass
    return (tmp_path / "starts.log").read_text()


def test_a_suite_the_stack_cannot_serve_gets_a_new_stack_after_the_old_is_gone(
        tmp_path, monkeypatch, capsys):
    """241: isaac_basic/towns left the Isaac world on Town02, and rear_contact (Town01) was
    refused (G82: scensim's switch_problem refuses even the stack's own first world)."""
    starts = _two_suites(tmp_path, monkeypatch,
                         {"simulator": "isaac", "map": "Town02", "preload": ["Town02"]})
    assert "suites/s2.yaml previous_stack_alive=False" in starts
    assert "[t3] map Town02 -> Town01: restarting the stack" in capsys.readouterr().out


def test_a_stack_that_can_switch_is_kept_between_suites(tmp_path, monkeypatch, capsys):
    starts = _two_suites(tmp_path, monkeypatch, {"simulator": "carla", "map": "Town02",
                                                 "maps": ["Town01", "Town02"]})
    assert "suites/s2.yaml previous_stack_alive=True" in starts
    assert "restarting the stack" not in capsys.readouterr().out


def test_record_takes_only_this_hosts_last_run_of_the_suite(tmp_path, monkeypatch, capsys):
    """G82: rear_contact's summary.csv, from an earlier commit's run, would have been taken."""
    import json
    stamp = tmp_path / "work" / "stamps" / "zz_t3test__s.json"
    logs, m = _results(tmp_path, ["aaaaaaa"])
    args = ["record", "--manifest", str(m), "--logs", str(logs), "--work", str(tmp_path / "work")]
    _quiet(monkeypatch)
    # started after the results' run
    stamp.write_text(json.dumps({"key": "zz_t3test/s", "started": "2026-11-17T09-00-00"}))
    assert t3.main(args) == 1
    assert ("not recorded, results not from this host's last `t3.py run` of it: "
            "2026-11-17T00-00-00_a0_r0 (before this run, 2026-11-17T09-00-00)"
            in capsys.readouterr().out)
    # the run's rows, though t3's own checkout is another commit (243: the RC's runs, built
    # at the tag, recorded with a newer t3)
    stamp.write_text(json.dumps({"key": "zz_t3test/s", "started": "2026-11-17T00-00-00"}))
    assert t3.commit() == "bbbbbbb"                 # _quiet: t3's checkout, not the results'
    assert t3.main(args) == 0
    meta = json.loads((tmp_path / "refs" / "zz_t3test" / "s.json").read_text())
    assert meta["commit"] == "aaaaaaa" and meta["recorded_with"] == "bbbbbbb"
    capsys.readouterr()
    (tmp_path / "refs" / "zz_t3test" / "s.json").unlink()
    # never run here
    stamp.unlink()
    assert t3.main(args) == 1
    assert "not recorded, no `t3.py run` of it on this host" in capsys.readouterr().out
    assert not (tmp_path / "refs" / "zz_t3test" / "s.json").exists()


def test_the_seats_record_is_stopped_only_when_its_keeper_is_this_stacks(tmp_path, monkeypatch):
    """By a project's end the seat may hold another's stack: its keeper is not t3's."""
    import subprocess
    calls = []
    import scensim.stack
    monkeypatch.setattr(scensim.stack, "stop_keeper", lambda rec: calls.append(rec["pid"]))
    mine = subprocess.Popen(["sleep", "30"], start_new_session=True)
    other = subprocess.Popen(["sleep", "30"], start_new_session=True)
    try:
        monkeypatch.setattr(t3, "stack_record", lambda logs, d, seat: {"pid": other.pid})
        t3.stop_stack((mine.pid, 0), None, tmp_path, grace=1)
        assert calls == [] and other.poll() is None      # not ours: left alone
        mine = subprocess.Popen(["sleep", "30"], start_new_session=True)
        monkeypatch.setattr(t3, "stack_record", lambda logs, d, seat: {"pid": mine.pid})
        t3.stop_stack((mine.pid, 0), None, tmp_path, grace=1)
        assert calls == [mine.pid]
    finally:
        for p in (mine, other):
            p.kill()
            p.wait()
    assert t3.descends(os.getpid(), os.getppid()) and not t3.descends(os.getppid(), os.getpid())


def test_compare_takes_references_recorded_before_the_stamps(tmp_path, monkeypatch, capsys):
    """The release compare runs against the references 118 and 241 recorded with 876f8b3's
    t3.py, which wrote no run stamps (G82): compare needs none."""
    import json
    logs, m = _results(tmp_path, ["aaaaaaa"])
    (tmp_path / "work" / "stamps" / "zz_t3test__s.json").unlink()   # no stamp on this host
    ref = tmp_path / "refs" / "zz_t3test" / "s.csv"                # 876f8b3's reference
    ref.parent.mkdir(parents=True)
    ref.write_text("scenario,params,repetition,seed,verdict,judges,duration\n"
                   "a0.xosc,,0,,passed,,100\n")
    ref.with_suffix(".json").write_text(json.dumps({
        "commit": "aaaaaaa", "submodules": {"src/scensim_isaac_world": "f00"},
        "recorded_with": "876f8b3", "host": "h", "summary": "s", "runs": 1,
        "gpu_at_record": TWO, "gpu_gated": {"indices": [0], "source": "host gpu: 0"},
        "load_at_record": [2.0, 32], "notes": []}))
    _quiet(monkeypatch)
    assert t3.main(["compare", "--manifest", str(m), "--logs", str(logs),
                    "--work", str(tmp_path / "work")]) == 0
    assert "runs at aaaaaaa; the reference at aaaaaaa" in capsys.readouterr().out


def test_the_summary_path_is_relative_to_the_log_root_through_a_link(tmp_path):
    real = tmp_path / "disk" / "scensim-logs"
    summ = real / "proj" / "suites" / "s" / "summary.csv"
    summ.parent.mkdir(parents=True)
    summ.write_text("")
    (tmp_path / "logs").symlink_to(real)            # the root as the host names it
    want = "<host logs>/proj/suites/s/summary.csv"
    assert t3.host_relative(summ, tmp_path / "logs") == want
    assert t3.host_relative(tmp_path / "logs" / "proj" / "suites" / "s" / "summary.csv",
                            real) == want
    assert t3.host_relative(summ, tmp_path / "elsewhere") == want   # outside: the last 4 parts


def test_the_started_line_of_either_scensim_is_read():
    """T3 on CARLA 1.0: scensim run's "stack <name> on seat N: starting (pid P" was not read, so
    a project's stack stayed up after its last suite and held CARLA's world-server lock, and the
    next project's world server was refused. The older form still reads."""
    import t3
    for line, want in (("stack    basic on seat 0: starting (pid 808387, console /x)", ("0", "808387")),
                       ("stack    starting on seat 2 (pid 55, console x)", ("2", "55"))):
        m = t3.STARTED.match(line)
        assert m and (m.group(1), m.group(2)) == want, line
    assert t3.STARTED.match("stack    basic on seat 0: started (CARLA, Town01); left up") is None
