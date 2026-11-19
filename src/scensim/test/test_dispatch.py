"""scensim run --seats (roadmap 088 step 2): the dispatcher, its admission, its overload signal,
its retirement of a seat, and the one report, with fake workers (no stack, no GPU)."""
import io
import json
import sys
import textwrap
from pathlib import Path
from types import SimpleNamespace

import pytest

from scensim import config, dispatch, results, run, runs as runsmod

SEATS = {"domains": {"base": 1, "stride": 3}, "ports": {"base": 5555, "stride": 100},
         "web": {"base": 8080, "stride": 10}, "count": 4, "carla": {"base": 3000, "stride": 10}}
CFG = {"vram_per_seat_mb": 3000, "load_per_seat": 12, "skip_limit": 12, "start_gap": 0,
       "overload_load": 32}


def host(**over):
    h = {**config.DEFAULTS, "seats": dict(SEATS), "seat_locks": "/nonexistent"}
    h.update(over)
    return h


def specs(n, name="s"):
    return [run.RunSpec(scenario=Path(f"/x/{name}{i}.xosc")) for i in range(n)]


# ------------------------------------------------------------------------------ the seats

def test_seats_by_number_or_auto():
    got = dispatch.parse_seats("0,2", host(), holder=lambda i, d: {})
    assert [s.index for s in got] == [0, 2] and [s.carla_port for s in got] == [3000, 3020]
    held = {1: {"pid": 1}}
    got = dispatch.parse_seats("auto", host(), holder=lambda i, d: held.get(i, {}))
    assert [s.index for s in got] == [0, 2, 3]


@pytest.mark.parametrize("value, words", [
    ("0,0", "named twice"), ("9", "not in the seat table"), ("a,b", "seat numbers"),
    (" ", "seat numbers")])
def test_bad_seat_lists_are_refused(value, words):
    with pytest.raises(dispatch.DispatchRefused, match=words):
        dispatch.parse_seats(value, host(), holder=lambda i, d: {})


def test_seats_need_a_carla_each():
    h = host(seats={k: v for k, v in SEATS.items() if k != "carla"})
    with pytest.raises(dispatch.DispatchRefused, match="seats.carla"):
        dispatch.parse_seats("0,1", h, holder=lambda i, d: {})


def test_auto_with_every_seat_held_is_refused():
    with pytest.raises(dispatch.DispatchRefused, match="every seat"):
        dispatch.parse_seats("auto", host(), holder=lambda i, d: {"pid": 1})


def test_dispatch_settings_are_checked():
    assert dispatch.settings({"dispatch": {"skip_limit": 5}})["skip_limit"] == 5
    with pytest.raises(dispatch.DispatchRefused, match="dispatch.load_per_seat"):
        dispatch.settings({"dispatch": {"load_per_seat": "high"}})


# ------------------------------------------------------------------------------ admission

def test_admission_takes_free_vram_and_the_load():
    seat = SimpleNamespace(gpu=1)
    ok = dict(free_mb=lambda g: 8000, load=lambda: (4.0, 0, 0), cores=lambda: 32)
    assert dispatch.admission(seat, CFG, **ok) == ""
    why = dispatch.admission(seat, CFG, **{**ok, "free_mb": lambda g: 2048})
    assert "GPU 1 has 2048 MiB free" in why
    why = dispatch.admission(seat, CFG, **{**ok, "load": lambda: (25.0, 0, 0)})
    assert "load is 25.0 on 32 cores" in why
    # nvidia-smi absent: the load alone decides
    assert dispatch.admission(seat, CFG, **{**ok, "free_mb": lambda g: None}) == ""


# ---------------------------------------------------------------------- the overload signal

def test_acb_skips_counts_new_lines_only(tmp_path):
    node = tmp_path / "pilot-ego" / "2026-x" / "node" / "acb_bridge"
    node.mkdir(parents=True)
    out = node / "out"
    out.write_text("\x1b[33mWARN\x1b[0m Skipped 3 CARLA frame(s)\nSkipped 4 CARLA frames\n")
    console = tmp_path / "pilot-ego" / "console.log"
    console.write_text("[acb_bridge-1] Skipped 2 CARLA frames\n[other-2] Skipped 50 CARLA frames\n")
    offsets = {}
    assert dispatch.acb_skips(tmp_path, offsets) == 9
    assert dispatch.acb_skips(tmp_path, offsets) == 0
    with open(out, "a") as f:
        f.write("Skipped 5 CARLA frames\n")
    assert dispatch.acb_skips(tmp_path, offsets) == 5
    assert dispatch.acb_skips(tmp_path / "none", {}) == 0


# ------------------------------------------------------------------------------ the worker

def test_the_worker_serves_runs_until_stop(tmp_path):
    ctx = SimpleNamespace(stack={"stack_id": "st", "proc": "/p"}, runs_root=tmp_path,
                          pilots_broken=None)
    ran = []

    def run_one(c, spec, attempt):
        ran.append((spec.key, attempt))
        return results.RunResult(f"run-{spec.key}", results.PASSED)
    inp = io.StringIO("".join(json.dumps(o) + "\n" for o in (
        {"run": True, "spec": dispatch.spec_to_json(specs(1)[0]), "attempt": 1},
        {"stop": True},
        {"run": True, "spec": dispatch.spec_to_json(specs(2)[1]), "attempt": 0})))
    out = io.StringIO()
    assert dispatch.worker_loop(ctx, inp, out, run_one=run_one, line=lambda r, d, e: "L",
                                outcome=lambda r, e: 0) == 0
    events = [json.loads(x) for x in out.getvalue().splitlines()]
    assert [e["event"] for e in events] == ["ready", "done"]
    assert ran == [("s0_r0", 1)]
    assert events[1]["run_dir"] == str(tmp_path / "run-s0_r0")


def test_a_worker_whose_pilots_break_retires(tmp_path):
    ctx = SimpleNamespace(stack={}, runs_root=tmp_path, pilots_broken=None)

    def run_one(c, spec, attempt):
        c.pilots_broken = "ego"
        return results.RunResult("r", results.ERROR)
    inp = io.StringIO(json.dumps({"run": True, "spec": dispatch.spec_to_json(specs(1)[0])}) +
                      "\n")
    out = io.StringIO()
    dispatch.worker_loop(ctx, inp, out, run_one=run_one, line=lambda r, d, e: "L",
                         outcome=lambda r, e: 1)
    assert [json.loads(x)["event"] for x in out.getvalue().splitlines()] == [
        "ready", "done", "retired"]


def test_spec_round_trip():
    s = run.RunSpec(scenario=Path("/a/b.xosc"), parameters={"v": 3}, repetition=2, seed=7,
                    expect="failed", ssv2={"frame_rate": 20})
    assert dispatch.spec_from_json(json.loads(json.dumps(dispatch.spec_to_json(s)))) == s


# ------------------------------------------------------------------------- the dispatcher

FAKE_WORKER = textwrap.dedent('''\
    """A seat's worker in miniature: behaviour from FAKE_PLAN (JSON {seat: {...}})."""
    import json, os, sys, time
    seat = sys.argv[1]
    root, plan = sys.argv[2], json.loads(os.environ["FAKE_PLAN"]).get(seat, {})
    def emit(**e):
        sys.stdout.write(json.dumps(e) + "\\n"); sys.stdout.flush()
    print(f"seat {seat} worker up", file=sys.stderr, flush=True)
    if plan.get("retire_at_start"):
        emit(event="retired", why="its stack did not become ready: no readiness"); sys.exit(0)
    time.sleep(plan.get("start_delay", 0))
    emit(event="ready", stack=f"stack-{seat}", proc=f"/proc-{seat}")
    n = 0
    for raw in sys.stdin:
        o = json.loads(raw)
        if o.get("stop"):
            break
        n += 1
        key = o["spec"]["scenario"].rsplit("/", 1)[-1].removesuffix(".xosc") + "_r0"
        if plan.get("die_on") == n:
            os._exit(9)
        broken = plan.get("broken_on") == n
        time.sleep(plan.get("run_time", 0.05))
        verdict = plan.get("verdicts", {}).get(key, "passed")
        if o.get("attempt"):
            verdict = "passed"
        d = os.path.join(root, f"seat{seat}-{n}-{key}")
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "result.json"), "w") as f:
            json.dump({"run_id": os.path.basename(d), "verdict": verdict,
                       "scenario": {"file": key + ".xosc", "repetition": 0},
                       "seat": seat}, f)
        if broken:
            verdict = "error"
        why = "its pilot ego's launch did not load /perception/occupancy_grid_map" if broken else ""
        emit(event="done", key=key, run_dir=d, verdict=verdict,
             line=f"{verdict:8} {key}  seat {seat}", outcome={"passed": 0, "failed": 1}.get(verdict, 2),
             seat_broken=why)
        if broken:
            emit(event="retired", why=why)
            break
    print(f"seat {seat} worker stops its stack", file=sys.stderr, flush=True)
''')


@pytest.fixture
def fake(tmp_path, monkeypatch):
    script = tmp_path / "fake_worker.py"
    script.write_text(FAKE_WORKER)
    runs = tmp_path / "runs"
    runs.mkdir()

    def make(plan=None):
        monkeypatch.setenv("FAKE_PLAN", json.dumps(plan or {}))
        return lambda seat: [sys.executable, str(script), str(seat.index), str(runs)]
    return make


def seats(*idx):
    return [SimpleNamespace(index=i, gpu=0, carla_port=3000 + 10 * i) for i in idx]


def dispatcher(tmp_path, fake, n=6, plan=None, seat_idx=(0, 1), retry=None, **kw):
    suite = tmp_path / "suite"
    suite.mkdir(exist_ok=True)
    out = io.StringIO()
    d = dispatch.Dispatcher(specs(n), seats(*seat_idx), {**CFG, **kw.pop("cfg", {})}, suite,
                            retry, out=out, worker_argv=fake(plan),
                            admit=kw.pop("admit", lambda s: ""),
                            skips=kw.pop("skips", lambda w: 0),
                            load=kw.pop("load", lambda: 4.0), **kw)
    return d, suite, out


def rows(suite: Path):
    import csv
    with open(suite / "summary.csv") as f:
        return list(csv.DictReader(f))


def test_every_run_once_on_whichever_seat_is_free_into_one_report(tmp_path, fake):
    d, suite, out = dispatcher(tmp_path, fake, n=6)
    assert d.run() == 0
    links = sorted(p.name for p in suite.iterdir() if p.is_symlink())
    assert links == [f"s{i}_r0" for i in range(6)]
    seats_used = {json.loads((suite / k / "result.json").read_text())["seat"] for k in links}
    assert seats_used == {"0", "1"}
    assert [r["verdict"] for r in rows(suite)] == ["passed"] * 6
    text = out.getvalue()
    assert "[seat 0] seat 0 worker up" in text and "summary: " in text
    assert "seat 1 worker stops its stack" in text   # every worker was told to stop
    assert all(w.proc.poll() is not None for w in d.workers)


def test_a_failed_run_sets_the_exit_and_an_error_is_retried(tmp_path, fake):
    plan = {"0": {"verdicts": {"s1_r0": "error", "s2_r0": "failed"}},
            "1": {"verdicts": {"s1_r0": "error", "s2_r0": "failed"}}}
    d, suite, out = dispatcher(tmp_path, fake, n=3, plan=plan, retry={"times": 1})
    assert d.run() == 1
    verdicts = {r["run_id"].split("-", 2)[2]: r["verdict"] for r in rows(suite)}
    assert verdicts == {"s0_r0": "passed", "s1_r0": "passed", "s2_r0": "failed"}


def test_a_seat_that_never_becomes_ready_is_retired_and_the_others_run(tmp_path, fake):
    d, suite, out = dispatcher(tmp_path, fake, n=4, plan={"1": {"retire_at_start": True}})
    assert d.run() == 0
    assert "dispatch seat 1: retired: its stack did not become ready" in out.getvalue()
    assert len(rows(suite)) == 4


def test_a_worker_dying_mid_run_hands_its_run_to_another_seat(tmp_path, fake):
    d, suite, out = dispatcher(tmp_path, fake, n=4, plan={"0": {"die_on": 1}})
    assert d.run() == 0
    text = out.getvalue()
    assert "seat 0: retired: its worker ended (exit 9)" in text
    assert "back in the queue (seat 0 retired)" in text
    assert sorted(r["run_id"].split("-", 2)[2] for r in rows(suite)) == [
        f"s{i}_r0" for i in range(4)]


def test_every_seat_retired_leaves_the_rest_unrun(tmp_path, fake):
    plan = {"0": {"retire_at_start": True}, "1": {"retire_at_start": True}}
    d, suite, out = dispatcher(tmp_path, fake, n=3, plan=plan)
    assert d.run() == run.EXIT_INFRA
    assert "every seat is retired; 3 run(s) not run" in out.getvalue()


def test_an_overloaded_run_is_run_again_on_one_seat_fewer(tmp_path, fake):
    """acb_bridge's skips in one run past the limit: that try stays on disk, unlinked; the run
    goes back to the queue once; its seat is parked."""
    first = {"done": False}

    def skips(w):
        # the end of seat 1's first run reports 34 skips; every other look nothing
        if w.seat.index == 1 and w.job is None and not first["done"]:
            first["done"] = True
            return 34
        return 0
    d, suite, out = dispatcher(tmp_path, fake, n=6, seat_idx=(0, 1), skips=skips,
                               plan={"0": {"run_time": 0.3}, "1": {"run_time": 0.05}})
    assert d.run() == 0
    text = out.getvalue()
    assert "overloaded: acb_bridge skipped 34 CARLA frames (limit 12)" in text
    assert "seat 1: the host is overloaded" in text and "1 seat(s) take work" in text
    assert "run again (overloaded)" in text
    assert d.limit == 1 and len(rows(suite)) == 6   # one row per run: the second try linked
    tries = sorted((tmp_path / "runs").iterdir())
    assert len(tries) == 7
    flagged = [json.loads((x / "dispatch.json").read_text()) for x in tries
               if json.loads((x / "dispatch.json").read_text())["overloaded"]]
    assert len(flagged) == 1 and flagged[0]["seat"] == 1 and flagged[0]["acb_skipped_frames"] == 34
    assert all(r["overload"] == "" for r in rows(suite))


def test_a_run_overloaded_twice_keeps_its_verdict_flagged(tmp_path, fake):
    """The host's mean load past overload_load in every run: each run is tried twice, and the
    second try's verdict stands, flagged in summary.csv."""
    d, suite, out = dispatcher(tmp_path, fake, n=2, seat_idx=(0,), load=lambda: 78.4,
                               plan={"0": {"run_time": 1.5}})
    assert d.run() == 0
    got = rows(suite)
    assert len(got) == 2 and all("mean load 78.4 (limit 32)" in r["overload"] for r in got)
    assert out.getvalue().count("run again (overloaded)") == 2


def test_admission_waits_for_room_then_starts_the_seat(tmp_path, fake):
    looks = {"n": 0}

    def admit(seat):
        if seat.index == 1:
            looks["n"] += 1
            return "GPU 0 has 1000 MiB free, a seat needs 3000" if looks["n"] < 3 else ""
        return ""
    d, suite, out = dispatcher(tmp_path, fake, n=6, admit=admit,
                               plan={"0": {"run_time": 0.6}})
    assert d.run() == 0
    text = out.getvalue()
    assert text.count("seat 1: waits: GPU 0 has 1000 MiB free") == 1
    assert "dispatch seat 1: starting its stack" in text


def test_start_gap_spaces_the_seats(tmp_path, fake):
    now = {"t": 0.0}
    d, suite, out = dispatcher(tmp_path, fake, n=2, cfg={"start_gap": 30},
                               clock=lambda: now["t"])
    d._admit_next()
    d._admit_next()
    assert [w.state for w in d.workers] == ["starting", "waiting"]
    now["t"] = 31.0
    d._admit_next()   # the first seat is still coming up: one at a time
    assert [w.state for w in d.workers] == ["starting", "waiting"]
    d.workers[0].state = "ready"
    d._admit_next()
    assert [w.state for w in d.workers] == ["ready", "starting"]
    d.stop(grace=5)


# ------------------------------------------------------------------------------ the command

def test_seats_refuses_the_flags_it_does_not_take(tmp_path, capsys):
    tb = tmp_path / "tb"
    tb.mkdir()
    (tb / "testbench.yaml").write_text("simulator: carla\n")
    for extra in (["--seat", "1"], ["--shard", "1/2"], ["--stop-stack"]):
        rc = run.main([str(tb / "x.xosc"), "--project", str(tb), "--seats", "0,1", *extra],
                      out=io.StringIO())
        assert rc == run.EXIT_REFUSED
        assert "--seats takes no" in capsys.readouterr().err
    rc = run.main([str(tb / "x.xosc"), "--project", str(tb), "--keep-stacks"],
                  out=io.StringIO())
    assert rc == run.EXIT_REFUSED and "--keep-stacks is for --seats" in capsys.readouterr().err


# ------------------------------------------------- against the real printers, not copies

ACB_SOURCE = (Path(__file__).resolve().parents[2] / "autoware_carla_bridge" / "src" /
              "acb_bridge" / "src" / "tick_follower.rs")


def test_the_skip_counter_reads_acb_bridges_own_message(tmp_path):
    """The overload signal is acb_bridge's warning as its source writes it (format string
    taken from tick_follower.rs), not a copy kept here."""
    import re
    if not ACB_SOURCE.is_file():
        pytest.skip("acb's source is not checked out (src/autoware_carla_bridge)")
    m = re.search(r'"(Skipped \{\} CARLA frame[^"]*)"', ACB_SOURCE.read_text())
    assert m, "acb_bridge's skipped-frames warning is no longer in tick_follower.rs"
    fmt = m.group(1).replace("\\\n", "")
    message = fmt.replace("{}", "17", 1).replace("{}", "4242")
    node = tmp_path / "pilot-ego" / "x" / "node" / "acb_bridge"
    node.mkdir(parents=True)
    (node / "out").write_text(f"\x1b[33m WARN\x1b[0m acb_bridge::tick_follower: {message}\n")
    (tmp_path / "pilot-ego" / "console.log").write_text(f"[acb_bridge-1] {message}\n")
    assert dispatch.acb_skips(tmp_path, {}) == 34


def test_the_worker_reports_with_the_runners_own_line_and_exit(tmp_path):
    """No line of scensim run's output is parsed: the worker sends the run's verdict and
    directory as data, and its printed line is run.line's (the real printer)."""
    ctx = SimpleNamespace(stack={}, runs_root=tmp_path, pilots_broken=None)
    spec = run.RunSpec(scenario=Path("/x/a.xosc"), expect="failed")
    res = results.RunResult("2026-x_a_r0", results.FAILED)
    inp = io.StringIO(json.dumps({"run": True, "spec": dispatch.spec_to_json(spec)}) + "\n")
    out = io.StringIO()
    dispatch.worker_loop(ctx, inp, out, run_one=lambda c, s, a: res)
    done = [json.loads(x) for x in out.getvalue().splitlines()][1]
    assert done["verdict"] == "failed" and done["run_dir"] == str(tmp_path / res.run_id)
    assert done["line"] == run.line(res, tmp_path / res.run_id, "failed")
    assert done["outcome"] == run.outcome(res, "failed") == run.EXIT_OK


# ------------------------------------------- issue 064: a seat whose composable did not load

ROS_LAUNCH_LOAD_FAILED = (
    "[ERROR] [launch_ros.actions.load_composable_nodes]: Failed to load node 'lidar_centerpoint' "
    "of type 'autoware::lidar_centerpoint::LidarCenterPointNode' in container "
    "'pointcloud_container': Component constructor threw an exception: Failed to setup TRT "
    "engine.")   # as ros2 launch printed it on 118, 2026-11-18 (a CUDA out of memory)


def test_a_seat_whose_composable_did_not_load_retires_through_issue_064s_check(tmp_path):
    """The worker asks timeline.missing_composables (the real check scensim run's readiness
    uses) of its stack pilots' consoles after an error run: a line ros2 launch printed retires
    the seat, naming the composable; a console without one does not."""
    proc = tmp_path / "proc"
    console = proc / "pilot-ego" / "console.log"
    console.parent.mkdir(parents=True)
    console.write_text("[INFO] [launch]: All log files can be found below /x\n"
                       + ROS_LAUNCH_LOAD_FAILED + "\n")
    ctx = SimpleNamespace(stack={"proc": str(proc), "pilots": [{"entity": "ego"}]},
                          runs_root=tmp_path, pilots_broken=None)
    assert "lidar_centerpoint" in dispatch.stack_pilot_broken(ctx)
    inp = io.StringIO(json.dumps({"run": True, "spec": dispatch.spec_to_json(specs(1)[0])})
                      + "\n" + json.dumps({"run": True,
                                           "spec": dispatch.spec_to_json(specs(2)[1])}) + "\n")
    out = io.StringIO()
    dispatch.worker_loop(ctx, inp, out,
                         run_one=lambda c, s, a: results.RunResult("r", results.ERROR))
    events = [json.loads(x) for x in out.getvalue().splitlines()]
    assert [e["event"] for e in events] == ["ready", "done", "retired"]   # the 2nd run not taken
    assert "did not load" in events[1]["seat_broken"] and "lidar_centerpoint" in events[2]["why"]
    console.write_text("[INFO] [launch]: All log files can be found below /x\n")
    assert dispatch.stack_pilot_broken(ctx) == ""


def test_a_broken_seats_run_goes_to_another_seat(tmp_path, fake):
    d, suite, out = dispatcher(tmp_path, fake, n=4, plan={"0": {"broken_on": 1}})
    assert d.run() == 0
    text = out.getvalue()
    assert "back in the queue (seat 0 is broken)" in text
    assert "seat 0: retired: its pilot ego's launch did not load" in text
    assert sorted(r["verdict"] for r in rows(suite)) == ["passed"] * 4
