"""scensim run --seats (roadmap 088 steps 2 and 3): the dispatcher, its admission (counting the
testbench's Autoware pilots), its overload signals, its retirement of a seat, rollouts and the
one report, with fake workers (no stack, no GPU)."""
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
CFG = {**config.DEFAULTS["dispatch"], "start_gap": 0}


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
    with pytest.raises(dispatch.DispatchRefused, match="dispatch.load_base"):
        dispatch.settings({"dispatch": {"load_base": "high"}})
    # a name that is not a setting is refused, naming the ones there are
    with pytest.raises(dispatch.DispatchRefused, match="dispatch.vram_per_seat_mb is not a "
                                                       "setting.*vram_base_mb"):
        dispatch.settings({"dispatch": {"vram_per_seat_mb": 3000}})


# ------------------------------------------------------------------------------ admission

def test_admission_takes_free_vram_and_the_load():
    seat = SimpleNamespace(gpu=1)
    ok = dict(free_mb=lambda g: 8000, load=lambda: (4.0, 0, 0), cores=lambda: 32)
    assert dispatch.admission(seat, CFG, 1, **ok) == ""
    why = dispatch.admission(seat, CFG, 1, **{**ok, "free_mb": lambda g: 2048})
    assert "GPU 1 has 2048 MiB free, a seat (CARLA and 1 Autoware pilot) needs 3000" in why
    why = dispatch.admission(seat, CFG, 1, **{**ok, "load": lambda: (25.0, 0, 0)})
    assert "load is 25.0 on 32 cores, a seat (CARLA and 1 Autoware pilot) adds about 12" in why
    # nvidia-smi absent: the load alone decides
    assert dispatch.admission(seat, CFG, 1, **{**ok, "free_mb": lambda g: None}) == ""


def test_admission_counts_the_autoware_pilots():
    """A three-pilot testbench is not one ego's seat: each Autoware pilot adds its VRAM and
    load; a seat with none needs its CARLA's only."""
    seat = SimpleNamespace(gpu=0)
    ok = dict(free_mb=lambda g: 4500, load=lambda: (4.0, 0, 0), cores=lambda: 32)
    assert dispatch.seat_needs(CFG, 0) == (2000, 2)
    assert dispatch.seat_needs(CFG, 3) == (5000, 32)
    assert dispatch.admission(seat, CFG, 0, **ok) == ""
    assert dispatch.admission(seat, CFG, 2, **ok) == ""
    why = dispatch.admission(seat, CFG, 3, **ok)
    assert "has 4500 MiB free, a seat (CARLA and 3 Autoware pilots) needs 5000" in why
    why = dispatch.admission(seat, CFG, 3, **{**ok, "free_mb": lambda g: 9000})
    assert "the load is 4.0 on 32 cores, a seat (CARLA and 3 Autoware pilots) adds about 32" \
        in why
    assert "no Autoware pilot" in dispatch.admission(seat, CFG, 0, **{**ok, "free_mb":
                                                                       lambda g: 100})


def test_autoware_pilots_are_counted_from_the_testbench(tmp_path):
    (tmp_path / config.PROJECT_FILE).write_text(textwrap.dedent("""\
        simulator: carla
        pilots:
          ego: {kind: autoware}
          av2: {kind: autoware, lifetime: run}
          npc: {kind: carla_basic}
        """))
    assert dispatch.autoware_pilots(tmp_path) == 2
    assert dispatch.autoware_pilots(tmp_path / "none") == 0


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
        key = (o["spec"]["scenario"].rsplit("/", 1)[-1].removesuffix(".xosc")
               + f"_r{o['spec'].get('repetition') or 0}")
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
                       "scenario": {"file": o["spec"]["scenario"].rsplit("/", 1)[-1],
                                    "repetition": o["spec"].get("repetition") or 0,
                                    "seed": o["spec"].get("seed"),
                                    "parameters": o["spec"].get("parameters") or {}},
                       "metrics": plan.get("metrics", {}), "seat": seat}, f)
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
    """Every run's world ticks over budget past the limit: each run is tried twice, and the
    second try's verdict stands, flagged in summary.csv."""
    d, suite, out = dispatcher(tmp_path, fake, n=2, seat_idx=(0,),
                               plan={"0": {"metrics": {"ticks_over_budget": 23,
                                                       "frame_count": 200}}})
    assert d.run() == 0
    got = rows(suite)
    assert len(got) == 2 and all("11.5 % of ticks over budget (23 of 200; limit 5 %)"
                                 in r["overload"] for r in got)
    assert out.getvalue().count("run again (overloaded)") == 2


def test_the_load_during_a_run_is_recorded_not_judged(tmp_path, fake):
    """Autoware's start-up lifts the 1-minute load to hundreds with no harm to the run (243's
    reruns: two_av peak 616, 0.4 % ticks over budget): no rerun for it."""
    d, suite, out = dispatcher(tmp_path, fake, n=2, seat_idx=(0,), load=lambda: 616.0,
                               plan={"0": {"run_time": 1.2, "metrics": {
                                   "ticks_over_budget": 1, "frame_count": 250}}})
    assert d.run() == 0
    assert "overloaded" not in out.getvalue()
    rec = json.loads((suite / "s0_r0" / "dispatch.json").read_text())
    assert rec["load_max"] == 616.0 and rec["overloaded"] == ""
    assert rec["ticks_over_budget"] == 1 and rec["frame_count"] == 250


def test_cpu_pressure_during_a_run_is_overload(tmp_path, fake):
    """/proc/pressure/cpu's full stall share of the run past psi_full_limit."""
    clock = {"t": 0.0}
    psi = {"us": 0}

    def tick():
        clock["t"] += 0.5
        psi["us"] += 100_000   # 0.1 s stalled in every 0.5 s: 20 %
        return clock["t"]
    d, suite, out = dispatcher(tmp_path, fake, n=1, seat_idx=(0,), clock=tick,
                               psi=lambda: psi["us"])
    assert d.run() == 0
    assert "the CPU fully stalled 20.0 % of the run (limit 10 %)" in out.getvalue()


def test_psi_is_read_from_the_kernel_file(tmp_path):
    f = tmp_path / "cpu"
    f.write_text("some avg10=1.00 avg60=0.50 avg300=0.10 total=900\n"
                 "full avg10=0.00 avg60=0.00 avg300=0.00 total=1234\n")
    assert dispatch.psi_cpu_full_us(f) == 1234
    assert dispatch.psi_cpu_full_us(tmp_path / "none") is None


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


def test_seats_check_the_project_once_before_any_seat_starts(tmp_path):
    """A map or dataset not fetched refuses the dispatch once, naming the fix, rather than
    retiring every seat on it (133's live check, try 1)."""
    from scensim.findings import Finding
    found = [Finding("scensim.lock", "maps: Town01 is not fetched", "scensim assets fetch",
                     refused=True, host=True)]
    with pytest.raises(run.Refused) as e:
        run.check_project_once(tmp_path, check=lambda p: found)
    assert "no seat was started" in str(e.value)
    assert "(fix: scensim assets fetch)" in str(e.value)
    run.check_project_once(tmp_path, check=lambda p: [])   # nothing found: no refusal


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


# ------------------------------------------------------------------------------ rollouts

def test_rollouts_on_two_seats_are_grouped_per_scenario(tmp_path, fake):
    """088 step 3: a scenario's rollouts (seeds 100-105, two parameter values) are queue items
    like a suite's runs; rollouts.csv groups them: counts, the pass rate, the seeds, spreads."""
    roll = [run.RunSpec(scenario=Path("/x/cut.xosc"), parameters={"gap": g}, repetition=k,
                        seed=100 + k) for g in (6, 10) for k in range(3)]
    plan = {s: {"verdicts": {"cut_r1": "failed"}, "metrics": {"duration_s": 30.0 + int(s),
                                                              "ticks_over_budget": 0,
                                                              "frame_count": 600}}
            for s in ("0", "1")}
    suite = tmp_path / "suite"
    suite.mkdir()
    out = io.StringIO()
    d = dispatch.Dispatcher(roll, seats(0, 1), CFG, suite, None, out=out,
                            worker_argv=fake(plan), admit=lambda s: "", skips=lambda w: 0,
                            load=lambda: 4.0, autoware=1)
    assert d.run() == 1
    assert sorted(p.name for p in suite.iterdir() if p.is_symlink()) == sorted(
        s.key for s in roll)
    import csv
    with open(suite / "rollouts.csv") as f:
        got = {r["params"]: r for r in csv.DictReader(f)}
    assert set(got) == {"gap=6", "gap=10"}
    g6 = got["gap=6"]
    assert (g6["runs"], g6["passed"], g6["failed"], g6["error"]) == ("3", "2", "1", "0")
    assert g6["pass_rate"] == "0.6667" and g6["seeds"] == "100-102"
    assert 30.0 <= float(g6["duration_min"]) <= float(g6["duration_max"]) <= 31.0
    with open(suite / "rollout_metrics.csv") as f:
        m = [r for r in csv.DictReader(f) if r["params"] == "gap=10"]
    assert {r["metric"] for r in m} == {"duration_s", "ticks_over_budget", "frame_count"}
    assert all(r["n"] == "3" for r in m)
    rec = json.loads((suite / roll[0].key / "dispatch.json").read_text())
    assert rec["autoware_pilots"] == 1
