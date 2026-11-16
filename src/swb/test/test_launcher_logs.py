"""The log readers give the same answers from either launcher's layout (roadmap 079, step 4):
play_launch's ``<stamp>/node/<name>/{out,err}`` and ``ros2 launch``'s screen (console.log),
rcl's ``<exe>_<pid>_<epoch ms>.log`` files and ``<stamp-host-pid>/launch.log``."""
from swb import results, run, timeline

PL = "2026-11-16_21-22-33"                                # play_launch's run directory
R2 = "2026-11-16-21-24-41-163017-newslab-host-1543166"     # ros2 launch's

# (process as play_launch names its directory, as ros2 launch names it, rcl file, line)
LINES = {
    "bridge": [
        ("node/scenario_world_bridge/out", "scenario_world_bridge-1",
         "scenario_world_bridge_1482331_1794835477564",
         "2026-11-16T13:24:41.500000Z  WARN scenario_world_bridge: Frame 7 processing over 25 ms"),
        ("node/scenario_world_bridge/out", "scenario_world_bridge-1", None,
         "WARNING: attempting to unsubscribe from stream"),
    ],
    "pilot-ego": [
        ("node/autoware_ekf_localizer_node-1/err", "autoware_ekf_localizer_node-66",
         "autoware_ekf_localizer_node_1625905_1794835518271",
         "[WARN] [1001.5] [localization.pose_twist_fusion_filter.ekf_localizer]: not activated"),
        ("load_node/occupancy_grid_map_node/err", "component_container_mt-14",
         "component_container_mt_1623908_1794835517925",
         "[ERROR] [1002.0] [perception.occupancy_grid_map.occupancy_grid_map_node]: no tf"),
        ("node/acb_bridge/out", "acb_bridge-1", None,
         "2026-11-16T13:24:42.000000Z  WARN acb_bridge: Skipped 2 CARLA frame(s)"),
    ],
    "scenario": [
        ("node/scenario_test_runner/out", "scenario_test_runner.py-1",
         "python3_1630950_1794835522781",
         "[INFO] [1000.5] [simulation.scenario_test_runner]: Run a.xosc (1 of 1)"),
        ("node/openscenario_interpreter_node-1/err", "openscenario_interpreter_node-1",
         "openscenario_interpreter_node_1630952_1794835522352",
         "[WARN] [1001.0] [simulation.openscenario_interpreter]: Your machine is not powerful"),
        ("node/openscenario_interpreter_node-1/err", "openscenario_interpreter_node-1",
         "openscenario_interpreter_node_1630952_1794835522352",
         "[INFO] [1003.0] [simulation.openscenario_interpreter]: Passed"),
    ],
}


def _append(path, line):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a") as f:
        f.write(line + "\n")


def play_launch_tree(logs):
    for group, lines in LINES.items():
        _append(logs / group / "console.log",
                "2026-11-16T13:22:32.715289Z  INFO Step 1/3: Parsing launch file...")
        _append(logs / group / PL / "play_launch.log",
                "2026-11-16T13:22:32.715289Z  INFO Step 1/3: Parsing launch file...")
        for node_file, _, _, line in lines:
            _append(logs / group / PL / node_file, line)
            (logs / group / PL / node_file).parent.joinpath("metadata.json").write_text("{}")
    return logs


def ros2_tree(logs):
    for group, lines in LINES.items():
        d = logs / group
        _append(d / "console.log", f"[INFO] [launch]: All log files can be found below {d / R2}")
        _append(d / R2 / "launch.log", "1794835481.1637862 [INFO] [launch]: All log files ...")
        for _, proc, rcl, line in lines:
            _append(d / "console.log", f"[{proc}] {line}")
            _append(d / R2 / "launch.log", f"1794835519.0802557 [{proc}] {line}")
            if rcl:                        # rcl's file: the node's rcl lines alone
                _append(d / f"{rcl}.log", line)
        # the launch's own lines below WARN are not the timeline's
        _append(d / "console.log", "[INFO] [launch]: process[scenario_test_runner.py-1] was "
                                   "required: shutting down launched system")
    return logs


def _key(tl):
    return sorted((e["group"], e.get("process"), e["event"], e.get("level"), e.get("message"))
                  for e in tl)


def test_same_timeline_from_either_layout(tmp_path):
    pl = timeline.build([{"event": "run.start", "t_wall": 1000.0}],
                        slices_dir=play_launch_tree(tmp_path / "pl"))
    r2 = timeline.build([{"event": "run.start", "t_wall": 1000.0}],
                        slices_dir=ros2_tree(tmp_path / "r2"))
    assert _key(pl) == _key(r2)
    procs = {(e["group"], e.get("process")) for e in r2}
    assert procs == {("run", None), ("bridge", "scenario_world_bridge"),
                     ("pilot-ego", "autoware_ekf_localizer_node"),
                     ("pilot-ego", "occupancy_grid_map_node"), ("pilot-ego", "acb_bridge"),
                     ("scenario", "scenario_test_runner"),
                     ("scenario", "openscenario_interpreter_node")}
    # each line once: the 8 log lines (3 of them milestones' INFO), the run's start
    assert len(r2) == 9
    assert [e["event"] for e in r2 if e["event"].startswith("ssv2.")] == ["ssv2.run",
                                                                         "ssv2.passed"]


def test_rcl_files_read_without_a_console(tmp_path):
    """No screen beside them (a nested launch whose screen went elsewhere): rcl's files are
    the source, named without ``_<pid>_<epoch ms>``; launch.log gives its own lines' time."""
    d = tmp_path / "logs" / "bridge"
    _append(d / "python3_1482333_1794835442777.log", "[WARN] [1001.0] [relay]: agent gone")
    _append(d / R2 / "launch.log", "1794835673.78 [ERROR] [relay-2]: process has died "
                                   "[pid 1482333, exit code 1, cmd 'x']")
    tl = timeline.build(slices_dir=tmp_path / "logs")
    assert [(e["process"], e["level"], e["t_wall"]) for e in tl] == [
        ("python3", "WARN", 1001.0), ("relay", "ERROR", 1794835673.78)]


def test_timeline_keeps_deaths_at_shutdown(tmp_path):
    """A crash on exit stays visible: only the crash counter ignores the stop's deaths."""
    d = tmp_path / "logs" / "pilot-ego"
    _append(d / "console.log", "[INFO] [launch]: process[clock_guard-82] was required: "
                               "shutting down launched system")
    _append(d / "console.log", "[ERROR] [vehicle_agent-2]: process has died "
                               "[pid 1623906, exit code -11, cmd 'x']")
    tl = timeline.build(slices_dir=tmp_path / "logs")
    assert [(e["process"], e["level"]) for e in tl] == [("vehicle_agent", "ERROR")]
    assert tl[0]["message"].startswith("process has died")


def test_launcher_lines_once(tmp_path):
    """play_launch's own WARN is in its console and its play_launch.log: counted once."""
    logs = tmp_path / "logs"
    w = "2026-11-16T13:22:32.715289Z  WARN Press Ctrl-C once more for immediate kill"
    _append(logs / "pilot-ego" / "console.log", w)
    _append(logs / "pilot-ego" / PL / "play_launch.log", w)
    tl = timeline.build(slices_dir=logs)
    assert [(e["process"], e["message"]) for e in tl] == [
        ("console", "Press Ctrl-C once more for immediate kill")]


def test_process_name():
    for raw, name in [("scenario_world_bridge_1482331_1794835477564", "scenario_world_bridge"),
                      ("component_container_mt-14", "component_container_mt"),
                      ("scenario_test_runner.py-1", "scenario_test_runner"),
                      ("openscenario_interpreter_node-1", "openscenario_interpreter_node"),
                      ("acb_bridge", "acb_bridge")]:
        assert timeline.process_name(raw) == name


# --- the pilot crash counter ----------------------------------------------------------------

def _pl_crash(n, node="/command_follower"):
    return (f"2026-11-16T13:22:40.0Z  WARN [node:{node}] Exited with code 1 "
            f"(crash {n} since last healthy run)")


def _r2_died(pid, name="command_follower-1", code=1):
    return f"[ERROR] [{name}]: process has died [pid {pid}, exit code {code}, cmd 'x']"


def _watch(tmp_path, name, before, after):
    log = tmp_path / name
    log.write_text("".join(x + "\n" for x in before))
    w = run.CrashWatch(log)
    with open(log, "a") as f:
        f.write("".join(x + "\n" for x in after))
    return w.crashing()


def test_crash_watch_same_under_either_launcher(tmp_path):
    pl = _watch(tmp_path, "pl.log", [_pl_crash(9, "/other")], [_pl_crash(n) for n in range(1, 6)])
    r2 = _watch(tmp_path, "r2.log", [_r2_died(1, "other-3")] * 9,
                [_r2_died(100 + n) for n in range(5)])
    assert pl == ("/command_follower", 5) and r2 == ("command_follower", 5)
    # four is not yet the limit, under either
    assert _watch(tmp_path, "pl4.log", [], [_pl_crash(n) for n in range(1, 5)]) is None
    assert _watch(tmp_path, "r24.log", [], [_r2_died(100 + n) for n in range(4)]) is None
    # five processes of one executable dying once each are not one process crashing 5 times
    assert _watch(tmp_path, "r2n.log", [], [_r2_died(100 + n, f"component_container_mt-{n}")
                                            for n in range(5)]) is None


def test_crash_watch_ignores_deaths_at_shutdown(tmp_path):
    for stop in ("[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)",
                 "[INFO] [launch]: process[clock_guard-82] was required: shutting down "
                 "launched system"):
        after = [_r2_died(1)] + [stop] + [_r2_died(100 + n, code=-11) for n in range(6)]
        assert _watch(tmp_path, "stop.log", [], after) is None
    # not a shutdown: a node noting its own SIGINT, a nested launch's shutdown line
    for other in ("[x-3] [INFO] [1.0] [rclcpp]: signal_handler(SIGINT/SIGTERM)",
                  "[nested-2] [WARNING] [launch]: user interrupted with ctrl-c (SIGINT)"):
        after = [other] + [_r2_died(100 + n) for n in range(5)]
        assert _watch(tmp_path, "other.log", [], after) == ("command_follower", 5)
    # a nested launch's own deaths are its own, as play_launch sees only the nested launch
    after = [f"[nested-2] {_r2_died(100 + n)}" for n in range(5)]
    assert _watch(tmp_path, "nested.log", [], after) is None


# --- SSv2's hint ----------------------------------------------------------------------------

def test_ssv2_hint_same_under_either_launcher(tmp_path):
    runner = ["Traceback (most recent call last):", "  File \"x.py\", line 1, in <module>",
              "ModuleNotFoundError: No module named 'xmlschema'"]
    interp = ["[ERROR] [1001.0] [simulation.openscenario_interpreter]: something broke"]
    pl = tmp_path / "pl"
    for node, lines in (("scenario_test_runner", runner),
                        ("openscenario_interpreter_node-1", interp)):
        for x in lines:
            _append(pl / PL / "node" / node / "err", x)
    r2 = tmp_path / "r2"
    _append(r2 / "console.log", "[INFO] [launch]: All log files can be found below x")
    _append(r2 / "console.log", "[rviz2-6] ValueError: not SSv2's")
    for x in interp:
        _append(r2 / "console.log", f"[openscenario_interpreter_node-1] {x}")
        _append(r2 / "openscenario_interpreter_node_1630952_1794835522352.log", x)
    for x in runner:
        _append(r2 / "console.log", f"[scenario_test_runner.py-1] {x}")
    want = "ModuleNotFoundError: No module named 'xmlschema'"
    assert results.ssv2_hint(pl) == results.ssv2_hint(r2) == want

    # its timeout line outranks the rest, under either
    t = "[ERROR] [1002.0] [simulation.scenario_test_runner]: The simulation has timed out (400 s)"
    _append(pl / PL / "node" / "scenario_test_runner" / "err", t)
    _append(r2 / "console.log", f"[scenario_test_runner.py-1] {t}")
    assert results.ssv2_hint(pl) == results.ssv2_hint(r2) == "The simulation has timed out (400 s)"
