from scensim import timeline


def test_parse_ros_line():
    lv, t, node, msg = timeline.parse_line(
        "\x1b[33m[WARN] [1792452164.527513971] [traffic_simulator]: use_custom_centerline\x1b[0m\n")
    assert (lv, node, msg) == ("WARN", "traffic_simulator", "use_custom_centerline")
    assert abs(t - 1792452164.527514) < 1e-5
    lv, _, node, _ = timeline.parse_line("[component_container-1] [ERROR] [1.5] [ndt]: boom")
    assert (lv, node) == ("ERROR", "ndt")


def test_parse_play_launch_and_bare():
    lv, t, tgt, msg = timeline.parse_line(
        "2026-10-19T23:22:41.612593Z  WARN play_launch::web: Web UI is exposed")
    assert (lv, tgt, msg) == ("WARN", "play_launch::web", "Web UI is exposed")
    assert t == timeline.parse_iso("2026-10-19T23:22:41.612593+00:00")
    lv, t, _, msg = timeline.parse_line("2026-10-19 23:22:41,612 WARNING root: x")
    assert lv == "WARN" and t is not None
    assert timeline.parse_line("ERROR: failed to destroy actor 176") == (
        "ERROR", None, None, "failed to destroy actor 176")
    assert timeline.parse_line("just some output") is None
    assert timeline.parse_line("1792452161.641846 [9] openscenar: config: x") is None


def test_parse_iso_nanoseconds():
    assert timeline.parse_iso("1970-01-01T00:00:01.123456789Z") == 1.123456
    assert timeline.parse_iso("nope") is None


def test_build_orders_and_infers(tmp_path):
    logs = tmp_path / "logs"
    (logs / "bridge").mkdir(parents=True)
    (logs / "bridge" / "scenario_world_bridge.log").write_text(
        "[INFO] [105.0] [bridge]: fine\n"
        "[WARN] [103.0] [bridge]: slow tick\n"
        "WARNING: attempting to unsubscribe\n")
    (logs / "relay").mkdir()
    (logs / "relay" / "scenario_agent_relay.log").write_text(
        "[ERROR] [104.0] [relay]: agent gone\n")
    scen = tmp_path / "scen"
    (scen / "node" / "openscenario_interpreter_node-1").mkdir(parents=True)
    (scen / "node" / "openscenario_interpreter_node-1" / "err").write_text(
        "[INFO] [106.0] [simulation.openscenario_interpreter]: \x1b[32mPassed\x1b[0m\n")
    (scen / "node" / "openscenario_interpreter_node-1" / "metadata.json").write_text(
        '{"x": "[ERROR] [1.0] [n]: not a log"}\n')
    (scen / "node" / "scenario_test_runner").mkdir()
    (scen / "node" / "scenario_test_runner" / "out").write_text(
        "[INFO] [101.0] [simulation.scenario_test_runner]: Run a.xosc (1 of 1)\n")
    tl = timeline.build(
        [{"event": "run.start", "t_wall": 100.0}, {"event": "run.end", "t_wall": 107.0},
         {"type": "collision.vehicle", "t_wall": "1970-01-01T00:01:45.500Z", "t_sim": 3.0,
          "entity": "ego"}],
        slices_dir=logs, scenario_dir=scen)
    ev = [(e["t_wall"], e["event"]) for e in tl]
    assert ev == [(100.0, "run.start"), (101.0, "ssv2.run"), (103.0, "log.warn"),
                  (103.0, "log.warn"), (104.0, "log.error"), (105.5, "collision.vehicle"),
                  (106.0, "ssv2.passed"), (107.0, "run.end")]
    bare = [e for e in tl if e.get("t_inferred")]
    assert len(bare) == 1 and bare[0]["message"] == "attempting to unsubscribe"
    assert bare[0]["group"] == "bridge" and bare[0]["process"] == "scenario_world_bridge"
    passed = [e for e in tl if e["event"] == "ssv2.passed"][0]
    assert passed["group"] == "scenario" and passed["process"] == "openscenario_interpreter_node"
    first = timeline.first_problem(tl)
    assert first["message"] == "slow tick" and first["group"] == "bridge"
    assert timeline.first_problem(tl, "ERROR")["group"] == "relay"

    p = timeline.write(tl, tmp_path / "timeline.jsonl")
    assert timeline.read(p) == tl


def test_bare_line_before_any_time_takes_default(tmp_path):
    d = tmp_path / "logs" / "simulator"
    d.mkdir(parents=True)
    (d / "carla.log").write_text("ERROR: x\n")
    tl = timeline.build([], slices_dir=tmp_path / "logs", default_t=50.0)
    assert tl[0]["t_wall"] == 50.0 and tl[0]["t_inferred"]


def test_stack_error_is_a_problem():
    tl = timeline.build([{"type": "stack.error", "t_wall": 1.0, "message": "world server died"}])
    assert timeline.first_problem(tl)["message"] == "world server died"
    assert timeline.first_problem([]) is None


def test_a_time_without_a_zone_is_local(monkeypatch):
    """G69: Python logging's asctime is local time; read as UTC it was 28800 s late at +8."""
    import time
    monkeypatch.setenv("TZ", "Asia/Taipei")
    time.tzset()
    try:
        utc = timeline.parse_iso("2026-11-16T09:43:07.091834Z")
        _, t, name, _ = timeline.parse_line(
            "2026-11-16 17:43:07,091 WARNING scensim_world_server: step 2 has run 180.1 s")
        assert name == "scensim_world_server"
        assert abs(t - (utc - 0.000834)) < 1e-6  # the same instant, not 8 h later
        assert timeline.parse_iso("2026-11-16T17:43:07.091834+08:00") == utc  # a zone stays
    finally:
        monkeypatch.delenv("TZ")
        time.tzset()


def test_why_prefers_swbs_own_refusal_and_skips_capability_notes():
    """G67: the bridge's "Cannot spawn ..." outranks SSv2's generic SemanticError; play_launch's
    capability notes are never first."""
    def e(t, group, process, level, message):
        return {"t_wall": t, "group": group, "process": process, "level": level,
                "event": f"log.{level.lower()}", "message": message}
    cap = e(1.0, "scenario", "play_launch", "WARN",
            "/x/bin/play_launch_io_helper lacks cap_sys_ptrace: I/O monitoring cannot read "
            "privileged processes.")
    setcap = e(1.1, "scenario", "console", "WARN", "run `play_launch setcap` to grant it")
    semantic = e(5.0, "scenario", "openscenario_interpreter_node-1", "ERROR",
                 "SemanticError: Failed to add entity 'ego'")
    spawn = e(5.2, "bridge", "scenario_world_bridge", "WARN",
              "Cannot spawn 'ego' (autoware): no agent registered for it")
    tl = [cap, setcap, semantic, spawn]
    assert timeline.why(tl) is spawn
    assert timeline.first_problem(tl) is semantic  # the capability notes are skipped
    assert timeline.first_problem(tl, "ERROR") is semantic
    # no refusal from scensim's processes: the first problem, as before
    assert timeline.why([cap, semantic]) is semantic
    # a world server's failure counts too; another group's "failed" does not outrank
    ws = e(6.0, "simulator", "scensim_isaac_world", "ERROR", "load_world failed: no USD stage")
    pilot = e(4.0, "pilot-ego", "planning", "WARN", "planning failed to find a path")
    assert timeline.why([cap, pilot, semantic, ws]) is ws
    assert timeline.why([cap, setcap]) is None
    # G70: the world server's watchdog names the request it ended
    dog = e(7.0, "simulator", "scensim_isaac_world", "ERROR",
            "step 1 (3 x 0.0166667 s at sim time 0.000 s) has run 180.0 s, past its deadline "
            "of 180 s: ending world server 4242 for a fresh one")
    bridge = e(6.5, "bridge", "scenario_world_bridge", "ERROR",
               "update_frame{frame=1}: the isaac world server did not answer Step within 10 s")
    # the errored run sees the bridge give up (10 s); the watchdog's line (180 s) comes after
    # it, in the stack's log or the next run's slices
    assert timeline.why([cap, semantic, bridge, dog]) is bridge
    assert timeline.why([cap, semantic, dog]) is dog


# issue 064: 088 on swb 133, a pilot counted ready with 126 of 127 composables loaded
_PL = "2026-11-18T20:00:{s}Z  {level} {msg}"
_FAILED = ("container:/pointcloud_container: control channel LOAD_FAILED for "
           "'composable:/perception/occupancy_grid_map/occupancy_grid_map_node': component_node "
           "failed: Exception loading 'autoware::occupancy_grid_map::PointcloudBasedOccupancy"
           "GridMapNode': cudaErrorMemoryAllocation (2)")
_SUMMARY = ('STARTUP_SUMMARY {"composable_loaded":126,"composable_shortfall":[{"container":'
            '"pointcloud_container","declared":8,"loaded":7,"missing":["/perception/'
            'occupancy_grid_map/occupancy_grid_map_node"]}],"composable_total":127,"ok":false}')
_CLEAN = 'STARTUP_SUMMARY {"composable_loaded":127,"composable_shortfall":[],"ok":true}'


def _console(tmp_path, *lines):
    p = tmp_path / "console.log"
    p.write_text("".join(line + "\n" for line in lines))
    return p


def test_a_composable_play_launch_did_not_load_is_named_with_why(tmp_path):
    start = _PL.format(s="18", level="INFO", msg="Step 1/3: Parsing launch file...")
    p = _console(tmp_path, start, _PL.format(s="25", level="WARN", msg=_FAILED))
    got = timeline.missing_composables(p)          # before the summary: the LOAD_FAILED line
    assert got.startswith("/perception/occupancy_grid_map/occupancy_grid_map_node (")
    assert "cudaErrorMemoryAllocation" in got
    p = _console(tmp_path, start, _PL.format(s="25", level="WARN", msg=_FAILED),
                 _PL.format(s="27", level="INFO", msg=_SUMMARY))
    assert "cudaErrorMemoryAllocation" in timeline.missing_composables(p)
    # a retry that loaded it: the summary is the last word
    p = _console(tmp_path, start, _PL.format(s="25", level="WARN", msg=_FAILED),
                 _PL.format(s="27", level="INFO", msg=_CLEAN))
    assert timeline.missing_composables(p) == ""


def test_only_the_current_launch_of_a_respawned_pilot_counts(tmp_path):
    start = _PL.format(s="18", level="INFO", msg="Step 1/3: Parsing launch file...")
    p = _console(tmp_path, start, _PL.format(s="25", level="WARN", msg=_FAILED),
                 _PL.format(s="27", level="INFO", msg=_SUMMARY), start,
                 _PL.format(s="40", level="INFO", msg=_CLEAN))
    assert timeline.missing_composables(p) == ""
    assert timeline.missing_composables(tmp_path / "none.log") == ""


def test_a_composable_ros2_launch_did_not_load_is_named(tmp_path):
    p = _console(tmp_path, "[INFO] [launch]: All log files can be found below /x",
                 "[ERROR] [launch_ros.actions.load_composable_nodes]: Failed to load node "
                 "'occupancy_grid_map_node' of type 'autoware::occupancy_grid_map::Pointcloud"
                 "BasedOccupancyGridMapNode' in container '/pointcloud_container': "
                 "cudaErrorMemoryAllocation")
    assert timeline.missing_composables(p) == (
        "/pointcloud_container/occupancy_grid_map_node (cudaErrorMemoryAllocation)")


# Issue 088: a container's reply to the launch's load_node lost; launch_ros waits for it for ever
# (lines as the 241 console had them; the container's lines reach the console late, after a reply
# to a later load)
_BP = "planning.scenario_planning.lane_driving.behavior_planning.behavior_planning_container"
_BP_PATH = "/" + _BP.replace(".", "/")


def _stalled_console(tmp_path, t0, lost=True, answered=1):
    lines = ["[INFO] [launch]: All log files can be found below /x",
             "[component_container_mt-2] [INFO] [%.3f] [pointcloud_container]: Instantiate class: "
             "rclcpp_components::NodeFactoryTemplate<autoware::pointcloud_preprocessor::"
             "CropBoxFilterComponent>" % (t0 - 5),
             "[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/crop_box_filter' in "
             "container 'pointcloud_container'"]
    lines += ["[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node "
              f"'{_BP_PATH.rsplit('/', 1)[0]}/behavior_path_planner' in container '{_BP_PATH}'"] * answered
    lines.append(f"[component_container_mt-53] [INFO] [{t0:.3f}] [{_BP}]: Instantiate class: "
                 "rclcpp_components::NodeFactoryTemplate<autoware::glog_component::GlogComponent>")
    if lost:
        lines.append(f"[component_container_mt-53] [WARN] [{t0 + 0.2:.3f}] [{_BP}.rclcpp]: failed to "
                     f"send response to {_BP_PATH}/_container/load_node (timeout): error not set")
    lines.append(f"[component_container_mt-53] [INFO] [{t0 + 0.3:.3f}] [{_BP}]: Instantiate class: "
                 "rclcpp_components::NodeFactoryTemplate<autoware::behavior_path_planner::"
                 "BehaviorPathPlannerNode>")
    return _console(tmp_path, *lines)


def test_a_lost_load_node_reply_names_its_container(tmp_path):
    t0 = 1795190012.7
    p = _stalled_console(tmp_path, t0)
    assert timeline.stalled_loads(p, now=t0 + 10) == ""      # within the grace
    got = timeline.stalled_loads(p, now=t0 + 60)
    assert got.startswith(f"{_BP_PATH} (its reply to the launch's load_node was lost 60 s ago; "
                          "1 of 2 loads answered; loaded there: BehaviorPathPlannerNode, "
                          "GlogComponent)")
    assert "pointcloud_container" not in got                  # 'pointcloud_container' is '/pointcloud_container'


def test_a_load_unanswered_too_long_is_stalled_without_a_lost_reply(tmp_path):
    t0 = 1795190012.7
    p = _stalled_console(tmp_path, t0, lost=False)
    assert timeline.stalled_loads(p, now=t0 + 590) == ""     # 065: a constructor may build for minutes
    assert "a load unanswered for at least 600 s" in timeline.stalled_loads(p, now=t0 + 600.3)


def test_every_load_answered_is_no_stall(tmp_path):
    t0 = 1795190012.7
    p = _stalled_console(tmp_path, t0, lost=True, answered=2)
    assert timeline.stalled_loads(p, now=t0 + 3600) == ""
    assert timeline.stalled_loads(tmp_path / "none.log") == ""


def test_a_stall_before_the_pilots_restart_is_not_its(tmp_path):
    """A restarted pilot's console keeps the old launch's lines until its new launch writes."""
    t0 = 1795190012.7
    p = _stalled_console(tmp_path, t0)
    assert timeline.stalled_loads(p, now=t0 + 60, since=t0 + 30) == ""
    assert timeline.stalled_loads(p, now=t0 + 60, since=t0 - 1) != ""


def test_a_stall_of_an_earlier_launch_instance_is_not_this_ones(tmp_path):
    t0 = 1795190012.7
    first = _stalled_console(tmp_path, t0).read_text()
    p = _console(tmp_path, *first.splitlines(), "[INFO] [launch]: All log files can be found below /y")
    assert timeline.stalled_loads(p, now=t0 + 60) == ""


def test_launch_ros_lines_are_not_read_under_play_launch(tmp_path):
    """play_launch's own loader logs no 'Loaded node' lines: launch_ros's rule is not applied."""
    t0 = 1795190012.7
    body = _stalled_console(tmp_path, t0).read_text().splitlines()[1:]
    p = _console(tmp_path, "INFO Step 1/3: Parsing launch file...", *body)
    assert timeline.stalled_loads(p, now=t0 + 3600) == ""


# play_launch 0.35 with its container frozen (139, 2026-11-21): LoadNode replies lost, the loads
# pending for 12 minutes, no LOAD_FAILED and no STARTUP_SUMMARY
def _pl_line(t, msg):
    return timeline._dt.datetime.fromtimestamp(t, timeline._dt.timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%S.%fZ") + "  " + msg


def _pl_console(tmp_path, t0, until, lost=True, done_at=None, loaded_at=None):
    lines = ["INFO Step 1/3: Parsing launch file...",
             _pl_line(t0, "INFO Load timings: first_call=30s retry=60s attempts=3 busy_budget=600s")]
    t = t0 + 10
    while t <= until:
        n = 3 if loaded_at is not None and t >= loaded_at else 0
        lines.append(_pl_line(t, f"INFO Startup: nodes 0/0 running, containers 1/1 running, "
                                 f"composable {n}/3 loaded ({3 - n} pending)"))
        if lost and abs(t - (t0 + 30)) < 5:
            for c in ("talker", "listener", "server"):
                lines.append(_pl_line(t, f"WARN container:/planning/stall_container: LoadNode service "
                                         f"call timed out after 30s for composable:/{c} (attempt 1/3)"))
                lines.append(_pl_line(t, f"WARN container:/planning/stall_container: LoadNode response "
                                         f"lost for composable:/{c} \u2014 deferring to ComponentEvent"))
        if done_at is not None and t >= done_at:
            lines.append(_pl_line(t, "INFO Startup complete: all nodes ready (nodes 0/0, containers "
                                     "1/1, composable 3/3)"))
            break
        t += 10
    return _console(tmp_path, *lines)


def test_play_launch_lost_load_replies_name_the_container(tmp_path):
    t0 = 1795203785.0
    p = _pl_console(tmp_path, t0, until=t0 + 720)
    assert timeline.stalled_loads(p, now=t0 + 30 + 119) == ""   # its own retry may still recover
    got = timeline.stalled_loads(p, now=t0 + 30 + 125)
    assert got.startswith("/planning/stall_container (its LoadNode reply for /listener, /server, "
                          "/talker was lost 125 s ago; 0 of 3 composables loaded, 3 pending")


def test_play_launch_loads_pending_too_long_without_a_lost_reply(tmp_path):
    t0 = 1795203785.0
    p = _pl_console(tmp_path, t0, until=t0 + 720, lost=False)
    assert timeline.stalled_loads(p, now=t0 + 600) == ""
    assert "3 pending, none loaded for 610 s" in timeline.stalled_loads(p, now=t0 + 620)


def test_play_launch_recovered_or_complete_is_no_stall(tmp_path):
    """A lost reply play_launch recovers (the T3 start on 139 resent one, then completed)."""
    t0 = 1795203785.0
    p = _pl_console(tmp_path, t0, until=t0 + 720, loaded_at=t0 + 60, done_at=t0 + 60)
    assert timeline.stalled_loads(p, now=t0 + 3600) == ""
    p = _pl_console(tmp_path, t0, until=t0 + 720, loaded_at=t0 + 60)   # all loaded, no "complete" line
    assert timeline.stalled_loads(p, now=t0 + 3600) == ""


def test_play_launch_stall_before_the_pilots_restart_is_not_its(tmp_path):
    t0 = 1795203785.0
    p = _pl_console(tmp_path, t0, until=t0 + 720)
    assert timeline.stalled_loads(p, now=t0 + 800, since=t0 + 730) == ""
