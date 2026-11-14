from swb import timeline


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
    assert passed["group"] == "scenario" and passed["process"] == "openscenario_interpreter_node-1"
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
