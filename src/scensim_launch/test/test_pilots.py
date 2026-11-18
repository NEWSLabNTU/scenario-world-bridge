"""The pilot launches (roadmap 076 step 5): one argument contract for every kind, and
stack.launch.py's `pilots:` (docs/design/projects-and-pilots.md, "Pilots")."""
import importlib.util
import json
import os
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH = os.path.join(HERE, "..", "launch")
PILOTS = os.path.join(LAUNCH, "pilots")

CONTRACT = ("entity", "relay", "map_path", "vehicle", "seed", "run_dir", "sim", "use_sim_time",
            "carla_host", "carla_port", "release_notify_endpoint", "release_ack_endpoint")


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


stack = load(os.path.join(LAUNCH, "stack.launch.py"), "stack_launch")
carla_basic = load(os.path.join(PILOTS, "carla_basic.launch.py"), "pilot_carla_basic")
launch_pilot = load(os.path.join(PILOTS, "launch.launch.py"), "pilot_launch")
command = load(os.path.join(PILOTS, "command.launch.py"), "pilot_command")
simple = load(os.path.join(PILOTS, "simple.launch.py"), "pilot_simple")


def xml_args(name):
    root = ET.parse(os.path.join(PILOTS, name)).getroot()
    assert root.tag == "launch"
    return {a.get("name"): a for a in root.findall("arg")}, root


# --- the contract -------------------------------------------------------------------------
@pytest.mark.parametrize("name", ["autoware.launch.xml", "vehicle_side_carla.launch.xml"])
def test_xml_pilots_declare_the_contract(name):
    args, _ = xml_args(name)
    missing = [a for a in CONTRACT if a not in args]
    assert not missing, f"{name} lacks {missing}"


@pytest.mark.parametrize("module", [carla_basic, launch_pilot, command, simple])
def test_python_pilots_declare_the_contract(module):
    assert set(CONTRACT) <= set(module.CONTRACT)


def test_every_kind_has_its_file():
    for kind, name in stack.PILOT_KINDS.items():
        if name is not None:
            assert os.path.isfile(os.path.join(PILOTS, name)), kind
    assert stack.PILOT_KINDS["none"] is None


def test_vehicle_side_carla_is_acb_bridge_only():
    _, root = xml_args("vehicle_side_carla.launch.xml")
    includes = [i.get("file") for i in root.iter("include")]
    assert includes == ["$(find-pkg-share acb_bridge)/launch/acb_bridge.launch.xml"]
    passed = {a.get("name") for a in root.find("include").findall("arg")}
    assert {"carla_physics", "carla_physics_mode", "carla_address", "carla_port",
            "release_notify_endpoint", "release_ack_endpoint", "vehicle_name"} <= passed


def test_autoware_composes_vehicle_side_agent_and_stock_or_profile():
    args, root = xml_args("autoware.launch.xml")
    assert "autoware_launch" in args
    assert args["autoware_profile"].get("default") == "stock"
    includes = [i.get("file") for i in root.iter("include")]
    # the driver last: it sets Autoware's `vehicle` switch (launch arguments are global),
    # which must not reach the vehicle side and the agent, where `vehicle` is the car
    assert includes == [
        "$(find-pkg-share scensim_launch)/launch/pilots/vehicle_side.launch.xml",
        "$(find-pkg-share scensim_launch)/launch/vehicle_agent.launch.xml",
        "$(var autoware_launch)",
        "$(find-pkg-share autoware_sim_profile)/launch/autoware.launch.xml"]
    stock, profile = root.findall("group")[-2:]
    assert "== 'stock'" in stock.get("if") and "== 'scensim'" in profile.get("if")
    passed = {a.get("name"): a.get("value") for a in profile.find("include").findall("arg")}
    assert passed["vehicle"] == "true" and passed["vehicle_model"] == "$(var vehicle)"
    # stock: only what a simulator needs, the rest is the user's Autoware's own
    passed = {a.get("name"): a.get("value") for a in stock.find("include").findall("arg")}
    assert passed == {
        "map_path": "$(var map_path)", "vehicle_model": "$(var vehicle)",
        "sensor_model": "$(var sensor_model)", "use_sim_time": "$(var use_sim_time)",
        "vehicle": "true", "launch_vehicle_interface": "false",
        "launch_sensing_driver": "false", "system_run_mode": "logging_simulation",
        "traffic_light_recognition/use_high_accuracy_detection": "false",
        "rviz": "$(var rviz)"}
    assert args["rviz"].get("default") == "false"


def test_vehicle_side_dispatches_on_sim():
    root = ET.parse(os.path.join(PILOTS, "vehicle_side.launch.xml")).getroot()
    groups = root.findall("group")
    assert len(groups) == 1 and "'carla'" in groups[0].get("if")
    assert groups[0].find("include").get("file").endswith("pilots/vehicle_side_carla.launch.xml")


def test_xml_frontend_parses():
    """launch_xml reads each file (no substitution is evaluated)."""
    frontend = pytest.importorskip("launch.frontend")
    pytest.importorskip("launch_xml")
    for name in ("autoware.launch.xml", "vehicle_side_carla.launch.xml", "vehicle_side.launch.xml"):
        entity, parser = frontend.Parser.load(os.path.join(PILOTS, name))
        assert parser.parse_description(entity).entities


@pytest.mark.parametrize("relay,want", [("tcp://localhost:5560", "false"), ("none", "true")])
def test_a_managed_autoware_ego_starts_no_gnss_initializer(relay, want):
    """Issue 046: with a relay the agent initializes at the scenario's pose, so the profile's
    gnss_enabled is false (no automatic_pose_initializer to queue a second align); local mode
    keeps Autoware's default."""
    frontend = pytest.importorskip("launch.frontend")
    pytest.importorskip("launch_xml")
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    args, root = xml_args("autoware.launch.xml")
    profile = root.findall(".//include")[-1]
    passed = {a.get("name"): a.get("value") for a in profile.findall("arg")}
    assert passed["gnss_enabled"] == "$(var gnss_enabled)"
    entity, parser = frontend.Parser.load(os.path.join(PILOTS, "autoware.launch.xml"))
    [declared] = [e for e in parser.parse_description(entity).entities
                  if isinstance(e, DeclareLaunchArgument) and e.name == "gnss_enabled"]
    context = LaunchContext()
    context.launch_configurations["relay"] = relay
    assert perform_substitutions(context, declared.default_value) == want


@pytest.mark.parametrize("relay,want", [("tcp://localhost:5560", "false"), ("none", "true")])
def test_a_managed_ego_on_carla_is_not_reseeded_by_acb(relay, want):
    """Issue 049: acb_bridge's /initialpose on attach became a second localization initialize
    beside the agent's (two NDT aligns per reset); with a relay the agent initializes, so the
    vehicle side does not seed. Without one acb seeds, as stand-alone."""
    frontend = pytest.importorskip("launch.frontend")
    pytest.importorskip("launch_xml")
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    args, root = xml_args("vehicle_side_carla.launch.xml")
    passed = {a.get("name"): a.get("value") for a in root.find("include").findall("arg")}
    assert passed["seed_localization_on_attach"] == "$(var seed_localization_on_attach)"
    entity, parser = frontend.Parser.load(os.path.join(PILOTS, "vehicle_side_carla.launch.xml"))
    [declared] = [e for e in parser.parse_description(entity).entities
                  if isinstance(e, DeclareLaunchArgument)
                  and e.name == "seed_localization_on_attach"]
    context = LaunchContext()
    context.launch_configurations["relay"] = relay
    assert perform_substitutions(context, declared.default_value) == want


# --- the kinds' own logic -----------------------------------------------------------------
def test_command_argv_appends_the_contract():
    values = {"run": "python3 -m my.policy --policy 'push hard'", "relay": "tcp://h:5560",
              "entity": "cut_in", "seed": "1002", "run_dir": "/r/1"}
    assert command.argv(values) == [
        "python3", "-m", "my.policy", "--policy", "push hard", "--relay", "tcp://h:5560",
        "--entity", "cut_in", "--seed", "1002", "--run-dir", "/r/1"]
    assert "--run-dir" not in command.argv({**values, "run_dir": ""})
    with pytest.raises(RuntimeError, match="run:="):
        command.argv({**values, "run": ""})


def test_launch_driver_gets_contract_then_launch_args():
    values = {k: f"<{k}>" for k in launch_pilot.CONTRACT}
    values["launch_args"] = "map_path:=/mine x:=1"
    args = launch_pilot.driver_args(values)
    assert args["entity"] == "<entity>" and args["seed"] == "<seed>"
    assert args["map_path"] == "/mine" and args["x"] == "1"
    assert "carla_host" not in args  # the vehicle side's, not the driver's


def test_launch_resolves_a_path(tmp_path):
    f = tmp_path / "mine.launch.xml"
    f.write_text("<launch/>")
    assert launch_pilot.resolve_launch(str(f)) == str(f)
    with pytest.raises(RuntimeError, match="no file"):
        launch_pilot.resolve_launch(str(tmp_path / "absent.launch.xml"))
    with pytest.raises(RuntimeError, match="package"):
        launch_pilot.resolve_launch("a b c")


def test_carla_basic_config(tmp_path):
    user = tmp_path / "c.json"
    user.write_text(json.dumps({"target_speed_kmh": 20, "carla_port": 1}))
    values = {"config": str(user), "entity": "ego", "carla_host": "h", "carla_port": "2002",
              "role_name": "", "carla_pythonapi": "/carla/PythonAPI/carla"}
    config = carla_basic.agent_config(values)
    assert config == {"target_speed_kmh": 20, "carla_port": 2002, "carla_host": "h",
                      "role_name": "hero", "carla_pythonapi": "/carla/PythonAPI/carla"}
    assert carla_basic.agent_config({**values, "entity": "npc_1"})["role_name"] == "npc_1"


def _context(values):
    from launch import LaunchContext
    context = LaunchContext()
    for k, v in values.items():
        context.launch_configurations[k] = v
    return context


def test_carla_basic_evaluates(tmp_path):
    defaults = {k: d for k, (d, _) in {**carla_basic.CONTRACT, **carla_basic.EXTRA}.items()}
    actions = carla_basic._pilot(_context({**defaults, "entity": "npc_1", "run_dir": str(tmp_path)}))
    (process,) = actions
    written = tmp_path / "carla_basic-npc_1.json"
    assert json.loads(written.read_text())["role_name"] == "npc_1"
    cmd = [c.perform(_context({})) if hasattr(c, "perform") else c
           for part in process.cmd for c in part]
    assert "".join(cmd).replace(" ", "").startswith("ros2runscensim_agentcarla_basic")
    with pytest.raises(RuntimeError, match="CARLA's only"):
        carla_basic._pilot(_context({**defaults, "sim": "isaac"}))


def test_command_evaluates_without_vehicle_side():
    defaults = {k: d for k, (d, _) in {**command.CONTRACT, **command.EXTRA}.items()}
    (process,) = command._pilot(_context({**defaults, "run": "true"}))
    assert [t.perform(_context({})) for t in process.name] == ["command_ego"]


def test_simple_config(tmp_path):
    user = tmp_path / "s.yaml"
    user.write_text("max_speed_kmh: 20\nlateral_accel: 1.5\nmap_path: /elsewhere\n")
    values = {"config": str(user), "map_path": "/maps/Town01", "sim": "isaac",
              "vehicle": "tesla_model3", "use_sim_time": "true", "odom_topic": "",
              "max_speed_kmh": ""}
    assert simple.agent_config(values) == {
        "max_speed_kmh": 20, "lateral_accel": 1.5, "map_path": "/maps/Town01", "sim": "isaac",
        "vehicle": "tesla_model3", "use_sim_time": True}
    config = simple.agent_config({**values, "odom_topic": "/gt", "max_speed_kmh": "12"})
    assert config["odom_topic"] == "/gt" and config["max_speed_kmh"] == 12.0


def test_simple_evaluates(tmp_path):
    defaults = {k: d for k, (d, _) in {**simple.CONTRACT, **simple.EXTRA}.items()}
    values = {**defaults, "map_path": "/maps/Town01", "run_dir": str(tmp_path)}
    side, rsp, process = simple._pilot(_context(values))
    assert type(side).__name__ == "IncludeLaunchDescription"
    assert type(rsp).__name__ == "Node"
    assert json.loads((tmp_path / "simple_pilot-ego.json").read_text())["map_path"] == \
        "/maps/Town01"
    cmd = [c.perform(_context({})) if hasattr(c, "perform") else c
           for part in process.cmd for c in part]
    assert "".join(cmd).replace(" ", "").startswith("ros2runscensim_agentsimple_pilot")
    assert os.path.isfile(simple.XACRO)
    assert len(simple._pilot(_context({**values, "publish_description": "false"}))) == 2
    with pytest.raises(RuntimeError, match="map_path"):
        simple._pilot(_context({**values, "map_path": ""}))
    with pytest.raises(RuntimeError, match="relay"):
        simple._pilot(_context({**values, "relay": "none"}))


def test_simple_has_a_vehicle_side_and_one_on_isaac():
    assert stack.has_vehicle_side("simple", {})
    (p,) = stack.parse_pilots("[{entity: ego, kind: simple}]", "isaac", "1", "9")
    assert p["domain"] == "1"
    with pytest.raises(RuntimeError, match="one pilot has a vehicle side"):
        stack.parse_pilots("[{entity: a, kind: simple}, {entity: b, kind: autoware}]",
                           "isaac", "1", "9")


# --- stack.launch.py's pilots: -----------------------------------------------------------
def test_no_pilots():
    assert stack.parse_pilots("", "carla", "1", "9") == []
    assert stack.parse_pilots("  ", "carla", "1", "9") == []


def test_list_with_auto_domains():
    pilots = stack.parse_pilots(
        "[{entity: ego, kind: autoware, vehicle: tesla_model3},"
        " {entity: bg_av_1, kind: autoware},"
        " {entity: npc, kind: carla_basic}]", "carla", "1", "9")
    assert [(p["entity"], p["domain"]) for p in pilots] == [("ego", "1"), ("bg_av_1", "2"),
                                                            ("npc", "3")]
    assert pilots[0]["args"] == {"vehicle": "tesla_model3"}


def test_json_and_mapping_forms():
    a = stack.parse_pilots(json.dumps([{"entity": "ego", "kind": "autoware", "domain": 4}]),
                           "carla", "1", "9")
    b = stack.parse_pilots("ego: {kind: autoware, domain: 4}", "carla", "1", "9")
    assert a == b == [{"entity": "ego", "kind": "autoware", "args": {}, "domain": "4"}]


def test_auto_domain_skips_scenario_and_explicit():
    pilots = stack.parse_pilots(
        "[{entity: a, kind: autoware}, {entity: b, kind: autoware}, {entity: c, kind: autoware, domain: 8}]",
        "carla", "8", "9")
    assert [p["domain"] for p in pilots] == ["10", "11", "8"]


def test_old_ego_holds_ego_domain():
    pilots = stack.parse_pilots("[{entity: bg, kind: autoware}]", "carla", "1", "9", ego_taken=True)
    assert pilots[0]["domain"] == "2"
    with pytest.raises(RuntimeError, match="domain 1 is taken"):
        stack.parse_pilots("[{entity: bg, kind: autoware, domain: 1}]", "carla", "1", "9",
                           ego_taken=True)


@pytest.mark.parametrize("text,match", [
    ("[{entity: a, kind: autoware, domain: 9}]", "domain 9 is taken"),
    ("[{entity: a, kind: autoware, domain: 58}]", "use 0-57"),
    ("[{entity: a, kind: autoware, domain: x}]", "not a number"),
    ("[{entity: a, kind: autoware}, {entity: a, kind: none}]", "twice"),
    ("[{kind: autoware}]", "no entity"),
    ("[{entity: a, kind: container}]", "kind 'container'"),
    ("[{entity: a, kind: autoware, lifetime: run}]", "lifetime 'run'"),
    ("[{entity: a, kind: launch}]", "needs launch"),
    ("[{entity: a, kind: command}]", "needs run"),
    ("[{entity: a, kind: autoware, args: [x]}]", "args is a mapping"),
    ("[a, b]", "a mapping"),
    ("just text", "a list"),
    ("[{entity: a", "not YAML"),
])
def test_refusals(text, match):
    with pytest.raises(RuntimeError, match=match):
        stack.parse_pilots(text, "carla", "1", "9")


def test_none_takes_no_domain_and_starts_nothing():
    pilots = stack.parse_pilots("[{entity: user, kind: none}, {entity: a, kind: autoware}]",
                                "carla", "1", "9")
    assert pilots[0]["domain"] is None and pilots[1]["domain"] == "1"
    assert [n for n, *_ in stack.pilot_launches(pilots, "/p", {})] == ["pilot_a"]


def test_argument_values():
    (p,) = stack.parse_pilots(
        "[{entity: c, kind: command, run: [python3, -m, x, --name, 'a b'], vehicle_side: true,"
        " seed: 3, args: {seed: 4}}]", "carla", "1", "9")
    assert p["args"] == {"run": "python3 -m x --name 'a b'", "vehicle_side": "true", "seed": "4"}


def test_isaac_rules():
    (p,) = stack.parse_pilots("[{entity: ego, kind: autoware}]", "isaac", "1", "9")
    assert p["domain"] == "1"
    with pytest.raises(RuntimeError, match="world server's"):
        stack.parse_pilots("[{entity: ego, kind: autoware, domain: 3}]", "isaac", "1", "9")
    with pytest.raises(RuntimeError, match="one pilot has a vehicle side"):
        stack.parse_pilots("[{entity: ego, kind: autoware}, {entity: b, kind: launch,"
                           " launch: x.xml, domain: 1}]", "isaac", "1", "9")
    with pytest.raises(RuntimeError, match="CARLA only"):
        stack.parse_pilots("[{entity: n, kind: carla_basic}]", "isaac", "1", "9")
    # a command without a vehicle side may run anywhere
    pilots = stack.parse_pilots("[{entity: ego, kind: autoware}, {entity: c, kind: command,"
                                " run: [x]}]", "isaac", "1", "9")
    assert pilots[1]["domain"] == "2"


def test_pilot_launches_merge_contract():
    pilots = stack.parse_pilots(
        "[{entity: ego, kind: autoware, vehicle: my_car}, {entity: n, kind: carla_basic}]",
        "carla", "1", "9")
    common = {"relay": "tcp://localhost:5560", "map_path": "/m", "vehicle": "tesla_model3",
              "sim": "carla", "use_sim_time": "true", "spec_hash_file": "", "carla_host": ""}
    (ego, npc) = stack.pilot_launches(pilots, "/share/pilots", common)
    name, path, args, domain, ros = ego
    assert (name, path, domain, ros) == ("pilot_ego", "/share/pilots/autoware.launch.xml", "1", True)
    assert args == {"relay": "tcp://localhost:5560", "map_path": "/m", "vehicle": "my_car",
                    "sim": "carla", "use_sim_time": "true", "entity": "ego"}
    assert npc[1].endswith("carla_basic.launch.py") and npc[4] is False and npc[3] == "2"


def test_the_release_check_resolves_what_the_stock_pilot_passes():
    """scripts/release/check_autoware_side.sh resolves the user's autoware_launch with the
    stock pilot's own arguments: the two lists are one."""
    import re
    script = os.path.join(os.path.dirname(__file__), "..", "..", "..", "scripts", "release",
                          "check_autoware_side.sh")
    if not os.path.isfile(script):   # an installed package without the checkout
        pytest.skip("not in a scensim checkout")
    text = open(script).read()
    words = re.findall(r"(\S+):=(\S+)",
                       re.search(r'^CAR="(.*?)"', text, re.S | re.M).group(1).replace("\\\n", " ")
                       + " " + re.search(r'^STOCK="(.*?)"', text, re.S | re.M).group(1)
                       .replace("\\\n", " "))
    from_script = {k for k, _ in words} | {"data_path"}
    _, root = xml_args("autoware.launch.xml")
    stock = root.findall("group")[-2].find("include")
    passed = {a.get("name") for a in stock.findall("arg")}
    # data_path: the check's stub of Autoware's models (a user's own is their Autoware's)
    assert from_script - {"data_path"} == passed
