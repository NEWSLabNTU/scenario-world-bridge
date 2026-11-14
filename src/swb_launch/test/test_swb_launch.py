"""swb.launch.xml / swb.launch.py (roadmap 076 step 6): the wrapper passes every argument, and
stack.launch.py takes every argument swb.stack's plan gives it."""
import importlib.util
import os
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH = os.path.join(HERE, "..", "launch")


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


swb = load(os.path.join(LAUNCH, "swb.launch.py"), "swb_launch_py")
stack = load(os.path.join(LAUNCH, "stack.launch.py"), "stack_launch_py")


def declared(ld):
    from launch.actions import DeclareLaunchArgument
    return {e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)}


def test_xml_wrapper_passes_every_argument():
    root = ET.parse(os.path.join(LAUNCH, "swb.launch.xml")).getroot()
    args = {a.get("name") for a in root.findall("arg")}
    inc = root.find("include")
    assert inc.get("file").endswith("/launch/swb.launch.py")
    passed = {a.get("name"): a.get("value") for a in inc.findall("arg")}
    assert args == set(swb.ARGS) == set(passed)
    assert all(v == f"$(var {k})" for k, v in passed.items())


def test_swb_launch_declares_its_arguments():
    assert declared(swb.generate_launch_description()) == set(swb.ARGS)


def test_stack_takes_the_plans_arguments():
    names = declared(stack.generate_launch_description())
    plan_args = {"sim", "ego_launch", "ego_domain", "scenario_domain", "ssv2_port", "agent_port",
                 "world_port", "carla_host", "carla_port", "log_dir", "map_path",
                 "vehicle_model", "nest_side", "pilots", "sim_args"}
    assert plan_args <= names


def test_named_args():
    assert swb.named_args("a:=1 'b:=x y'") == {"a": "1", "b": "x y"}


def test_retired_arguments_are_refused_by_name():
    """roadmap 076 step 10: ego_launch and the rest, given to swb.launch.xml, are refused
    before anything is planned, naming what replaced them."""
    import pytest
    from launch import LaunchContext
    context = LaunchContext()
    for name, (default, _) in swb.ARGS.items():
        context.launch_configurations[name] = default
    context.launch_configurations["ego_launch"] = "none"
    context.launch_configurations["ego_launch_args"] = "rviz:=true"
    with pytest.raises(RuntimeError) as e:
        swb._swb(context)
    msg = str(e.value)
    assert "ego_launch:= is retired" in msg and "ego_launch_args:= is retired" in msg
    assert "swb.yaml" in msg


def test_another_simulator_starts_no_ego():
    """stack.launch.py: a simulator without a default autopilot starts no ego (pilots drive)."""
    import inspect
    src = inspect.getsource(stack._stack)
    assert "sim not in PRESETS" in src and "ego_launch:= your launch" not in src
