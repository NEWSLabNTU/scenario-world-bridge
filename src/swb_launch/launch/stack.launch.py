"""The whole stack in one launch (roadmap 073): a simulator, the simulation side, the user's
autopilot, and optionally an SSv2 scenario. swb names no car, scenario or use case here: each
is a slot.

    play_launch launch swb_launch stack.launch.py sim:=isaac|carla|<package> \\
        [sim_args:="name:=value ..."] [ego_launch:=<path>|"<package> <file>"|none] \\
        [ego_launch_args:="name:=value ..."] [vehicle_model:=...] [spec_hash_file:=...] \\
        [vehicle_catalogs:=...] [scenario:=<path to .xosc>] [scenario_args:="..."] \\
        [exit_on_scenario_end:=false]

The shortcuts for the supported simulators, isaac.launch.xml and carla.launch.xml, fill
`sim` and `sim_args` from arguments of their own.

Groups and their ROS domains:
- sim (ego domain): the world server.
  - isaac: swb_isaac_world.launch.xml.
  - carla: swb_carla_world.launch.xml; CARLA itself runs apart (`just carla-start`).
  - <package>: that package's launch/world.launch.xml, which starts a world server speaking
    swb_world.proto (docs/design/simulator-interface-split.md; swb_world_check conformance)
    on `world_endpoint`.
  `sim_args` go to that launch.
- simulation side (scenario domain): the bridge and the agent relay (simulation.launch.xml).
  `side_args` go to that launch (config_file, weather, gap_trace_dir, npc_motion, ...).
- ego (ego domain): the autopilot's launch and swb's vehicle agent (vehicle_agent.launch.xml),
  as a nested play_launch, respawned. An autopilot stack ends with its episode (a world load,
  a simulator restart: acb_bridge is required), and that must not end the stack; the
  respawn brings it back.
  - Default ego_launch: on isaac, aib_launch isaac_simulator.launch.xml (aib's Autoware
    profile; the vehicle interface and sensors are the world server's); on carla,
    acb_launch carla_simulator.launch.xml (Autoware with acb_bridge, CARLA's vehicle
    bridge). none: no ego.
- run (scenario domain): with `scenario`, SSv2 (scenario.launch.xml) as a nested play_launch:
  it shuts its own launch down when the scenario ends, which must not end the stack. With
  exit_on_scenario_end:=true the stack ends with it (scripted runs). Run the next scenario
  against the live stack with `play_launch launch swb_launch scenario.launch.xml
  scenario:=<path>` in the scenario domain.

Time is synchronous with a scenario: SSv2 steps the world frame by frame through the bridge
(CARLA in synchronous mode, Isaac's world server stepped), at global_real_time_factor
(scenario_args), as fast as the stack keeps up.

Host settings are the environment's, as for `just`: EGO_ROS_DOMAIN_ID (1),
SCENARIO_ROS_DOMAIN_ID (9), SSV2_PORT (5555), AGENT_PORT (5560), ISAAC_PORT (5590),
WORLD_PORT (5591), CARLA_HOST, CARLA_PORT, SWB_LOG_ROOT (the nested launches' logs, default
./play_log), CYCLONEDDS_URI (default swb_launch's localhost profile), RMW_IMPLEMENTATION.
"""
import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PRESETS = ("isaac", "carla")


def _env(name, default):
    return os.environ.get(name) or default


def named_args(text, what):
    """`name:=value ...` as a dict; an empty value is left out (the launch's default)."""
    out = {}
    for tok in shlex.split(text or ""):
        name, sep, value = tok.partition(":=")
        if not sep or not name:
            raise RuntimeError(f"stack: {what}: '{tok}' is not name:=value")
        if value != "":
            out[name] = value
    return out


def launch_target(spec, what):
    """A launch file as play_launch takes it: a path, or "<package> <file>"."""
    parts = shlex.split(spec)
    if len(parts) == 1:
        if not os.path.isfile(parts[0]):
            raise RuntimeError(f"stack: {what}: no file {parts[0]}")
        return [os.path.abspath(parts[0])]
    if len(parts) == 2:
        return parts
    raise RuntimeError(f"stack: {what}: '{spec}' is neither a path nor '<package> <file>'")


def include(path, args):
    return IncludeLaunchDescription(AnyLaunchDescriptionSource(path), launch_arguments=args.items())


def nested(name, target, args, domain, log_dir, respawn=False, extra=()):
    """A launch as its own play_launch process: its own lifetime, its own log directory."""
    cmd = ["play_launch", "launch", "--disable-web-ui", "--log-dir", log_dir, *extra, *target]
    cmd += [f"{k}:={v}" for k, v in args.items()]
    return ExecuteProcess(cmd=cmd, name=name, output="screen", respawn=respawn,
                          respawn_delay=5.0 if respawn else 0.0,
                          additional_env={"ROS_DOMAIN_ID": domain})


def _stack(context):
    get = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    sim = get("sim")
    if not sim:
        raise RuntimeError("stack: sim:= isaac, carla or a package of yours with launch/world.launch.xml")
    ego_domain = get("ego_domain")
    scenario_domain = get("scenario_domain")
    ssv2_port, agent_port = get("ssv2_port"), get("agent_port")
    log_root = os.path.abspath(get("log_dir"))
    sim_args = named_args(get("sim_args"), "sim_args")
    swb = get_package_share_directory("swb_launch")

    # --- sim
    endpoint = get("world_endpoint")
    if sim == "isaac":
        world = os.path.join(get_package_share_directory("swb_isaac_world"), "launch",
                             "swb_isaac_world.launch.xml")
        endpoint = endpoint or f"tcp://127.0.0.1:{_env('ISAAC_PORT', '5590')}"
        sim_env = [SetEnvironmentVariable("EGO_ROS_DOMAIN_ID", ego_domain)]
        timeouts = {}
    elif sim == "carla":
        world = os.path.join(get_package_share_directory("swb_carla_world"), "launch",
                             "swb_carla_world.launch.xml")
        port = _env("WORLD_PORT", "5591")
        endpoint = endpoint or f"tcp://127.0.0.1:{port}"
        sim_args = {"port": port, "ssv2_port": ssv2_port,
                    "carla_host": _env("CARLA_HOST", ""), "carla_port": _env("CARLA_PORT", ""),
                    **sim_args}
        sim_env = []
        # above CARLA's own watchdogs (carla_simulation.launch.xml)
        timeouts = {"world_request_timeout_seconds": "90", "world_load_timeout_seconds": "300"}
    else:
        world = os.path.join(get_package_share_directory(sim), "launch", "world.launch.xml")
        if not os.path.isfile(world):
            raise RuntimeError(f"stack: sim {sim!r}: no {world} (a simulator package installs "
                               "launch/world.launch.xml)")
        if not endpoint:
            raise RuntimeError(f"stack: sim {sim!r}: world_endpoint:= the world server's "
                               "tcp://host:port")
        sim_env, timeouts = [], {}
    actions = [GroupAction(scoped=True, actions=[
        SetEnvironmentVariable("ROS_DOMAIN_ID", ego_domain), *sim_env, include(world, sim_args)])]

    # --- simulation side
    side = {"world_endpoint": endpoint, "ssv2_port": ssv2_port, "agent_port": agent_port,
            "vehicle_catalogs": get("vehicle_catalogs"), **timeouts,
            **named_args(get("side_args"), "side_args")}
    actions.append(GroupAction(scoped=True, actions=[
        SetEnvironmentVariable("ROS_DOMAIN_ID", scenario_domain),
        include(os.path.join(swb, "launch", "simulation.launch.xml"), side)]))

    # --- ego
    ego_launch = get("ego_launch")
    if ego_launch != "none":
        if not ego_launch:
            ego_launch = {"isaac": "aib_launch isaac_simulator.launch.xml",
                          "carla": "acb_launch carla_simulator.launch.xml"}.get(sim)
            if not ego_launch:
                raise RuntimeError(f"stack: sim {sim!r} has no default autopilot: ego_launch:= "
                                   "your launch, or none")
            # the preset's Autoware: its map, and on CARLA the server it drives
            preset = {"map_path": get("map_path")}
            if sim == "carla":
                preset.update(carla_host=_env("CARLA_HOST", ""), carla_port=_env("CARLA_PORT", ""))
            if not get("vehicle_model"):
                context.launch_configurations["vehicle_model"] = "tesla_model3"
        else:
            preset = {}
        ego_args = {**{k: v for k, v in preset.items() if v},
                    **named_args(get("ego_launch_args"), "ego_launch_args")}
        relay = f"tcp://localhost:{agent_port}"
        actions.append(nested("ego", launch_target(ego_launch, "ego_launch"), ego_args, ego_domain,
                              os.path.join(log_root, "ego"), respawn=True,
                              extra=("--container-mode", "isolated", "--composable-respawn",
                                     "on-crash")))
        if get("agent") == "true":
            agent = {"relay": relay, "entity": get("entity"), "vehicle_model": get("vehicle_model"),
                     "spec_hash_file": get("spec_hash_file")}
            actions.append(nested("vehicle_agent", [os.path.join(swb, "launch",
                                                                 "vehicle_agent.launch.xml")],
                                  {k: v for k, v in agent.items() if v}, ego_domain,
                                  os.path.join(log_root, "agent"), respawn=True))

    # --- run
    scenario = get("scenario")
    if scenario:
        path = os.path.abspath(os.path.expanduser(scenario))
        if not os.path.isfile(path):
            raise RuntimeError(f"stack: no scenario {path}")
        run = {"scenario": path, "port": ssv2_port,
               **named_args(get("scenario_args"), "scenario_args")}
        proc = nested("scenario", [os.path.join(swb, "launch", "scenario.launch.xml")], run,
                      scenario_domain, os.path.join(log_root, "scenario"))
        actions.append(proc)
        if get("exit_on_scenario_end") == "true":
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=proc, on_exit=[EmitEvent(event=Shutdown(reason="scenario ended"))])))
    elif get("spawn"):
        raise RuntimeError("stack: spawn:= (free drive) is not there yet (roadmap 073 S5); "
                           "on Isaac, aib_launch isaac_vehicle.launch.xml drives without a scenario")
    return actions


def generate_launch_description():
    dds = os.environ.get("CYCLONEDDS_URI") or (
        "file://" + os.path.join(get_package_share_directory("swb_launch"), "config",
                                 "cyclonedds-localhost.xml"))
    arg = DeclareLaunchArgument
    return LaunchDescription([
        arg("sim", default_value="", description="isaac, carla, or a package with launch/world.launch.xml"),
        arg("sim_args", default_value="", description="name:=value ... for the world server's launch"),
        arg("world_endpoint", default_value="", description="the world server's tcp://host:port; empty: the preset's"),
        arg("ego_launch", default_value="", description="the autopilot's launch: a path or '<package> <file>'; empty: the sim's default; none: no ego"),
        arg("ego_launch_args", default_value="", description="name:=value ... for ego_launch"),
        arg("agent", default_value="true", description="start swb's vehicle agent beside the autopilot (false: ego_launch has its own)"),
        arg("entity", default_value="ego", description="the scenario entity the ego is"),
        arg("map_path", default_value="", description="the default autopilot's map directory (lanelet2_map.osm, pointcloud_map)"),
        arg("vehicle_model", default_value="", description="the agent's vehicle model (its description's spec_hash.yaml)"),
        arg("spec_hash_file", default_value="", description="the agent's spec_hash YAML, instead of vehicle_model's"),
        arg("vehicle_catalogs", default_value="", description="the bridge's extra vehicle catalogs, comma separated"),
        arg("side_args", default_value="", description="name:=value ... for the simulation side's launch (simulation.launch.xml: config_file, weather, gap_trace_dir, ...)"),
        arg("scenario", default_value="", description="an .xosc to run; empty: none"),
        arg("scenario_args", default_value="", description="name:=value ... for scenario.launch.xml (global_timeout, global_real_time_factor, output_directory, ...)"),
        arg("exit_on_scenario_end", default_value="false", description="true: the stack ends with the scenario"),
        arg("spawn", default_value="", description="free drive at x,y,z,yaw without a scenario (roadmap 073 S5)"),
        arg("ego_domain", default_value=_env("EGO_ROS_DOMAIN_ID", "1")),
        arg("scenario_domain", default_value=_env("SCENARIO_ROS_DOMAIN_ID", "9")),
        arg("ssv2_port", default_value=_env("SSV2_PORT", "5555")),
        arg("agent_port", default_value=_env("AGENT_PORT", "5560")),
        arg("log_dir", default_value=_env("SWB_LOG_ROOT", "play_log"), description="where the nested launches (ego, agent, scenario) log"),
        SetEnvironmentVariable("CYCLONEDDS_URI", dds),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", _env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")),
        OpaqueFunction(function=_stack),
    ])
