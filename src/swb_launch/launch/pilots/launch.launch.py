"""The launch pilot (docs/design/projects-and-pilots.md, "Pilots"): the vehicle side, the
user's launch file as the driver, and swb's vehicle agent unless agent:=own.

    play_launch launch swb_launch launch.launch.py entity:=ego relay:=tcp://localhost:5560 \\
        map_path:=<map dir> sim:=carla launch:='<package> <file>'|<path> \\
        [launch_args:='a:=1 b:=2'] [agent:=swb|own] [vehicle:=tesla_model3]

The user's launch receives the common contract -- entity, relay, map_path, vehicle, seed,
run_dir, sim, use_sim_time -- and then launch_args (which win). The vehicle side is
vehicle_side.launch.xml (CARLA: acb_bridge; Isaac: the world server's rig, nothing here), given
every argument of this launch; vehicle_side:=false leaves it out. agent:=own: the user's launch
registers the entity with the relay itself.
"""
import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

CONTRACT = {
    "entity": ("ego", "the scenario entity this pilot drives"),
    "relay": ("none", "the agent relay, tcp://host:port; none = no relay"),
    "map_path": ("", "the map directory"),
    "vehicle": ("tesla_model3", "the car: its spec name"),
    "seed": ("0", "the run's seed"),
    "run_dir": ("", "the run's directory; empty: none"),
    "sim": ("carla", "carla | isaac"),
    "use_sim_time": ("true", "use simulation time"),
    "carla_host": ("localhost", "CARLA's RPC host (the vehicle side)"),
    "carla_port": ("2000", "CARLA's RPC port (the vehicle side)"),
    "release_notify_endpoint": ("tcp://127.0.0.1:5556", "swb_carla_world's release PUB socket"),
    "release_ack_endpoint": ("tcp://127.0.0.1:5557", "swb_carla_world's release PULL socket"),
}
DRIVER_CONTRACT = ("entity", "relay", "map_path", "vehicle", "seed", "run_dir", "sim", "use_sim_time")
EXTRA = {
    "launch": ("", "the driver: '<package> <file>' or a path"),
    "launch_args": ("", "name:=value ... for the driver's launch"),
    "agent": ("swb", "swb: swb's vehicle agent registers the entity; own: the driver's launch does"),
    "vehicle_side": ("true", "false: no vehicle side (the driver brings its own)"),
    "spec_hash_file": ("", "the agent's spec_hash YAML instead of the vehicle's description package's"),
    "agent_config": ("", "the vehicle agent's settings (YAML/JSON); empty = defaults"),
}


def named_args(text, what):
    out = {}
    for tok in shlex.split(text or ""):
        name, sep, value = tok.partition(":=")
        if not sep or not name:
            raise RuntimeError(f"pilot launch: {what}: '{tok}' is not name:=value")
        out[name] = value
    return out


def resolve_launch(spec):
    """A path, or "<package> <file>": the file under the package's share (launch/ first)."""
    parts = shlex.split(spec or "")
    if len(parts) == 1:
        path = os.path.abspath(os.path.expanduser(parts[0]))
        if not os.path.isfile(path):
            raise RuntimeError(f"pilot launch: no file {path}")
        return path
    if len(parts) != 2:
        raise RuntimeError("pilot launch: launch:= '<package> <file>' or a path, "
                           f"not {spec!r}")
    package, name = parts
    share = get_package_share_directory(package)
    for candidate in (os.path.join(share, "launch", name), os.path.join(share, name)):
        if os.path.isfile(candidate):
            return candidate
    for root, _, files in os.walk(share):
        if name in files:
            return os.path.join(root, name)
    raise RuntimeError(f"pilot launch: no {name} in package {package} ({share})")


def driver_args(values):
    """The contract the user's launch receives, then launch_args."""
    return {**{k: values[k] for k in DRIVER_CONTRACT},
            **named_args(values["launch_args"], "launch_args")}


def _pilot(context):
    values = {k: LaunchConfiguration(k).perform(context) for k in (*CONTRACT, *EXTRA)}
    if values["agent"] not in ("swb", "own"):
        raise RuntimeError(f"pilot launch: agent is {values['agent']!r}: swb or own")
    swb = get_package_share_directory("swb_launch")
    actions = []
    if values["vehicle_side"] == "true":
        actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(
            os.path.join(swb, "launch", "pilots", "vehicle_side.launch.xml")),
            # every argument given here reaches the vehicle side (as simple.launch.py)
            launch_arguments=list({**context.launch_configurations,
                                   "sim": values["sim"]}.items())))
    actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(
        resolve_launch(values["launch"])), launch_arguments=driver_args(values).items()))
    if values["agent"] == "swb":
        agent = {"map_path": values["map_path"], "use_sim_time": values["use_sim_time"],
                 "relay": "" if values["relay"] == "none" else values["relay"],
                 "entity": values["entity"], "vehicle_model": values["vehicle"],
                 "spec_hash_file": values["spec_hash_file"], "agent_config": values["agent_config"]}
        actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(
            os.path.join(swb, "launch", "vehicle_agent.launch.xml")),
            launch_arguments=agent.items()))
    return actions


def generate_launch_description():
    dds = os.environ.get("CYCLONEDDS_URI") or (
        "file://" + os.path.join(get_package_share_directory("swb_launch"), "config",
                                 "cyclonedds-localhost.xml"))
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=d, description=t)
         for k, (d, t) in {**CONTRACT, **EXTRA}.items()]
        + [SetEnvironmentVariable("CYCLONEDDS_URI", dds),
           SetEnvironmentVariable("RMW_IMPLEMENTATION",
                                  os.environ.get("RMW_IMPLEMENTATION") or "rmw_cyclonedds_cpp"),
           OpaqueFunction(function=_pilot)])
