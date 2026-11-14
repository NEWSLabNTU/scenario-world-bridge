"""The command pilot (docs/design/projects-and-pilots.md, "Pilots"): the user's program is
the driver and its own agent: it registers the entity with the relay itself (swb_agent's SDK).

    play_launch launch swb_launch command.launch.py entity:=cut_in \\
        relay:=tcp://localhost:5560 run:='python3 -m my_policies.cut_in --policy push' \\
        [seed:=1002] [run_dir:=<dir>] [vehicle_side:=false] [respawn:=true]

It runs `run` (shell words, not a shell) with `--relay R --entity E --seed S` appended, and
`--run-dir D` when run_dir is given. No ROS vehicle side unless vehicle_side:=true
(vehicle_side.launch.xml: CARLA's acb_bridge in this domain; on Isaac the world server's rig),
for a program that drives Autoware's vehicle interface. respawn:=true (the default) starts the
program again when it ends, as the stack's other pilots are.
"""
import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

CONTRACT = {
    "entity": ("ego", "the scenario entity this pilot drives"),
    "relay": ("tcp://localhost:5560", "the agent relay, tcp://host:port"),
    "map_path": ("", "the map directory (the vehicle side's)"),
    "vehicle": ("tesla_model3", "the car: its spec name (the vehicle side's)"),
    "seed": ("0", "the run's seed, given as --seed"),
    "run_dir": ("", "the run's directory, given as --run-dir; empty: not given"),
    "sim": ("carla", "carla | isaac (the vehicle side's)"),
    "use_sim_time": ("true", "use simulation time (the vehicle side's)"),
    "carla_host": ("localhost", "CARLA's RPC host (the vehicle side)"),
    "carla_port": ("2000", "CARLA's RPC port (the vehicle side)"),
    "release_notify_endpoint": ("tcp://127.0.0.1:5556", "swb_carla_world's release PUB socket"),
    "release_ack_endpoint": ("tcp://127.0.0.1:5557", "swb_carla_world's release PULL socket"),
}
EXTRA = {
    "run": ("", "the program and its arguments, as shell words"),
    "vehicle_side": ("false", "true: start the simulator's ROS vehicle side too"),
    "respawn": ("true", "start the program again when it ends"),
}


def argv(values):
    """The user's words, then what every command pilot is given."""
    words = shlex.split(values["run"] or "")
    if not words:
        raise RuntimeError("pilot command: run:= the program to run (shell words)")
    words += ["--relay", values["relay"], "--entity", values["entity"], "--seed", values["seed"]]
    if values["run_dir"]:
        words += ["--run-dir", values["run_dir"]]
    return words


def _pilot(context):
    values = {k: LaunchConfiguration(k).perform(context) for k in (*CONTRACT, *EXTRA)}
    actions = []
    if values["vehicle_side"] == "true":
        swb = get_package_share_directory("swb_launch")
        actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(
            os.path.join(swb, "launch", "pilots", "vehicle_side.launch.xml")),
            launch_arguments=[("sim", values["sim"])]))
    respawn = values["respawn"] == "true"
    actions.append(ExecuteProcess(cmd=argv(values), name=f"command_{values['entity']}",
                                  output="screen", respawn=respawn,
                                  respawn_delay=3.0 if respawn else 0.0))
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
