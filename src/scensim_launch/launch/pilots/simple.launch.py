"""The simple pilot (docs/design/projects-and-pilots.md, "Pilots"; roadmap 070 D): scensim_agent's
simple_pilot drives the entity's car through the vehicle interface, no Autoware. It routes on
the map's lanelet2 file, steers by pure pursuit and holds the speed with a PID; it ignores
traffic (lights, vehicles, pedestrians).

    play_launch launch scensim_launch simple.launch.py entity:=ego \\
        relay:=tcp://localhost:5560 map_path:=<map dir> [sim:=carla|isaac] \\
        [vehicle:=tesla_model3] [config:=<simple_pilot.yaml>] [odom_topic:=<topic>]

Starts, in this ROS domain:

- the vehicle side (vehicle_side.launch.xml): on CARLA acb_bridge for the car; on Isaac
  nothing, the car's rig is the world server's (and publishes on the ego's domain, which an
  Isaac pilot runs in);
- robot_state_publisher with the car's description (config/pilot_vehicle.xacro: the car's
  <vehicle>_description and <vehicle>_sensor_kit_description), which CARLA's vehicle bridge
  waits for before it attaches; publish_description:=false leaves it out;
- `ros2 run scensim_agent simple_pilot --relay R --entity E --config C`, respawned. C is a JSON
  file this launch writes (into run_dir when given, else a temporary directory) from `config`
  (a YAML/JSON file of the adapter's keys: max_speed_kmh, lateral_accel, ...) overlaid with
  map_path, sim, vehicle and, when given, odom_topic and max_speed_kmh.

The pose is the car's ground truth: on CARLA acb's /carla/ground_truth/odom; on Isaac aib's
/ground_truth/odom, which the world server publishes when the ego config turns it on (scensim's
stack does for a simple pilot; by hand, an ego config with `ground_truth: {}`).

seed and use_sim_time are the common contract's; the agent runs on simulation time.
"""
import json
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# beside this file, in the source tree and in scensim_launch's share alike
HERE = os.path.dirname(os.path.abspath(__file__))
XACRO = os.path.normpath(os.path.join(HERE, "..", "..", "config", "pilot_vehicle.xacro"))

CONTRACT = {
    "entity": ("ego", "the scenario entity this pilot drives"),
    "relay": ("tcp://localhost:5560", "the agent relay, tcp://host:port"),
    "map_path": ("", "the map directory: its lanelet2_map.osm is what the pilot routes on"),
    "vehicle": ("tesla_model3", "the car: its spec (wheelbase, steering) and description"),
    "seed": ("0", "the run's seed (unused)"),
    "run_dir": ("", "the run's directory; the written config goes there when given"),
    "sim": ("carla", "carla | isaac: the vehicle side, and the pose topic's default"),
    "use_sim_time": ("true", "the agent and robot_state_publisher run on simulation time"),
    "carla_host": ("localhost", "CARLA's RPC host (the vehicle side)"),
    "carla_port": ("2000", "CARLA's RPC port (the vehicle side)"),
    "release_notify_endpoint": ("tcp://127.0.0.1:5556", "swb_carla_world's release PUB socket"),
    "release_ack_endpoint": ("tcp://127.0.0.1:5557", "swb_carla_world's release PULL socket"),
}
EXTRA = {
    "config": ("", "a YAML/JSON file of simple_pilot's settings; empty: its defaults"),
    "odom_topic": ("", "the car's pose (nav_msgs/Odometry, map frame); empty: "
                       "/carla/ground_truth/odom on CARLA, /ground_truth/odom on Isaac"),
    "max_speed_kmh": ("", "never faster than this; empty: the config's (50)"),
    "publish_description": ("true", "start robot_state_publisher with the car's description"),
    "spec": ("", "the car's spec file (its wheelbase and steering); empty: sim_vehicle_spec's "
                 "vehicles/<vehicle>.yaml"),
    "description": ("", "a URDF of the car to publish; empty: the car's Autoware description "
                        "packages through pilot_vehicle.xacro (an own car: the stack's, "
                        "generated from its spec)"),
}


def agent_config(values):
    """simple_pilot's config: the user's file, then the map, the simulator and the car."""
    config = {}
    if values.get("config"):
        with open(values["config"]) as f:
            text = f.read()
        if values["config"].endswith((".yaml", ".yml")):
            import yaml
            config = yaml.safe_load(text) or {}
        else:
            config = json.loads(text)
    config.update(map_path=values["map_path"], sim=values["sim"], vehicle=values["vehicle"],
                  use_sim_time=values["use_sim_time"] == "true")
    if values.get("odom_topic"):
        config["odom_topic"] = values["odom_topic"]
    if values.get("max_speed_kmh"):
        config["max_speed_kmh"] = float(values["max_speed_kmh"])
    if values.get("spec"):
        config["spec"] = values["spec"]
    return config


def command(values, config_path):
    return ["ros2", "run", "scensim_agent", "simple_pilot", "--relay", values["relay"],
            "--entity", values["entity"], "--config", config_path]


def _pilot(context):
    values = {k: LaunchConfiguration(k).perform(context) for k in (*CONTRACT, *EXTRA)}
    if values["relay"] in ("", "none"):
        raise RuntimeError("pilot simple: relay:= the agent relay (tcp://host:port)")
    if not values["map_path"]:
        raise RuntimeError("pilot simple: map_path:= the map directory (it routes on its "
                           "lanelet2_map.osm)")
    # every argument this launch was given reaches the vehicle side (ROS launch's arguments
    # are global; play_launch's parser scopes an include to the arguments it names): an own
    # car's vehicle_config and autoware_spec_hash_file (roadmap 077, G5)
    passed = dict(context.launch_configurations)
    passed["sim"] = values["sim"]
    actions = [IncludeLaunchDescription(AnyLaunchDescriptionSource(
        os.path.join(HERE, "vehicle_side.launch.xml")),
        launch_arguments=list(passed.items()))]
    if values["publish_description"] == "true":
        vehicle = values["vehicle"]
        if values.get("description"):
            with open(values["description"]) as f:
                description = f.read()
        else:
            description = Command(["xacro ", XACRO,
                                   f" vehicle_model:={vehicle}",
                                   f" sensor_model:={vehicle}_sensor_kit"])
        actions.append(Node(package="robot_state_publisher", executable="robot_state_publisher",
                            name="robot_state_publisher", output="screen",
                            parameters=[{"use_sim_time": values["use_sim_time"] == "true",
                                         "robot_description": ParameterValue(description,
                                                                             value_type=str)}]))
    folder = values["run_dir"] or tempfile.mkdtemp(prefix=f"scensim-simple-{values['entity']}-")
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"simple_pilot-{values['entity']}.json")
    with open(path, "w") as f:
        json.dump(agent_config(values), f, indent=2)
    actions.append(ExecuteProcess(cmd=command(values, path),
                                  name=f"simple_pilot_{values['entity']}",
                                  output="screen", respawn=True, respawn_delay=3.0))
    return actions


def generate_launch_description():
    dds = os.environ.get("CYCLONEDDS_URI") or (
        "file://" + os.path.join(get_package_share_directory("scensim_launch"), "config",
                                 "cyclonedds-localhost.xml"))
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=d, description=t)
         for k, (d, t) in {**CONTRACT, **EXTRA}.items()]
        + [SetEnvironmentVariable("CYCLONEDDS_URI", dds),
           SetEnvironmentVariable("RMW_IMPLEMENTATION",
                                  os.environ.get("RMW_IMPLEMENTATION") or "rmw_cyclonedds_cpp"),
           OpaqueFunction(function=_pilot)])
