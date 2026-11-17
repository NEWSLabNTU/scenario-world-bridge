"""isaac_t2/launch_own's driver: a user's own launch file as the ego's pilot (kind launch, agent: own).

It runs scensim_agent's simple_pilot, which registers the entity with the relay itself and drives on
the true pose (`inputs: [ground_truth]` turns Isaac's /ground_truth/odom on). The stack hands every
launch pilot the common contract (entity, relay, map_path, vehicle, seed, run_dir, sim,
use_sim_time); this launch turns map_path and sim into simple_pilot's config.
"""
import json
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

CONTRACT = ("entity", "relay", "map_path", "vehicle", "seed", "run_dir", "sim", "use_sim_time")


def driver(context):
    arg = {a: LaunchConfiguration(a).perform(context) for a in CONTRACT}
    out = arg["run_dir"] or tempfile.mkdtemp(prefix="own_driver_")
    os.makedirs(out, exist_ok=True)
    config = os.path.join(out, "own_driver.json")
    with open(config, "w") as f:
        json.dump({"map_path": arg["map_path"], "sim": arg["sim"] or "isaac"}, f)
    return [ExecuteProcess(cmd=["ros2", "run", "scensim_agent", "simple_pilot", "--relay", arg["relay"],
                                "--entity", arg["entity"], "--config", config],
                           output="screen", respawn=True, respawn_delay=5.0)]


def generate_launch_description():
    return LaunchDescription([*(DeclareLaunchArgument(a, default_value="") for a in CONTRACT),
                              OpaqueFunction(function=driver)])
