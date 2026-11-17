"""The carla_basic pilot (docs/design/projects-and-pilots.md, "Pilots"): CARLA's BasicAgent
drives the entity's vehicle; no vehicle side, no ROS, no Autoware. CARLA only.

    play_launch launch scensim_launch carla_basic.launch.py entity:=npc_1 \\
        relay:=tcp://localhost:5560 [carla_host:=localhost] [carla_port:=2000] \\
        [carla_pythonapi:=<CARLA>/PythonAPI/carla] [config:=<carla_basic.yaml>]

It runs `ros2 run scensim_agent carla_basic --relay R --entity E --config C`
(scensim_agent.adapters.carla_basic), respawned: the agent exits 3 when CARLA stops answering,
for a fresh client on the next server. C is a JSON file this launch writes (into run_dir when
given, else a temporary directory) from `config` (a YAML/JSON file of the adapter's keys:
target_speed_kmh, ignore_traffic_lights, ...) overlaid with carla_host, carla_port, role_name
and carla_pythonapi from the arguments.

Needs CARLA's Python client: the `carla` module importable by the python3 that runs it (the
CARLA 0.9.16 wheel, pip `carla==0.9.16`, or its PythonAPI/carla/dist egg on PYTHONPATH), and
the `agents` package from CARLA's PythonAPI/carla: `carla_pythonapi:=<dir>` (scensim's stack
passes <carla.root>/PythonAPI/carla from the host settings), else $CARLA_DIR/PythonAPI/carla.

role_name: the bridge spawns the scenario's ego as `hero` (bridge_config ego.role_name) and an
agent-driven entity under its own name; the default is hero for entity ego, else the entity.

seed, run_dir, map_path, vehicle, sim, use_sim_time and the release endpoints are the common
contract's and unused here (run_dir holds the written config).
"""
import json
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

CONTRACT = {
    "entity": ("ego", "the scenario entity this pilot drives"),
    "relay": ("tcp://localhost:5560", "the agent relay, tcp://host:port"),
    "map_path": ("", "the map directory (unused: BasicAgent plans on CARLA's own map)"),
    "vehicle": ("tesla_model3", "the car (unused: the bridge spawned it)"),
    "seed": ("0", "the run's seed (unused)"),
    "run_dir": ("", "the run's directory; the written config goes there when given"),
    "sim": ("carla", "carla only"),
    "use_sim_time": ("true", "unused (no ROS)"),
    "carla_host": ("localhost", "CARLA's RPC host"),
    "carla_port": ("2000", "CARLA's RPC port"),
    "release_notify_endpoint": ("", "unused (no sensors)"),
    "release_ack_endpoint": ("", "unused (no sensors)"),
}
EXTRA = {
    "role_name": ("", "the vehicle's CARLA role_name; empty: hero for entity ego, else the entity"),
    "carla_pythonapi": (os.path.join(os.environ.get("CARLA_DIR", ""), "PythonAPI", "carla")
                        if os.environ.get("CARLA_DIR") else "",
                        "CARLA's PythonAPI/carla (holds `agents`); default $CARLA_DIR/PythonAPI/carla"),
    "config": ("", "a YAML/JSON file of the adapter's settings; empty: its defaults"),
}


def agent_config(values):
    """The adapter's config: the user's file overlaid with the CARLA endpoint and role_name."""
    config = {}
    if values.get("config"):
        with open(values["config"]) as f:
            text = f.read()
        if values["config"].endswith((".yaml", ".yml")):
            import yaml
            config = yaml.safe_load(text) or {}
        else:
            config = json.loads(text)
    entity = values["entity"]
    config.update(carla_host=values["carla_host"], carla_port=int(values["carla_port"]),
                  role_name=values.get("role_name") or ("hero" if entity == "ego" else entity))
    if values.get("carla_pythonapi"):
        config["carla_pythonapi"] = values["carla_pythonapi"]
    return config


def command(values, config_path):
    return ["ros2", "run", "scensim_agent", "carla_basic", "--relay", values["relay"],
            "--entity", values["entity"], "--config", config_path]


def _pilot(context):
    values = {k: LaunchConfiguration(k).perform(context) for k in (*CONTRACT, *EXTRA)}
    if values["sim"] != "carla":
        raise RuntimeError(f"pilot carla_basic: sim is {values['sim']!r}; carla_basic is CARLA's only")
    if values["relay"] in ("", "none"):
        raise RuntimeError("pilot carla_basic: relay:= the agent relay (tcp://host:port)")
    folder = values["run_dir"] or tempfile.mkdtemp(prefix=f"scensim-carla_basic-{values['entity']}-")
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"carla_basic-{values['entity']}.json")
    with open(path, "w") as f:
        json.dump(agent_config(values), f, indent=2)
    return [ExecuteProcess(cmd=command(values, path), name=f"carla_basic_{values['entity']}",
                           output="screen", respawn=True, respawn_delay=3.0)]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=d, description=t)
         for k, (d, t) in {**CONTRACT, **EXTRA}.items()]
        + [OpaqueFunction(function=_pilot)])
