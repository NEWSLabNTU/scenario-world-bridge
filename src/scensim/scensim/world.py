"""The simulation side's settings a project names: ``world:`` and ``isaac:`` in ``testbench.yaml``.

docs/design/projects-and-pilots.md, "World settings". ``world:`` goes to the bridge and the
world server (``side_args`` and ``sim_args`` of stack.launch.py), ``isaac:`` to the Isaac
world server (``sim_args``). Both are merged with what scensim gives those launches itself, never
in place of it: a key scensim owns (the state directory, the trace and contact directories, the
world and the ports) is refused by name, in ``testbench.yaml`` and in ``stack_args``.

    world:
      weather: ClearNoon                # a preset, or parameters (below)
      # weather: {preset: ClearNoon, cloudiness: 30, sun_altitude_angle: 60}
      npc_motion: tracked               # tracked | kinematic
      contact_report_dir: off           # off; a directory of yours; unset: per run (scensim's)
      config_file: config/bridge.yaml   # the bridge's bridge_config.yaml
      world_config_file: config/carla_world.yaml   # CARLA: scensim_carla_world.yaml

    isaac:
      ego: config/ego_lockstep.yaml     # an aib ego config; default | none
      render: headless                  # auto | window | headless
      follow: false                     # true | false | editor
      physics_hz: 240
      preload: [Town05]                 # more maps the world server loads at start
      wheels_topic: true

Weather parameters are set through the bridge's ``set_weather`` service once the bridge
serves it, before the stack is ready (the keeper); a preset goes as the bridge's ``weather``
argument, which the bridge applies after every world load and reconnect.
"""
import shlex
from pathlib import Path

WORLD_KEYS = ("weather", "npc_motion", "contact_report_dir", "config_file", "world_config_file")
ISAAC_KEYS = ("ego", "render", "follow", "physics_hz", "preload", "wheels_topic")
NPC_MOTIONS = ("tracked", "kinematic")
RENDERS = ("auto", "window", "headless")
FOLLOWS = ("true", "false", "editor")
CONTACTS_OFF = "off"

# simulation.launch.xml arguments scensim gives itself (stack.launch.py's top-level arguments, or
# the plan's): name -> what to give instead
SIDE_OWNED = {
    "state_dir": "scensim's: <project>/state",
    "gap_trace_dir": "logging.gap_trace: true in testbench.yaml (each run's trace in "
                     "runs/<id>/gap_trace/)",
    "contact_report_dir": "world.contact_report_dir in testbench.yaml (unset: each run's in "
                          "runs/<id>/contacts/)",
    "world_endpoint": "scensim's: the seat's world server port",
    "ssv2_port": "scensim's: the seat's SSv2 port",
    "agent_port": "scensim's: the seat's relay port",
}
# the world server's launch arguments scensim gives itself
SIM_OWNED = {
    "carla": {"state_dir": "scensim's: <project>/state", "port": "scensim's: the seat's world port",
              "ssv2_port": "scensim's: the seat's SSv2 port",
              "carla_host": "the host settings' carla.host", "carla_port": "the host "
              "settings' carla.port (or the seat table's carla:)"},
    "isaac": {"state_dir": "scensim's: <project>/state",
              "world": "maps: in testbench.yaml and map:= (preload: in isaac:)"},
}


def _number(v) -> bool:
    return not isinstance(v, bool) and isinstance(v, (int, float))


def weather_problems(w, where="world.weather") -> list:
    if w is None:
        return []
    if isinstance(w, str):
        return [] if w.strip() else [(where, "is empty: a preset name, e.g. ClearNoon")]
    if not isinstance(w, dict) or not w:
        return [(where, "is neither a preset name nor a mapping of parameters")]
    out = []
    for k, v in w.items():
        if k == "preset":
            if not (isinstance(v, str) and v.strip()):
                out.append((f"{where}.preset", "is not a preset name"))
        elif not _number(v):
            out.append((f"{where}.{k}", f"{v!r} is not a number (a weather parameter of the"
                        " world server, e.g. cloudiness: 30)"))
    return out


def problems(settings: dict, simulator: str, project: Path = None) -> list:
    """``(where, message)`` for ``world:`` and ``isaac:``; files are checked when ``project``
    is given."""
    out = []
    w = settings.get("world")
    if w is not None and not isinstance(w, dict):
        out.append(("world", "is not a mapping: {weather: ClearNoon}"))
        w = None
    for k, v in (w or {}).items():
        if k not in WORLD_KEYS:
            why = SIDE_OWNED.get(k)
            out.append(("world", f"{k} is scensim's ({why})" if why else
                        f"unknown key {k} (use only {', '.join(WORLD_KEYS)})"))
        elif k == "weather" and simulator == "isaac":
            out.append(("world.weather", "Isaac has no weather service: a scenario's "
                        "EnvironmentAction sets the sky, fog and rain"))
        elif k == "weather":
            out += weather_problems(v)
        elif k == "npc_motion" and v not in NPC_MOTIONS:
            out.append(("world.npc_motion", f"{v!r} is not one of {', '.join(NPC_MOTIONS)}"))
        elif k == "contact_report_dir":
            if v is False:
                continue  # YAML's bare `off`
            if not (isinstance(v, str) and v.strip()):
                out.append(("world.contact_report_dir", "is not off or a directory"))
        elif k in ("config_file", "world_config_file"):
            if not (isinstance(v, str) and v.strip()):
                out.append((f"world.{k}", "is not a file name"))
            elif project is not None and not _path(project, v).is_file():
                out.append((f"world.{k}", f"{v}: no such file"))
            elif k == "world_config_file" and simulator and simulator != "carla":
                out.append(("world.world_config_file", "is CARLA's world server's (scensim_carla_"
                            f"world.yaml); the simulator is {simulator}"))
    i = settings.get("isaac")
    if i is None:
        return out
    if simulator and simulator != "isaac":
        return out + [("isaac", f"is for simulator: isaac, this project's is {simulator}")]
    if not isinstance(i, dict):
        return out + [("isaac", "is not a mapping: {render: headless}")]
    for k, v in i.items():
        if k not in ISAAC_KEYS:
            why = SIM_OWNED["isaac"].get(k)
            out.append(("isaac", f"{k} is scensim's ({why})" if why else
                        f"unknown key {k} (use only {', '.join(ISAAC_KEYS)})"))
        elif k == "ego":
            if not (isinstance(v, str) and v.strip()):
                out.append(("isaac.ego", "is not default, none or an ego config file"))
            elif v not in ("default", "none") and project is not None \
                    and not _path(project, v).is_file():
                out.append(("isaac.ego", f"{v}: no such file"))
        elif k == "render" and v not in RENDERS:
            out.append(("isaac.render", f"{v!r} is not one of {', '.join(RENDERS)}"))
        elif k == "follow" and _flag(v) not in FOLLOWS:
            out.append(("isaac.follow", f"{v!r} is not one of {', '.join(FOLLOWS)}"))
        elif k == "physics_hz" and not (_number(v) and v > 0):
            out.append(("isaac.physics_hz", f"{v!r} is not a positive number"))
        elif k == "wheels_topic" and not isinstance(v, bool):
            out.append(("isaac.wheels_topic", f"{v!r} is not true or false"))
        elif k == "preload":
            if not (isinstance(v, list) and all(isinstance(m, str) and m for m in v)):
                out.append(("isaac.preload", "is not a list of map names"))
            elif project is not None:
                for m in v:
                    if not (project / "assets" / "maps" / m).is_dir():
                        out.append(("isaac.preload", f"{m} is not fetched (assets/maps/{m})"))
    return out


def _flag(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    return str(v)


def _path(project: Path, p) -> Path:
    q = Path(str(p)).expanduser()
    return q if q.is_absolute() else (Path(project) / q)


def contacts_mode(world: dict):
    """``None`` when scensim collects the contact reports per run, else the user's value (``off``
    or a directory)."""
    v = (world or {}).get("contact_report_dir")
    if v is None:
        return None
    if v is False:
        return CONTACTS_OFF
    return str(v).strip()


def weather_preset(world: dict):
    w = (world or {}).get("weather")
    if isinstance(w, str):
        return w.strip()
    if isinstance(w, dict) and isinstance(w.get("preset"), str):
        return w["preset"].strip()
    return None


def weather_parameters(world: dict) -> dict:
    w = (world or {}).get("weather")
    if not isinstance(w, dict):
        return {}
    return {k: float(v) for k, v in w.items() if k != "preset"}


def side_args(project: Path, world: dict) -> dict:
    """simulation.launch.xml arguments from ``world:`` (the user's, beside scensim's own)."""
    world = world or {}
    out = {}
    preset = weather_preset(world)
    if preset:
        out["weather"] = preset
    if world.get("npc_motion"):
        out["npc_motion"] = world["npc_motion"]
    if world.get("config_file"):
        out["config_file"] = str(_path(project, world["config_file"]).resolve())
    mode = contacts_mode(world)
    if mode is not None:
        out["contact_report_dir"] = mode if mode == CONTACTS_OFF else \
            str(_path(project, mode).resolve())
    return out


def sim_args(project: Path, simulator: str, world: dict, isaac: dict) -> dict:
    """The world server's launch arguments from ``world:`` and ``isaac:``."""
    out = {}
    if simulator == "carla" and (world or {}).get("world_config_file"):
        out["config_file"] = str(_path(project, world["world_config_file"]).resolve())
    if simulator == "isaac":
        i = isaac or {}
        if i.get("ego"):
            e = str(i["ego"])
            out["ego"] = e if e in ("default", "none") else str(_path(project, e).resolve())
        for k in ("render", "physics_hz"):
            if i.get(k) is not None:
                out[k] = str(i[k])
        if i.get("follow") is not None:
            out["follow"] = _flag(i["follow"])
        if i.get("wheels_topic") is not None:
            out["wheels_topic"] = _flag(i["wheels_topic"])
    return out


SET_WEATHER = "/scenario_world_bridge/set_weather"


def weather_argv(params: dict) -> list:
    """``ros2 service call`` of the bridge's set_weather with ``params`` (name -> value)."""
    names = list(params)
    req = "{names: [" + ", ".join(names) + "], values: [" + \
        ", ".join(repr(float(params[n])) for n in names) + "]}"
    return ["ros2", "service", "call", SET_WEATHER, "scensim_interfaces/srv/SetWeather", req]


def weather_answer(text: str):
    """``(ok, message)`` from ``ros2 service call``'s output: ok True or False when the
    bridge answered, None when it did not (no response printed)."""
    import re
    m = re.search(r"success=(True|False)", text or "")
    if not m:
        return None, ""
    msg = re.search(r"message=(['\"])(.*?)\1", text or "")
    return m.group(1) == "True", msg.group(2) if msg else ""


def named(text, what) -> dict:
    """``name:=value ...`` as a dict (stack_args' side_args and sim_args)."""
    out = {}
    for tok in shlex.split(text or ""):
        k, sep, v = tok.partition(":=")
        if not sep or not k:
            raise ValueError(f"{what}: '{tok}' is not name:=value")
        out[k] = v
    return out


MAP_PROPS_DEFAULT = "frozen"


def carla_world_settings(rec: dict, share=None) -> dict:
    """The CARLA world server's settings that change results, for a run's provenance:
    ``{"map_props": ...}`` (roadmap 077 G58) from the config file the stack gave it
    (``config_file:=`` in its sim_args), else the installed ``scensim_carla_world.yaml``
    (``share``, found by ament when None). Empty for another simulator; ``None`` values when
    the file cannot be read."""
    if (rec or {}).get("simulator") != "carla":
        return {}
    try:
        path = named((rec.get("launch_args") or {}).get("sim_args", ""), "sim_args") \
            .get("config_file")
    except ValueError:
        path = None
    if not path:
        if share is None:
            try:
                from ament_index_python.packages import get_package_share_directory
                share = get_package_share_directory("scensim_carla_world")
            except Exception:
                return {"map_props": None}
        path = Path(share) / "config" / "scensim_carla_world.yaml"
    try:
        import yaml
        doc = yaml.safe_load(Path(path).read_text()) or {}
    except Exception:  # unreadable or malformed: the server refused it too
        return {"map_props": None}
    value = doc.get("map_props", MAP_PROPS_DEFAULT) if isinstance(doc, dict) else None
    return {"map_props": value}


def joined(args: dict) -> str:
    return " ".join(shlex.quote(f"{k}:={v}") for k, v in args.items())


def merge_user(scensim: dict, user: dict, owned: dict, what: str) -> dict:
    """scensim's arguments, then the user's; a key scensim owns raises ValueError naming what to
    give instead."""
    bad = [k for k in user if k in owned]
    if bad:
        raise ValueError("; ".join(f"{what}: {k}:= is scensim's ({owned[k]})" for k in bad))
    return {**scensim, **user}
