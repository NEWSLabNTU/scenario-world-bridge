"""The whole stack in one launch (roadmap 073): a simulator, the simulation side, the user's
autopilot, and optionally an SSv2 scenario. scensim names no car, scenario or use case here: each
is a slot.

    play_launch launch scensim_launch stack.launch.py sim:=isaac|carla|<package> \\
        [sim_args:="name:=value ..."] [ego_launch:=<path>|"<package> <file>"|none] \\
        [ego_launch_args:="name:=value ..."] [vehicle_model:=...] [spec_hash_file:=...] \\
        [vehicle_catalogs:=...] [scenario:=<path to .xosc>] [scenario_args:="..."] \\
        [exit_on_scenario_end:=false] [pilots:='<YAML or JSON list>']

The shortcuts for the supported simulators, isaac.launch.xml and carla.launch.xml, fill
`sim` and `sim_args` from arguments of their own. Those and the `ego_launch` path below are
the developers' (`just run`); a project's stack is scensim.launch.xml, whose keeper runs its pilots
and which refuses ego_launch, ego_launch_args, agent and map_path by name (roadmap 076 step
10). Another simulator here takes `pilots:` (no default autopilot).

Groups and their ROS domains:
- sim (ego domain): the world server.
  - isaac: scensim_isaac_world.launch.xml.
  - carla: scensim_carla_world.launch.xml; CARLA itself runs apart (CarlaUE4.sh), or, under
    scensim.launch.xml, is started by its keeper (scensim.simulator, `simulator_start`).
  - <package>: that package's launch/world.launch.xml, which starts a world server speaking
    scensim_world.proto (docs/design/simulator-interface-split.md; scensim_world_check conformance)
    on `world_endpoint`.
  `sim_args` go to that launch.
- simulation side (scenario domain): the bridge and the agent relay (simulation.launch.xml).
  `side_args` go to that launch (config_file, weather, gap_trace_dir, npc_motion, state_dir,
  ...).
- ego (ego domain): the autopilot's launch and scensim's vehicle agent (vehicle_agent.launch.xml),
  as a nested play_launch, respawned. An autopilot stack ends with its episode (a world load,
  a simulator restart: acb_bridge is required), and that must not end the stack; the
  respawn brings it back.
  - Default ego_launch: on isaac, aib_launch isaac_simulator.launch.xml (aib's Autoware
    profile; the vehicle interface and sensors are the world server's); on carla,
    acb_launch carla_simulator.launch.xml (Autoware with acb_bridge, CARLA's vehicle
    bridge); another simulator has none, so no ego starts (give `pilots`). none: no ego.
- pilots (roadmap 076, docs/design/projects-and-pilots.md "Pilots"): `pilots` is YAML or JSON
  text, a list `[{entity, kind, domain, <arg>: <value>, args: {...}}, ...]` (or a mapping
  entity -> {kind, ...}). Each pilot is a nested play_launch of scensim_launch's
  launch/pilots/<kind's file> in its own domain, respawned: autoware (autoware.launch.xml),
  carla_basic (carla_basic.launch.py), simple (simple.launch.py), launch (launch.launch.py),
  command (command.launch.py); none starts nothing (the user's own). Keys other than entity, kind, domain, lifetime and args
  are launch arguments too (a list is joined as shell words, so `run: [python3, -m, x]`
  works); `args` wins. Every pilot gets the common contract: entity, relay, map_path, vehicle
  (vehicle_model, else tesla_model3), sim, use_sim_time, spec_hash_file, and on CARLA
  carla_host, carla_port, the release endpoints and the physics check. A pilot without
  `domain` takes ego_domain if free, then the next free one above it. On Isaac the car's rig
  is the world server's, on ego_domain: a pilot with a vehicle side must run there (one such
  pilot), and carla_basic is refused; a simple pilot there drives on aib's ground truth, which
  the ego config given in sim_args (ego:=<yaml> with ground_truth on) must turn on (scensim.stack
  writes one). With pilots and no ego_launch the old ego is not
  started; ego_launch:=<launch> still starts it, in ego_domain, beside the pilots (the
  developers' path).
  scensim.launch.xml passes no pilots here: its keeper (scensim.stack) runs them, so that it can
  restart them with another map_path when a run is on another map.
- run (scenario domain): with `spawn` and no scenario, free drive (scensim_free_drive: the ego at
  that pose, time stepped synchronously, paced to the wall clock unless realtime:=false; the
  autopilot driven by hand). With `scenario`, SSv2 (scenario.launch.xml) as a nested play_launch:
  it shuts its own launch down when the scenario ends, which must not end the stack. With
  exit_on_scenario_end:=true the stack ends with it (scripted runs). Run the next scenario
  against the live stack with `play_launch launch scensim_launch scenario.launch.xml
  scenario:=<path>` in the scenario domain.

Time is synchronous with a scenario: SSv2 steps the world frame by frame through the bridge
(CARLA in synchronous mode, Isaac's world server stepped), at global_real_time_factor
(scenario_args), as fast as the stack keeps up.

Host settings are arguments (scensim.launch.xml fills them from the project's seat), else the
environment's, as for `just`: ego_domain (EGO_ROS_DOMAIN_ID, 1), scenario_domain
(SCENARIO_ROS_DOMAIN_ID, 9), ssv2_port (SSV2_PORT, 5555), agent_port (AGENT_PORT, 5560),
world_port (ISAAC_PORT 5590 / WORLD_PORT 5591), carla_host and carla_port (CARLA_HOST,
CARLA_PORT), log_dir (SCENSIM_LOG_ROOT, the nested launches' logs, default ./play_log);
CYCLONEDDS_URI (default scensim_launch's localhost profile), RMW_IMPLEMENTATION.
"""
import os
import sys
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable,
                            Shutdown)
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


# scensim_launch's localhost Cyclone profile sets DomainGain 1000: a domain's ports start at
# 7400 + 1000 * id, so 58 and up need ports past 65535 and every node dies at start with
# "failed to create domain" (roadmap 070, friction 15). Fast DDS (an unset RMW) allows 232.
MAX_DOMAIN = 57


def check_domains(ego, scenario):
    """Refuse domains the stack cannot run in, before anything starts."""
    ids = {}
    for name, text in (("ego_domain (EGO_ROS_DOMAIN_ID)", ego),
                       ("scenario_domain (SCENARIO_ROS_DOMAIN_ID)", scenario)):
        try:
            ids[name] = int(text)
        except ValueError:
            raise RuntimeError(f"stack: {name} is {text!r}, not a number") from None
        if not 0 <= ids[name] <= MAX_DOMAIN:
            raise RuntimeError(
                f"stack: {name} is {ids[name]}: use 0-{MAX_DOMAIN} (the localhost Cyclone "
                "profile's DomainGain 1000 puts higher domains' ports past 65535)")
    if len(set(ids.values())) == 1:
        raise RuntimeError(
            f"stack: ego_domain and scenario_domain are both {ego}: the autopilot's ADAPI and "
            "the relay's would meet; give them different domains")


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


def _launcher_module():
    try:
        from scensim import launcher
    except ImportError:  # a source tree without the scensim package installed
        here = os.path.dirname(os.path.abspath(__file__))
        sys.path.insert(0, os.path.join(here, "..", "..", "scensim"))
        from scensim import launcher
    return launcher


# the launcher of the nested launches, `launcher:=` (set by _stack before any nested())
# The launcher of every nested launch: set by _stack from `launcher:=` before it builds any
# action, so nested() must not be called before _stack runs (one stack per process).
LAUNCHER = {"name": "play_launch"}
NESTED_STOP_TIMEOUT = "30"  # s, a nested launch's whole stop before its SIGTERM


def nested(name, target, args, domain, log_dir, respawn=False, extra=(), required=False,
           env=None):
    """A launch as its own launcher process (play_launch or `ros2 launch`, roadmap 079): its
    own lifetime, its own log directory. Built by scensim.launcher, which gives play_launch alone
    its flags (`extra`; no /diagnostics recording, G59) and `ros2 launch` its ROS_LOG_DIR."""
    cmd, launcher_env = _launcher_module().argv(LAUNCHER["name"], target, args, log_dir,
                                                play_launch_flags=extra)
    if LAUNCHER["name"] == "ros2" and log_dir:
        # `ros2 launch` writes its nodes' screen to its own stdout, which would otherwise land
        # in the stack's console: into <log_dir>/console.log instead, beside its launch.log and
        # rcl files, as play_launch keeps a launch's output in its --log-dir (roadmap 079 step
        # 4: the readers take each process's lines from that console). exec: same pid, so the
        # launch's signals reach `ros2 launch` itself. env --default-signal: a stack started
        # in the background of a script (`ros2 launch scensim_launch scensim.launch.xml &`) has
        # SIGINT ignored, inherited by every launch below it; `ros2 launch` then ignores the
        # group's SIGINT while its rclpy nodes (which set their own handler) die on it and are
        # respawned, outliving the stack (roadmap 079 exit criterion 1, F1).
        # --default-signal needs coreutils 8.31 or later; Humble's Ubuntu 22.04 has 8.32
        os.makedirs(str(log_dir), exist_ok=True)
        cmd = ["sh", "-c", 'exec "$@" >> "$0" 2>&1',
               os.path.join(str(log_dir), "console.log"),
               "env", "--default-signal=INT,TERM,QUIT", *cmd]
    return ExecuteProcess(cmd=cmd, name=name, output="screen", respawn=respawn,
                          respawn_delay=5.0 if respawn else 0.0,
                          # longer than the nested launch's own stop (SIGINT to its nodes,
                          # SIGTERM 5 s later, then SIGKILL): a SIGTERM to it earlier leaves
                          # its nodes running without a launch (roadmap 079, step 3)
                          sigterm_timeout=NESTED_STOP_TIMEOUT,
                          additional_env={**(env or {}), **launcher_env,
                                          "ROS_DOMAIN_ID": str(domain)},
                          on_exit=Shutdown() if required else None)


# --- pilots (roadmap 076 step 5) ------------------------------------------------------------
# kind -> its launch under scensim_launch's launch/pilots/; None: nothing to start
PILOT_KINDS = {"autoware": "autoware.launch.xml", "carla_basic": "carla_basic.launch.py",
               "simple": "simple.launch.py", "launch": "launch.launch.py",
               "command": "command.launch.py", "none": None}
PILOT_KEYS = ("entity", "kind", "domain", "lifetime", "args")


def _arg_text(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        return shlex.join(str(v) for v in value)
    if value is None:
        return ""
    return str(value)


def has_vehicle_side(kind, args):
    """Whether the pilot starts a simulator vehicle side in its domain."""
    if kind in ("autoware", "simple"):
        return True
    if kind == "launch":
        return args.get("vehicle_side", "true") == "true"
    if kind == "command":
        return args.get("vehicle_side", "false") == "true"
    return False


def parse_pilots(text, sim, ego_domain, scenario_domain, ego_taken=False):
    """`pilots:` as [{entity, kind, domain, args}], domains assigned and checked.

    ego_taken: the old ego (ego_launch) holds ego_domain."""
    import yaml
    if not (text or "").strip():
        return []
    try:
        raw = yaml.safe_load(text)
    except yaml.YAMLError as e:
        raise RuntimeError(f"stack: pilots: not YAML or JSON: {e}") from None
    if raw is None:
        return []
    if isinstance(raw, dict):
        raw = [{"entity": name, **(spec or {})} for name, spec in raw.items()]
    if not isinstance(raw, list):
        raise RuntimeError("stack: pilots: a list of {entity, kind, ...} (or entity -> {kind, ...})")
    pilots, entities = [], set()
    for i, item in enumerate(raw):
        if not isinstance(item, dict):
            raise RuntimeError(f"stack: pilots[{i}]: a mapping, not {item!r}")
        entity = _arg_text(item.get("entity"))
        if not entity:
            raise RuntimeError(f"stack: pilots[{i}]: no entity")
        if entity in entities:
            raise RuntimeError(f"stack: pilots: entity {entity!r} twice")
        entities.add(entity)
        kind = item.get("kind")
        if kind not in PILOT_KINDS:
            raise RuntimeError(f"stack: pilot {entity}: kind {kind!r}: one of "
                               + ", ".join(PILOT_KINDS))
        lifetime = item.get("lifetime", "stack")
        if lifetime != "stack":
            raise RuntimeError(f"stack: pilot {entity}: lifetime {lifetime!r}: the stack starts "
                               "lifetime stack pilots only (lifetime run is the run's)")
        extra = item.get("args") or {}
        if not isinstance(extra, dict):
            raise RuntimeError(f"stack: pilot {entity}: args is a mapping of launch arguments")
        args = {k: _arg_text(v) for k, v in item.items() if k not in PILOT_KEYS}
        args.update({k: _arg_text(v) for k, v in extra.items()})
        if kind == "carla_basic" and sim != "carla":
            raise RuntimeError(f"stack: pilot {entity}: carla_basic drives CARLA only, sim is {sim!r}")
        if kind == "launch" and not args.get("launch"):
            raise RuntimeError(f"stack: pilot {entity}: kind launch needs launch: '<pkg> <file>' or a path")
        if kind == "command" and not args.get("run"):
            raise RuntimeError(f"stack: pilot {entity}: kind command needs run: [argv...]")
        domain = item.get("domain")
        pilots.append({"entity": entity, "kind": kind, "args": args,
                       "domain": None if domain in (None, "") else _arg_text(domain)})

    used = {int(scenario_domain)}
    if ego_taken:
        used.add(int(ego_domain))
    for p in pilots:  # explicit domains first
        if p["domain"] is None or p["kind"] == "none":
            continue
        try:
            d = int(p["domain"])
        except ValueError:
            raise RuntimeError(f"stack: pilot {p['entity']}: domain {p['domain']!r} is not a number") from None
        if not 0 <= d <= MAX_DOMAIN:
            raise RuntimeError(f"stack: pilot {p['entity']}: domain {d}: use 0-{MAX_DOMAIN}")
        if d in used:
            raise RuntimeError(f"stack: pilot {p['entity']}: domain {d} is taken (the scenario's, "
                               "the old ego's or another pilot's)")
        used.add(d)
        p["domain"] = str(d)
    for p in pilots:  # then the rest: ego_domain, then upwards
        if p["domain"] is not None or p["kind"] == "none":
            continue
        d = int(ego_domain)
        while d in used:
            d += 1
        if d > MAX_DOMAIN:
            raise RuntimeError(f"stack: pilot {p['entity']}: no free domain up to {MAX_DOMAIN}")
        used.add(d)
        p["domain"] = str(d)
    if sim == "isaac":
        sided = [p for p in pilots if has_vehicle_side(p["kind"], p["args"])]
        if len(sided) > 1:
            raise RuntimeError("stack: on Isaac one pilot has a vehicle side (one ego): "
                               + ", ".join(p["entity"] for p in sided))
        for p in sided:
            if p["domain"] != str(ego_domain):
                raise RuntimeError(
                    f"stack: pilot {p['entity']}: on Isaac the car's rig is the world server's, "
                    f"on ego_domain {ego_domain}; this pilot is on {p['domain']}")
    return pilots


def pilot_launches(pilots, pilots_dir, common):
    """(name, launch path, arguments, domain, ros) per pilot to start. `common` is the contract
    the stack gives every pilot; the pilot's own arguments override it. Empty values are left
    out (the launch's default)."""
    out = []
    for p in pilots:
        name = PILOT_KINDS[p["kind"]]
        if name is None:
            continue
        args = {**common, "entity": p["entity"], **p["args"]}
        args = {k: v for k, v in args.items() if v != ""}
        ros = p["kind"] != "carla_basic"
        out.append((f"pilot_{p['entity']}", os.path.join(pilots_dir, name), args, p["domain"], ros))
    return out


def spec_physics(value, vehicle_model, spec_hash_file, share=None):
    """The world server's vehicle_physics on CARLA: `spec` (carla.launch.xml's default) is the
    spec car's generated carla_physics.yaml, from sim_vehicle_spec, so that every car of the
    ego's model runs the spec, the vehicle agents' too (without it they kept the blueprint's
    physics while the ego ran the spec's; roadmap 074, the Y merge). A path or none is kept."""
    if value != "spec":
        return value
    if spec_hash_file:
        raise RuntimeError("stack: vehicle_physics spec names sim_vehicle_spec's cars; for a car "
                           "of another repository (spec_hash_file:=) give "
                           "vehicle_physics:=<its carla_physics.yaml>, or none")
    car = vehicle_model or "tesla_model3"
    share = share or get_package_share_directory("sim_vehicle_spec")
    path = os.path.join(share, "generated", car, "carla_physics.yaml")
    if not os.path.isfile(path):
        raise RuntimeError(f"stack: vehicle_physics spec: no {path} (sim_vehicle_spec has no "
                           f"car {car!r}); give vehicle_physics:=<carla_physics.yaml>, or none")
    return path


def _stack(context):
    get = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    LAUNCHER["name"] = get("launcher") or "play_launch"
    if LAUNCHER["name"] not in ("play_launch", "ros2"):
        raise RuntimeError(f"stack: launcher:={LAUNCHER['name']}: play_launch or ros2")
    sim = get("sim")
    if not sim:
        raise RuntimeError("stack: sim:= isaac, carla or a package of yours with launch/world.launch.xml")
    ego_domain = get("ego_domain")
    scenario_domain = get("scenario_domain")
    check_domains(ego_domain, scenario_domain)
    ssv2_port, agent_port = get("ssv2_port"), get("agent_port")
    world_port = get("world_port")
    carla_host, carla_port = get("carla_host"), get("carla_port")
    nest_side = get("nest_side") == "true"
    log_root = os.path.abspath(get("log_dir"))
    sim_args = named_args(get("sim_args"), "sim_args")
    scensim = get_package_share_directory("scensim_launch")

    # --- sim
    endpoint = get("world_endpoint")
    if sim == "isaac":
        world = os.path.join(get_package_share_directory("scensim_isaac_world"), "launch",
                             "scensim_isaac_world.launch.xml")
        port = world_port or _env("ISAAC_PORT", "5590")
        endpoint = endpoint or f"tcp://127.0.0.1:{port}"
        # scensim_isaac_world.launch.xml reads both from the environment
        sim_env = {"EGO_ROS_DOMAIN_ID": ego_domain, "ISAAC_PORT": port}
        timeouts = {}
    elif sim == "carla":
        world = os.path.join(get_package_share_directory("scensim_carla_world"), "launch",
                             "scensim_carla_world.launch.xml")
        port = world_port or _env("WORLD_PORT", "5591")
        endpoint = endpoint or f"tcp://127.0.0.1:{port}"
        sim_args = {"port": port, "ssv2_port": ssv2_port,
                    "carla_host": carla_host, "carla_port": carla_port, **sim_args}
        if "vehicle_physics" in sim_args:
            sim_args["vehicle_physics"] = spec_physics(sim_args["vehicle_physics"],
                                                       get("vehicle_model"), get("spec_hash_file"))
        sim_env = {}
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
        sim_env, timeouts = {}, {}
    if nest_side:
        # the world server as its own play_launch, logging under log_dir/simulator (the stack
        # of scensim.launch.xml: its runs cut slices from there)
        actions = [nested("simulator", [world], {k: v for k, v in sim_args.items() if v != ""},
                          ego_domain, os.path.join(log_root, "simulator"), respawn=True,
                          env=sim_env)]
    else:
        actions = [GroupAction(scoped=True, actions=[
            SetEnvironmentVariable("ROS_DOMAIN_ID", ego_domain),
            *[SetEnvironmentVariable(k, v) for k, v in sim_env.items()],
            include(world, sim_args)])]

    # --- simulation side
    side = {"world_endpoint": endpoint, "ssv2_port": ssv2_port, "agent_port": agent_port,
            "vehicle_catalogs": get("vehicle_catalogs"), **timeouts,
            **named_args(get("side_args"), "side_args")}
    side_launch = os.path.join(scensim, "launch", "simulation.launch.xml")
    if nest_side:
        actions.append(nested("bridge", [side_launch], {k: v for k, v in side.items() if v != ""},
                              scenario_domain, os.path.join(log_root, "bridge"), respawn=True))
    else:
        actions.append(GroupAction(scoped=True, actions=[
            SetEnvironmentVariable("ROS_DOMAIN_ID", scenario_domain), include(side_launch, side)]))

    # --- ego (the old path; the pilots below succeed it)
    ego_launch = get("ego_launch")
    pilots_text = get("pilots")
    if pilots_text.strip() and not ego_launch:
        ego_launch = "none"
    pilots = parse_pilots(pilots_text, sim, ego_domain, scenario_domain,
                          ego_taken=ego_launch != "none")
    if not ego_launch and sim not in PRESETS:
        # another simulator has no default autopilot: its vehicles come from `pilots`
        ego_launch = "none"
    if ego_launch != "none":
        if not ego_launch:
            ego_launch = {"isaac": "aib_launch isaac_simulator.launch.xml",
                          "carla": "acb_launch carla_simulator.launch.xml"}[sim]
            # the preset's Autoware: its map, and on CARLA the server it drives
            preset = {"map_path": get("map_path")}
            if not get("vehicle_model"):
                context.launch_configurations["vehicle_model"] = "tesla_model3"
            if sim == "carla":
                # the CARLA world server's sensor-release sockets follow the SSv2 port (+1, +2)
                preset.update(carla_host=carla_host, carla_port=carla_port,
                              vehicle_model=get("vehicle_model"),
                              release_notify_endpoint=f"tcp://127.0.0.1:{int(ssv2_port) + 1}",
                              release_ack_endpoint=f"tcp://127.0.0.1:{int(ssv2_port) + 2}")
                # The pedal maps belong to the physics: when the world server applies the
                # spec's physics, acb must drive with the spec's maps and only verify the
                # physics. Without this the ego ran the blueprint's maps on the spec's car and
                # overran Autoware's commands (1.2 m/s^2 for +0.1, to 30 m/s against a limit of
                # 8; roadmap 074 C3).
                physics = sim_args.get("vehicle_physics", "none")
                if physics and physics != "none":
                    preset.update(carla_physics=physics, carla_physics_mode="verify")
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
            actions.append(nested("vehicle_agent", [os.path.join(scensim, "launch",
                                                                 "vehicle_agent.launch.xml")],
                                  {k: v for k, v in agent.items() if v}, ego_domain,
                                  os.path.join(log_root, "agent"), respawn=True))

    # --- pilots
    if pilots:
        common = {"relay": f"tcp://localhost:{agent_port}", "map_path": get("map_path"),
                  "vehicle": get("vehicle_model") or "tesla_model3", "sim": sim,
                  "use_sim_time": "true", "spec_hash_file": get("spec_hash_file")}
        if sim == "carla":
            # as the old ego: the world server's release sockets (SSv2 port + 1, + 2), and the
            # spec's physics read back when the world server applies it (roadmap 074 C3)
            common.update(carla_host=carla_host, carla_port=carla_port,
                          release_notify_endpoint=f"tcp://127.0.0.1:{int(ssv2_port) + 1}",
                          release_ack_endpoint=f"tcp://127.0.0.1:{int(ssv2_port) + 2}")
            physics = sim_args.get("vehicle_physics", "none")
            if physics and physics != "none":
                common.update(carla_physics=physics, carla_physics_mode="verify")
        for name, path, args, domain, ros in pilot_launches(
                pilots, os.path.join(scensim, "launch", "pilots"), common):
            extra = ("--container-mode", "isolated", "--composable-respawn", "on-crash") if ros else ()
            actions.append(nested(name, [path], args, domain,
                                  os.path.join(log_root, name.replace("pilot_", "pilot-", 1)),
                                  respawn=True, extra=extra))

    # --- run
    scenario = get("scenario")
    if scenario:
        path = os.path.abspath(os.path.expanduser(scenario))
        if not os.path.isfile(path):
            raise RuntimeError(f"stack: no scenario {path}")
        run = {"scenario": path, "port": ssv2_port,
               **named_args(get("scenario_args"), "scenario_args")}
        # exit_on_scenario_end: the scenario's process is required (on_exit=Shutdown, which
        # play_launch honours as stock ros2 launch does)
        actions.append(nested("scenario", [os.path.join(scensim, "launch", "scenario.launch.xml")],
                              run, scenario_domain, os.path.join(log_root, "scenario"),
                              required=get("exit_on_scenario_end") == "true"))
    elif get("spawn"):
        # Free drive: scensim_free_drive in SSv2's place spawns the ego at `spawn` and steps the
        # world, synchronously, paced to the wall clock unless realtime:=false. Respawned:
        # it fails while the world server is still loading and simply starts over.
        map_path = get("map_path")
        if not map_path:
            raise RuntimeError("stack: spawn:= needs map_path:= (its lanelet2_map.osm)")
        catalogs = [c for c in get("vehicle_catalogs").replace(":", ",").split(",") if c]
        catalog = catalogs[0] if catalogs else os.path.join(
            get_package_share_directory("sim_vehicle_spec"), "catalogs", "vehicles.xosc")
        entry = get("vehicle") or get("vehicle_model") or "tesla_model3"
        actions.append(ExecuteProcess(
            cmd=["ros2", "run", "scenario_world_bridge", "scensim_free_drive",
                 "--map", map_path, "--catalog", catalog, "--entry", entry,
                 "--pose", get("spawn"), "--port", ssv2_port, "--name", get("entity"),
                 "--step", str(1.0 / float(get("frame_rate"))), "--realtime", get("realtime")],
            name="free_drive", output="screen", respawn=True, respawn_delay=5.0))
    return actions


def generate_launch_description():
    dds = os.environ.get("CYCLONEDDS_URI") or (
        "file://" + os.path.join(get_package_share_directory("scensim_launch"), "config",
                                 "cyclonedds-localhost.xml"))
    arg = DeclareLaunchArgument
    return LaunchDescription([
        arg("sim", default_value="", description="isaac, carla, or a package with launch/world.launch.xml"),
        arg("sim_args", default_value="", description="name:=value ... for the world server's launch"),
        arg("world_endpoint", default_value="", description="the world server's tcp://host:port; empty: the preset's"),
        arg("ego_launch", default_value="", description="the developers' autopilot launch (just run): a path or '<package> <file>'; empty: the sim's default (carla, isaac; another sim: none); none: no ego. A project gives pilots"),
        arg("ego_launch_args", default_value="", description="name:=value ... for ego_launch"),
        arg("agent", default_value="true", description="start scensim's vehicle agent beside the autopilot (false: ego_launch has its own)"),
        arg("entity", default_value="ego", description="the scenario entity the ego is"),
        arg("map_path", default_value="", description="the default autopilot's map directory (lanelet2_map.osm, pointcloud_map)"),
        arg("vehicle_model", default_value="", description="the agent's vehicle model (its description's spec_hash.yaml)"),
        arg("spec_hash_file", default_value="", description="the agent's spec_hash YAML, instead of vehicle_model's"),
        arg("vehicle_catalogs", default_value="", description="the bridge's extra vehicle catalogs, comma separated"),
        arg("side_args", default_value="", description="name:=value ... for the simulation side's launch (simulation.launch.xml: config_file, weather, gap_trace_dir, state_dir, ...)"),
        arg("pilots", default_value="", description="YAML/JSON list of pilots [{entity, kind, domain, <arg>: <value>, args: {...}}], each a nested play_launch of launch/pilots/<kind> in its domain; empty: none (the ego_launch path)"),
        arg("scenario", default_value="", description="an .xosc to run; empty: none"),
        arg("scenario_args", default_value="", description="name:=value ... for scenario.launch.xml (global_timeout, global_real_time_factor, output_directory, ...)"),
        arg("exit_on_scenario_end", default_value="false", description="true: the stack ends with the scenario"),
        arg("spawn", default_value="", description="free drive without a scenario: the ego at x,y,z,yaw in the map frame (needs map_path)"),
        arg("vehicle", default_value="", description="free drive: the ego's entry in the (first) vehicle catalog; empty: vehicle_model"),
        arg("realtime", default_value="true", description="free drive: pace the synchronous steps to the wall clock; false: as fast as the stack keeps up"),
        arg("frame_rate", default_value="20", description="free drive: steps per simulated second"),
        arg("ego_domain", default_value=_env("EGO_ROS_DOMAIN_ID", "1"), description="ROS domain of the simulator's world server and the ego (0-57); default EGO_ROS_DOMAIN_ID, else 1"),
        arg("scenario_domain", default_value=_env("SCENARIO_ROS_DOMAIN_ID", "9"), description="ROS domain of the bridge, relay and SSv2 (0-57, not ego_domain); default SCENARIO_ROS_DOMAIN_ID, else 9"),
        arg("ssv2_port", default_value=_env("SSV2_PORT", "5555"), description="SSv2's ZMQ port on the bridge; default SSV2_PORT, else 5555"),
        arg("agent_port", default_value=_env("AGENT_PORT", "5560"), description="the agent relay's TCP port; default AGENT_PORT, else 5560"),
        arg("world_port", default_value="", description="the world server's port; empty: WORLD_PORT (CARLA, else 5591) or ISAAC_PORT (Isaac, else 5590)"),
        arg("carla_host", default_value=_env("CARLA_HOST", ""), description="CARLA's RPC host; default CARLA_HOST, else the world server's own default"),
        arg("carla_port", default_value=_env("CARLA_PORT", ""), description="CARLA's RPC port; default CARLA_PORT, else the world server's own default"),
        arg("nest_side", default_value="false", description="true: the world server and the simulation side (bridge, relay) as nested play_launches logging under log_dir/simulator and log_dir/bridge (scensim.launch.xml's stack)"),
        arg("launcher", default_value="play_launch", description="play_launch or ros2: the launcher of every nested launch (roadmap 079); scensim.launch.xml's stack passes the host settings' resolved launcher"),
        arg("log_dir", default_value=_env("SCENSIM_LOG_ROOT", "play_log"), description="where the nested launches (ego, agent, scenario, pilots; with nest_side the world server and the bridge) log"),
        # an argument, not the environment: scensim.launch.py's SetEnvironmentVariable is not in
        # os.environ when this file is read, and a sourced Autoware exports its own profile
        arg("dds_config", default_value=dds, description="CYCLONEDDS_URI for every process of the stack; default the environment's, else scensim_launch's localhost profile"),
        SetEnvironmentVariable("CYCLONEDDS_URI", LaunchConfiguration("dds_config")),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", _env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")),
        OpaqueFunction(function=_stack),
    ])
