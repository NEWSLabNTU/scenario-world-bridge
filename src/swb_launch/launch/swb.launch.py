"""The stack of a project (roadmap 076, docs/design/projects-and-pilots.md "The stack"):
stack.launch.py with the project's settings applied.

    cd my_project
    play_launch launch --web-addr 127.0.0.1:<seat web port> --log-dir <dir> \\
        swb_launch swb.launch.xml [project:=<dir>|pkg://<package>] [seat:=<n>|auto] \\
        [simulator:=carla|isaac] [simulator_start:=auto|start|external] [map:=<map>] \\
        [logs:=<dir>] [gpu:=<n>] [stack_args:="name:=value ..."]

The logic is swb.stack (importable, tested): it finds the project (project:=, else the
working directory or the nearest parent with swb.yaml), reads the host settings, picks the
seat (refusing one another stack holds), runs `project check` (refusing on any problem),
binds the project's pilots to the entities its scenarios drive from outside, each in a pilot
domain of the seat, and sets SWB_MAPS, SWB_CARLA_USD and the DDS profile for every process.

What it starts: a keeper (`python3 -m swb.stack keep`, required) that holds the seat's lock
while the stack is up, writes <logs>/<project>/stacks/<id>/stack.json and links
<logs>/<project>/seat-<n>-current to it, runs the stack's pilots (nested play_launches,
respawned) on the stack's map and moves them to another map when a run asks
(switch-request.json in the stack's directory), and marks the stack ready once every stack
pilot is; and stack.launch.py with the seat's domains and ports, the world server and the
bridge nested (nest_side:=true) so that every stack process logs under
stacks/<id>/proc/<group>/, where a run cuts its slices. stack.launch.py starts no pilot here.

The stack serves several maps (maps: in swb.yaml, else every map the project's scenarios
name). On CARLA its pilots start for the first run's map unless map:= names one; on Isaac
the world server loads its world at start, so the first map is the stack's.

play_launch binds its web UI and opens its log directory before this file is read, so those
two are the command line's: give --web-addr the seat's web port (`ros2 run swb seat show`;
this launch and the keeper print it) and --log-dir somewhere outside the project if you mind
play_log/ there.

CARLA (swb.simulator): simulator_start:=auto (the default, else the host settings'
carla.start) starts CARLA from carla.root when nothing answers on carla.host:carla.port and
uses the one that answers otherwise; start always starts it (refused when the port is held);
external never starts or stops it. A port held by something that is not CARLA's RPC server is
refused. A started CARLA belongs to the keeper: logged to proc/simulator/carla.log, waited for
(carla.ready_timeout, 300 s) before the pilots start and the stack is ready, and stopped with
the stack. Isaac's world server starts Isaac. stack_args go to stack.launch.py last and win;
their side_args and sim_args are merged by name with what swb and the project's world: and
isaac: give (swb.world), and a name swb sets itself (state_dir, gap_trace_dir,
contact_report_dir, the ports, Isaac's world) is refused with what to give instead.

The old stack's arguments are refused by name, given here or in stack_args, with what replaced
them (swb.stack.RETIRED_ARGS, roadmap 076 step 10): ego_launch, ego_launch_args and agent (the
project's pilots in swb.yaml) and map_path (map:=).
"""
import os
import shlex
import sys

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, Shutdown)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ARGS = {
    "project": ("", "the project: a directory or pkg://<package>; empty: the working directory's"),
    "seat": ("", "a seat number or auto; empty: the host settings' seat:"),
    "simulator": ("", "carla or isaac; empty: the project's simulator:"),
    "simulator_start": ("", "auto, start or external: whether the stack starts CARLA itself; empty: the host settings' carla.start (auto)"),
    "map": ("", "the map the stack's pilots start on; empty: the first run's (CARLA), the first the project's scenarios name (Isaac)"),
    "logs": ("", "the log root; empty: the host settings' logs:"),
    "gpu": ("", "the GPU index; empty: the seat's"),
    "launcher": ("", "auto, play_launch or ros2: the launcher of the stack's nested launches; empty: the host settings' launcher: (auto)"),
    "stack_args": ("", "name:=value ... for stack.launch.py, after swb's own (they win); side_args and sim_args merge by name with swb's, whose own names are refused"),
}


def _stack_module():
    try:
        from swb import stack
    except ImportError:  # a source tree without the swb package installed
        here = os.path.dirname(os.path.abspath(__file__))
        sys.path.insert(0, os.path.join(here, "..", "..", "swb"))
        from swb import stack
    return stack


def named_args(text):
    out = {}
    for tok in shlex.split(text or ""):
        name, sep, value = tok.partition(":=")
        if not sep or not name:
            raise RuntimeError(f"swb: stack_args: '{tok}' is not name:=value")
        out[name] = value
    return out


def _refused(why):
    """The refusal on stderr first, as one plain line, then raised: play_launch wraps a launch
    file's exception in its parser's error and a `--parser python` hint, which a reader who
    started play_launch directly saw instead of the reason (roadmap 077, G68)."""
    msg = f"swb: the stack is refused (3): {why}"
    print(msg, file=sys.stderr, flush=True)
    raise RuntimeError(msg) from None


KEEPER_STOP_TIMEOUT = "90"  # s: longer than the keeper's own stop (pilots, then CARLA)


def _swb(context):
    get = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    stack = _stack_module()
    # the old stack's arguments (carla.launch.xml's ego_launch, ...), which a launch would
    # drop without a word: refused by name, with what replaced them (roadmap 076 step 10)
    retired = stack.retired_args(n for n in context.launch_configurations if n not in ARGS)
    if retired:
        _refused("; ".join(retired))
    try:
        plan = stack.plan(project_arg=get("project") or None, seat_arg=get("seat") or None,
                          simulator=get("simulator") or None, map_name=get("map") or None,
                          logs=get("logs") or None, gpu=get("gpu") or None,
                          simulator_start=get("simulator_start") or None,
                          launcher=get("launcher") or None,
                          stack_args=named_args(get("stack_args")))
    except stack.StackRefused as e:
        _refused(e)
    stack.write_plan(plan)
    here = os.path.dirname(os.path.abspath(__file__))
    on = plan.map or "the first run's map"
    print(f"swb: project {plan.project} on seat {plan.seat.index} ({plan.simulator}, maps "
          f"{', '.join(plan.record['maps'])}; pilots on {on}); stack {plan.stack_dir}; web UI "
          f"port {plan.seat.web_port}", flush=True)
    actions = [SetEnvironmentVariable(k, v) for k, v in plan.env.items()]
    actions.append(ExecuteProcess(cmd=["python3", "-m", "swb.stack", "keep", str(plan.stack_dir)],
                                  name="stack_keeper", output="screen", on_exit=Shutdown(),
                                  # its stop: the pilots (up to 35 s: SIGINT, 20 s, SIGTERM,
                                  # 10 s, SIGKILL, 5 s), then the stack's CARLA (as long
                                  # again); a SIGTERM before that ends is a SIGKILL 5 s later
                                  # and leaves CARLA holding its port (roadmap 079, step 4).
                                  # play_launch ignores it (its own 2 s + 5 s); swb's own stop
                                  # asks the keeper first (swb.run.StartedStack.stop)
                                  sigterm_timeout=KEEPER_STOP_TIMEOUT))
    actions.append(IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(here, "stack.launch.py")),
        launch_arguments=list(plan.launch_args.items())))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=d, description=t) for k, (d, t) in ARGS.items()]
        + [OpaqueFunction(function=_swb)])
