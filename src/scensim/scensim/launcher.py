"""One builder for every launcher command line (roadmap 079, step 2).

scensim starts a launch file as a process of its own in four places: the stack itself (`run
--start-stack`), each nested launch of the stack (the world server, a pilot, SSv2's
scenario), a run's own pilots and a run's SSv2. Each goes through `argv`, for either
launcher (`launcher: auto | play_launch | ros2`, the host setting, resolved by
`scensim.tools.launcher`):

- play_launch: `play_launch launch [--web-addr <ip:port> | --disable-web-ui]
  --disable-diagnostics --log-dir <dir> [play_launch-only flags] <target> <args>`;
- ros2: `ros2 launch <target> <args>`, with ROS_LOG_DIR=<dir> in its environment (its nodes
  then write their logs there; step 4 reads that layout).

play_launch-only flags (the web UI, diagnostics, `--container-mode isolated
--composable-respawn on-crash` for Autoware's containers) go to play_launch alone.
"""
LAUNCHERS = ("play_launch", "ros2")

# Autoware's composable containers under play_launch: a crashed component restarts in its
# own process instead of taking its container down (roadmap 079, step 5 decides ros2's)
ISOLATED_CONTAINERS = ("--container-mode", "isolated", "--composable-respawn", "on-crash")


def argv(launcher, target, args=None, log_dir=None, web_port=None, web_host="127.0.0.1",
         play_launch_flags=(), diagnostics=False):
    """``(argv, env)``: the command line that launches ``target`` (``[package, file]`` or
    ``[path]``) with ``args`` (``{name: value}``, or ``name:=value`` words), logging under
    ``log_dir``, and the environment it needs beyond the caller's. ``web_port``: play_launch's
    web UI there (None: none); ``play_launch_flags``: flags only play_launch takes;
    ``diagnostics``: play_launch's /diagnostics recording (off for nested launches, G59)."""
    if launcher not in LAUNCHERS:
        raise ValueError(f"launcher {launcher!r}: one of {', '.join(LAUNCHERS)}")
    words = _words(args)
    env = {}
    if launcher == "play_launch":
        cmd = ["play_launch", "launch"]
        cmd += ["--web-addr", f"{web_host}:{web_port}"] if web_port else ["--disable-web-ui"]
        if not diagnostics:
            cmd.append("--disable-diagnostics")  # read by nothing, unbounded (077 G59)
        if log_dir is not None:
            cmd += ["--log-dir", str(log_dir)]
        cmd += list(play_launch_flags)
    else:
        cmd = ["ros2", "launch"]
        if log_dir is not None:
            env["ROS_LOG_DIR"] = str(log_dir)
    return cmd + [str(t) for t in target] + words, env


def _words(args):
    if args is None:
        return []
    if isinstance(args, dict):
        out = []
        for k, v in args.items():
            out.append(f"{k}:={_text(v)}")
        return out
    return [str(a) for a in args]


def _text(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return "" if value is None else str(value)


def of(record) -> str:
    """The launcher a stack record (stack.json) names; play_launch for a record written
    before roadmap 079."""
    name = (record or {}).get("launcher") or "play_launch"
    return name if name in LAUNCHERS else "play_launch"


def command(launcher, target, args=None, log_dir=None, **kw):
    """`argv` as one command line: its environment, if any, as an `env NAME=value` prefix
    (for callers that start a plain argv: ExecuteProcess, subprocess)."""
    cmd, env = argv(launcher, target, args, log_dir, **kw)
    return (["env"] + [f"{k}={v}" for k, v in env.items()] + cmd) if env else cmd
