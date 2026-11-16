# 044: `render:=window` without a DISPLAY respawns the Isaac world server every 10 s

**Severity**: low (a usage error; nothing breaks, but the launch never ends and fills its log
directory). **Status**: fixed in swb_isaac_world `180a3f5`: every refusal the launch can check is made before the node starts, under both launchers. Under play_launch an exit 2 the launch file did not foresee still respawns, an upstream play_launch gap (below). **Found**: 2026-11-17, swb 241, roadmap 077 item 55
(`just isaac-world` options): swb 81268e3, swb_isaac_world 3043e47.

## What happens

```
$ env -u DISPLAY just isaac-world render:=window
swb_isaac_world_server: render window needs DISPLAY
ERROR [node:/swb_isaac_world] Exited with code 2
...
```

The world server's entry script refuses at once, exit 2, and the launch starts it again 10 s
later, without end. In 120 s the node's log directory held `out` through `out.11`: 12 starts,
each refused the same way. The launch was still running when the test's `timeout` stopped it.

## Why

The check is in the node (`scripts/swb_isaac_world_server.in`), and the node is
`respawn="true"` in `swb_isaac_world.launch.xml`, as a world server must be (issue 011, roadmap
077 G70). The launch cannot tell a refused configuration from a crash, so it treats the
refusal like any death of the world server. A configuration the node refuses will be refused
again on every respawn. Worse, the world server's own refusals before Kit (`world(s) not
found`, `--physics-hz` not > 0, a world file without a usd, a refused surface map) were
`SystemExit("message")`, which exits 1, so even a launch that shut down on exit 2 respawned
them.

## Fix

`swb_isaac_world.launch.xml` keeps its name, its arguments and their documentation. It now
includes `world_server.launch.py`, which holds the node and three guards:

- **Refused before the node starts, under both launchers.** An `OpaqueFunction` fails the
  launch once (a `RuntimeError`, as `stack.launch.py` refuses its arguments) on anything the
  world server would refuse that is cheap and sure to check:
  - render, follow, record_view and wheels_topic outside the values the entry script takes;
  - physics_hz not a rate > 0;
  - ISAAC_PORT or EGO_ROS_DOMAIN_ID not a whole number;
  - an ego config or a world entry that does not exist, a world entry expanded as the world
    server expands it (variables, then `~`); a world name the launch's environment cannot
    resolve is left to the world server;
  - render:=window with DISPLAY unset or empty, naming DISPLAY.
  A project's `isaac.render: window`, or any of these, reaches the same check through
  `stack.launch.py`'s include of the XML. The launch then exits 1, under `ros2 launch` and
  under play_launch: not 2 or 3, so look for exit 1 and the message `swb_isaac_world:
  refused before the world server starts: ...`. The environment is read from `os.environ`,
  because play_launch's Rust parser has no `perform` on `EnvironmentVariable`.
- **The world server's refusals are usage exits.** Its refusals before Kit raise `Refused`,
  exit 2 (`EXIT_USAGE`), with the message on stderr, as argparse's do; before, they exited 1.
  Nothing after Kit's start exits 2, so the shutdown below never ends a launch that a
  respawn would cure: Isaac's `SimulationApp` exits 0 or 1 itself and `close()` exits 0 (the
  world server passes no `exit_code`); the request watchdog's kill is -9 (SIGKILL); an
  uncaught exception is 1; a crash is a signal (139, SIGSEGV, in 236 of this host's world
  server exits). Every exit 2 in this host's world server logs came from the entry script's
  refusals.
- **A usage exit ends the launch, as a backstop.** An `OnProcessExit` on the node emits
  `Shutdown`, with the reason, when the node exits 2. Any other exit respawns as before, after
  10 s. This works under `ros2 launch` only (below).

The port and the ego's domain stay `EnvironmentVariable` substitutions in the node's
arguments, as the XML's `$(env ...)` were: under play_launch's Rust parser `os.environ` does
not see a `SetEnvironmentVariable` made before the include. `stack.launch.py ego_domain:=7
world_port:=5599` gives `--port 5599 --ego-domain 7` under both launchers. The entry script's
own checks stay, for anyone who starts it by hand.

### Verified

On 241, with `DISPLAY` unset, each case limited to 60 s (`/home/aeon/coop-c3/i44/t.sh`),
play_launch 0.35.0 (Rust parser):

| case | `ros2 launch` | `play_launch` |
|---|---|---|
| `render:=window` | exit 1 at once, the message, 0 node starts | the same |
| `render:=bogus` | exit 1 at once, "render:=bogus is not one of auto, window, headless", 0 starts | the same |
| `world:=/nonexistent/w.usda` | exit 1 at once, "no such world file, map directory or USD file", 0 starts | the same |
| `ISAAC_ROOT=/nonexistent` (the node exits 1) | respawned, 6 starts in 60 s (as intended) | the same |
| project stack: `stack.launch.py sim:=isaac sim_args:=render:=window` | - | exit 1 at once, the message |

A normal start through the include (`just isaac-world ego:= physics_hz:=30 render:=headless`)
came up in 32.0 s with its arguments intact. Tests: `test_world_server_launch.py` (each
refusal, what the server takes, a name only the server's environment resolves, the shutdown
on exit 2 only, the XML passing every argument, the substitutions) and `test_world_server.py`
(the refusals before Kit exit 2, none left as a bare `SystemExit`): 262 passed.

### Not fixed here: play_launch's exit handlers (upstream)

133 filed these in play_launch main (`15eacb01`): #0090 (a conditional `OnProcessExit` is
ignored), #0091 (`--parser python` ignores #0081's unconditional one), #0092 (the Rust
parser's `SetEnvironmentVariable` does not reach `os.environ`). `EnvironmentVariable.perform`
was fixed on main after v0.36.0 (#0086); the 0.36.0 wheel predates it. The repro files are on
the NAS under "2026 SSv2 on CARLA/2026-11-17 play_launch vs ros2 launch (044)".


Under play_launch, an exit 2 that the launch file did not foresee still respawns: play_launch
ignores the conditional `OnProcessExit` handler, with either parser. Whether that gates the
release candidate is 133's call. Measured on 241 with minimal launch files
(`/home/aeon/coop-c3/i44/pl/`: a node that exits 2 at once, respawn_delay 2 s, 20 s each),
play_launch 0.35.0 and 0.36.0 (`play_launch 0.36.0 (ROS humble, v0.36.0, rlm v0.3.27)`, from
PyPI in a separate venv):

| finding | `ros2 launch` | 0.35.0 Rust | 0.35.0 python | 0.36.0 Rust | 0.36.0 python |
|---|---|---|---|---|---|
| `OnProcessExit(on_exit=<callable>)`, Shutdown when returncode == 2 | 1 start, shut down | respawned, 10 exits | 10 exits | 10 exits | 10 exits |
| `OnProcessExit(on_exit=[EmitEvent(Shutdown())])`, unconditional (play_launch #0081) | 1 start, shut down | 1 exit, shut down | **10 exits** | 1 exit, shut down | **10 exits** |
| `EnvironmentVariable(...).perform(context)` in an OpaqueFunction | works | `AttributeError: ... no attribute 'perform'` | works | **the same AttributeError** | works |
| `os.environ` after `SetEnvironmentVariable("PL_SET", "7")` | `'7'` | `None` | `'7'` | `None` | `'7'` |
