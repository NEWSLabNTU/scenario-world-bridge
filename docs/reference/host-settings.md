# Host settings

What differs between machines: seats, GPUs, install paths, the log root and the cache. They
live in files, never in a project's `swb.yaml` and never in exports. Every swb command and
the stack read them, highest first:

1. the command's or the launch's arguments (`--seat`, `seat:=`, `logs:=`, ...);
2. `<project>/.swb/local.yaml`: this checkout only (git-ignored);
3. `~/.config/swb/host.yaml`: yours;
4. `/etc/swb/host.yaml`: the host's, from an administrator (a shared seat table);
5. the defaults below.

A mapping merges key by key (`carla: {port: 3000}` keeps the default host).

```yaml
seat: auto                       # a seat number, or auto: the first free one
gpu: null                        # a GPU index over the seat table's
logs: ~/.local/state/swb/logs    # the log root: on a fast disk (an SSD)
cache: ~/.cache/swb              # the datasets, read-only, shared by your projects
carla: {root: "", host: localhost, port: 2000}
isaac: {root: ""}
autoware: {setup: /opt/autoware/1.5.0/setup.bash}
seats:
  domains: {base: 1, stride: 8}  # seat n: scenario domain base + stride n, then pilots'
  ports: {base: 5555, stride: 100}   # SSv2 +0 (+1, +2), relay +5, world server +36
  web: {base: 8080, stride: 10}  # the stack's web UI +0, a run's scenario +1
  count: 7
  gpus: null                     # a list, seat -> GPU index; null: GPU 0 for every seat
seat_locks: /tmp/swb-seats       # the seat locks: a directory every user can write
```

| Key | Read by | |
|---|---|---|
| `seat` | the stack, `run`, `seat` | your seat; `ros2 run swb seat take <n>` writes it |
| `gpu` | the stack | `CUDA_VISIBLE_DEVICES` for the stack's processes |
| `logs` | the stack, `run`, `runs` | `<logs>/<project name>/` holds the stacks and runs; `<project>/runs` links there |
| `cache` | `assets` | `<cache>/datasets/<repo>/<commit>/` per fetched dataset |
| `carla.host`, `carla.port` | the stack, pilots | where CARLA serves its RPC port |
| `carla.root` | the stack, pilots | CARLA's install: the `carla_basic` pilot's `agents` package (`<root>/PythonAPI/carla`), and the CARLA the stack starts (`carla.start`) |
| `carla.start` | the stack | `auto` (default: start CARLA from `carla.root` when nothing answers on the port), `start` (always; refuse a held port), `external` (never); `simulator_start:=` on the stack's command line overrides it |
| `carla.ready_timeout` | the stack | seconds to wait for a started CARLA's RPC (300) |
| `isaac.root` | not read yet | Isaac's install; the world server reads `ISAAC_ROOT` from the environment ([Getting started on Isaac Sim](../getting-started/isaac.md)) |
| `autoware.setup` | not read yet | Autoware's `setup.bash`; source it in your environment file |
| `seats` | the stack, `seat` | the host's seat table ([Several stacks on one host](../how-to/several-stacks.md)) |
| `seat_locks` | the stack, `seat` | the seat locks' directory |

Every domain of the seat table must be 0 to 57 (the localhost DDS profile's limit), and
seats must not overlap; a table that breaks either is refused when read, naming the file.
`SWB_CONFIG_HOME` and `SWB_SYSTEM_CONFIG` move the user's and the host's directory (for
tests and containers); nothing else is read from the environment.
