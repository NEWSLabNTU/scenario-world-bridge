# Host settings

What differs between machines: seats, GPUs, install paths, the log root and the cache. They
live in files, never in a testbench's `testbench.yaml` and never in exports. Every scensim command and
the stack read them, highest first:

1. the command's or the launch's arguments (`--seat`, `seat:=`, `logs:=`, ...);
2. `.scensim/local.yaml` in a testbench's work directory, `<logs>/.testbenches/<testbench>/`:
   this testbench on this host only (a project directory's own `.scensim/local.yaml` for a
   project);
3. `~/.config/scensim/host.yaml`: yours;
4. `/etc/scensim/host.yaml`: the host's, from an administrator (a shared seat table);
5. the defaults below.

A mapping merges key by key (`carla: {port: 3000}` keeps the default host).

```yaml
seat: auto                       # a seat number, or auto: the first free one
gpu: null                        # a GPU index over the seat table's
logs: ~/.local/state/scensim/logs    # the log root: on a fast disk (an SSD)
cache: ~/.cache/scensim          # the datasets, read-only, shared by your testbenches
launcher: auto                   # auto, play_launch or ros2
# workspaces: {autoware-drift: ~/repos/autoware-drift}   # a pilot's own ROS workspaces
carla: {root: "", host: localhost, port: 2000}
isaac: {root: ""}
autoware: {setup: /opt/autoware/1.5.0/setup.bash}
seats:
  domains: {base: 1, stride: 8}  # seat n: scenario domain base + stride n, then pilots'
  ports: {base: 5555, stride: 100}   # SSv2 +0 (+1, +2), relay +5, world server +36
  web: {base: 8080, stride: 10}  # the stack's web UI +0, a run's scenario +1
  # carla: {base: 2000, stride: 10}   # optional: seat n's CARLA port base + stride n
  count: 7
  gpus: null                     # a list, seat -> GPU index; null: GPU 0 for every seat
seat_locks: /tmp/scensim-seats   # the seat locks: a directory every user can write
```

| Key | Read by | |
|---|---|---|
| `seat` | the stack, `run`, `seat` | your seat; `scensim seat take <n>` writes it |
| `gpu` | the stack | `CUDA_VISIBLE_DEVICES` for the stack's processes |
| `logs` | the stack, `run`, `runs` | `<logs>/<testbench>/` holds the stacks and runs, `<logs>/.testbenches/<testbench>/` the testbench's work directory |
| `cache` | `assets` | `<cache>/datasets/<repo>/<commit>/` per fetched dataset |
| `datasets` | `assets fetch`, `assets check` | `{<dataset>: <directory>}`: datasets this host keeps a copy of itself (on a shared disk, say). A testbench's `- dataset: <name>` entry is then linked from that directory, with the testbench's `take:`, instead of fetched; `scensim.lock` keeps only what came from the Hub, so the testbench reads the same on every host |
| `workspaces` | `testbench check`, the stack | `{<name>: <directory>}`: ROS workspaces this host has built, for a pilot's `overlays:` entries `{workspace:<name>}/<path>` ([The testbench file](project-file.md)), as `datasets` names a dataset's copy; a testbench naming a workspace this host does not have is refused |
| `carla.host`, `carla.port` | the stack, pilots | where CARLA serves its RPC port |
| `carla.root` | the stack, pilots | CARLA's install: the `carla_basic` pilot's `agents` package (`<root>/PythonAPI/carla`), and the CARLA the stack starts (`carla.start`) |
| `carla.start` | the stack | `auto` (default: start CARLA from `carla.root` when nothing answers on the port), `start` (always; refuse a held port), `external` (never); `simulator_start:=` on the stack's command line overrides it |
| `carla.ready_timeout` | the stack | seconds to wait for a started CARLA's RPC (300) |
| `carla.map_packages` | `testbench check` | the directories this host's CARLA towns are installed in (default: `<carla.root>/CarlaUE4/Content`); a testbench map needs its town there (`<town>.umap` in a package's `Maps/`), so a town you installed into CARLA yourself is this host's, not the testbench's. Unset with no `carla.root`: not checked |
| `launcher` | `testbench check`, `seat show`, the stack, `run` | what starts the stack and each launch of it (its world server, its pilots, a run's SSv2): `auto` (default: play_launch when it is on `PATH` at 0.30 or later, else `ros2 launch`), `play_launch` (refused by `testbench check` when missing or older) or `ros2`; `launcher:=` on the stack's command line overrides it. `testbench check` and `seat show` print the choice (`launcher: auto -> play_launch 0.30.0`) |
| `dds.profile` | the stack, `seat env` | a Cyclone DDS XML for every process of the stack and every exported terminal, in place of scensim's localhost profile (a network that needs another, for example unicast on loopback); empty: scensim's |
| `isaac.root` | the stack, `testbench check` | Isaac's install, handed to the Isaac world server as `ISAAC_ROOT` ([Getting started on Isaac Sim](../getting-started/isaac.md)); required for an Isaac testbench, with no default. `testbench check` and the stack refuse an Isaac testbench when neither `isaac.root` nor `ISAAC_ROOT` is set, or when the root is not an Isaac Sim install (no `venv/bin/activate` or `venv/lib/python3.12/site-packages/isaacsim`, which `install_isaac.sh` makes) |
| `autoware.setup` | autoware pilots | Autoware's `setup.bash` (`/opt/autoware/1.5.0/setup.bash`); an autoware pilot sources it, then the items below, then this workspace's installs. Its directory names the release the pilot runs (`/opt/autoware/2.1.0/setup.bash`: 2.1.0, one of 1.5.0, 2.0.0 and 2.1.0); a pilot whose `args` name another `autoware_release` is refused, and so is one whose workspaces (this one's, acb's) were built against another Autoware: rebuild them with `AUTOWARE_SETUP` set to this file |
| `autoware.overlay`, `autoware.use_overlay` | autoware pilots | the Autoware fixes this workspace carries, built into `third_party/autoware_overlay` (default); `use_overlay: false` runs stock Autoware. The fixes are for Autoware 1.5.0: with another release the overlay is not sourced, and the pilot's log says so |
| `autoware.cuda_ndt` | autoware pilots | the GPU NDT matcher's build (default `third_party/cuda_ndt_matcher`), sourced when built |
| `autoware.tensorrt_lib` | autoware pilots | a TensorRT 10.8.0 runtime put first on `LD_LIBRARY_PATH` (default `third_party/tensorrt/lib`), so CenterPoint's cached engines load instead of being rebuilt at every start |
| `seats` | the stack, `seat` | the host's seat table ([Several stacks on one host](../how-to/several-stacks.md)) |
| `seats.carla` | the stack, `seat` | `{base, stride}`: each seat its own CARLA port, `base + stride n` for seat `n`, over `carla.port`. `stride` at least 3 (CARLA uses its port and the two after it); a table whose CARLA ports meet the seats' other ports is refused. `seat show` prints each seat's CARLA port. With `carla.root` set and `carla.start: auto`, each seat's stack starts its own CARLA there |
| `hub_url` | `assets` | the Hugging Face endpoint (default the public Hub); another URL is read over plain HTTPS with the Hub's API, for a mirror of the datasets |
| `seat_locks` | the stack, `seat` | the seat locks' directory |

Every domain of the seat table must be 0 to 57 (the localhost DDS profile's limit), and
seats must not overlap; a table that breaks either is refused when read, naming the file.
Seat `n` takes the domains from `domains.base + stride n` up to the next seat's, so the
last seat ends near `domains.base + stride count`: the default table (base 1, stride 8,
count 7) ends at domain 56. Raising `domains.base` therefore means lowering `count` (or
`stride`) to match. ROS itself allows domains up to 232, but the Cyclone profile used here
keeps them at 57 or below.
`SCENSIM_CONFIG_HOME` and `SCENSIM_SYSTEM_CONFIG` move the user's and the host's directory (for
tests and containers); nothing else is read from the environment.
