# Start a testbench of your own

A **testbench** is one use case: a ROS package in a colcon workspace that holds
`testbench.yaml`, its scenarios and suites. This page creates one, gives it your own maps,
and shares it. It assumes a working install ([Install](../getting-started/install.md)) and
the ideas of [Tutorial 1](../tutorials/01-first-drive.md); the package and its build are
described in [The testbench](../reference/testbench.md).

## From a template or an example

In the workspace's `src/`:

```bash
scensim testbench new my_testbench                          # CARLA, Autoware
scensim testbench new my_testbench --template multi-pilot   # three pilot kinds
scensim testbench new my_testbench --template isaac         # Isaac Sim
scensim testbench new my_testbench \
    --from pkg://scensim_examples/projects/safety_suite      # copies an example
```

The templates are `carla-first-drive` (the default), `multi-pilot` and `isaac`. `--from`
copies an example or another testbench, without what a host makes, and adds the package
manifest and a default suite. The installed examples, each
`pkg://scensim_examples/projects/<name>`:

<!-- TODO(exit run): the examples' names once scensim_examples ships them as testbenches. -->

| Example | Simulator | Shows |
|---|---|---|
| `first_drive` | CARLA | one Autoware ego: a drive to a goal, a red light, a pedestrian crossing |
| `multi_pilot` | CARLA | four pilots in one run: Autoware, a second Autoware, CARLA's BasicAgent and a program cutting in |
| `safety_suite` | CARLA | the Autoware Foundation's ACC and AEB use cases as a nightly suite, with an expected failure and a contact judge |
| `isaac_first_drive` | Isaac Sim | one Autoware ego on CARLA's Town01 in USD |
| `road_physics` | Isaac Sim | mud, gravel and a wet road under the ego's wheels |

Then build, fetch and check:

```bash
cd ~/my_ws && colcon build --symlink-install && source install/setup.bash
scensim assets fetch my_testbench
scensim testbench check my_testbench
```

## What goes in it

```
src/my_testbench/
  package.xml       the build type and the packages it names                (commit)
  testbench.yaml    the simulator, the assets, the pilots, logging, judges   (commit)
  scensim.lock      the assets' exact commits and checksums, written by fetch (commit)
  scenarios/        .xosc and TIER IV .yaml scenarios                         (commit)
  suites/           suite files; `default:` names the one run by name         (commit)
  pilots/, judges/  your own programs, if any                                 (commit)
```

What a host makes for a testbench is kept outside the package, in its work directory
under the log root, `<logs>/.testbenches/<testbench>/`: `assets/` (links into scensim's
cache), `state/` (what the stack derives for a map, such as signal tables and ruts) and
`.scensim/local.yaml` (settings for this host only). Runs go to `<logs>/<testbench>/runs/`.

## Your own maps

A map is a directory holding `lanelet2_map.osm` (and the point cloud, `map_config.yaml`,
`map_projector_info.yaml`, in Autoware's layout). One of your own goes in `assets:` by
path, relative to the testbench, and is linked like a dataset's map:

```yaml
assets:
  - path: maps/campus     # a directory in the testbench, linked as is
    as: campus
```

<!-- TODO(exit run): a map from another package of the workspace (pkg://) in assets:. -->

A scenario then names it `$(env SCENSIM_MAPS)/campus`. A map not drawn in CARLA's frame
needs `carla/map_origin.yaml` in it (the map-frame position of CARLA's world origin); on
Isaac a map needs `isaac/world.yaml` ([Maps and assets](../reference/maps-and-assets.md)).
A dataset of your own on Hugging Face works like the public ones: `{dataset: <name>, repo:
<you>/<name>, take: [...]}`, with `revision:` a tag or commit.

## Which maps a stack serves

Without `maps:`, a stack serves every map the testbench's scenarios name. Name them
yourself when runs use maps the testbench's own scenarios do not, or to fix their order:

```yaml
maps: [Town01, Town05]
```

On CARLA the pilots move to a run's map when it differs (a restart of the pilots, about a
minute); a suite is ordered by map so that it moves at most once per map. On Isaac a stack
serves one world for its life.

## Settings that are not the testbench's

Seats, GPUs, install paths, the log root and the cache belong to the host, never to
`testbench.yaml`: `~/.config/scensim/host.yaml` for you, and the work directory's
`.scensim/local.yaml` for one testbench on this host. Every key:
[Host settings](../reference/host-settings.md).

## Share it

Commit the package with its `scensim.lock`. On another machine:

```bash
mkdir -p ~/my_ws/src && cd ~/my_ws/src
git clone <your repository> my_testbench
cd ~/my_ws && colcon build --symlink-install && source install/setup.bash
scensim assets fetch my_testbench        # exactly the locked files
scensim testbench check my_testbench
```

`fetch` keeps the lock's commits; `scensim assets fetch my_testbench --update` resolves
each dataset again (its `revision:`, else the newest) and rewrites the lock. A run records
the lock's hash and the workspace's commits in its `result.json`, so that two hosts'
results show whether they ran the same thing. A repository may hold several testbenches
and support packages; the workspace builds them together.
