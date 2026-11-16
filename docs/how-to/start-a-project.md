# Start a project of your own

A project is one use case: a directory (a git repository, or a ROS package) with a
`swb.yaml`, its scenarios and suites. This page starts one, gives it your own maps, and
shares it. It assumes a working install ([Install](../getting-started/install.md)) and the
ideas of [Tutorial 1](../tutorials/01-first-drive.md).

## From a template or an example

```bash
ros2 run swb project new ~/my_project                          # CARLA, one Autoware ego
ros2 run swb project new ~/my_project --template multi-pilot   # three pilot kinds
ros2 run swb project new ~/my_project --template isaac         # Isaac Sim
ros2 run swb project new ~/my_project \
    --from pkg://swb_examples/projects/safety_suite            # a copy of an example
```

The templates are `carla-first-drive` (the default), `multi-pilot` and `isaac`. `--from`
copies any project directory (an installed example, or a colleague's checkout) without
what each host makes: `assets/`, `state/`, `runs`, `.swb/`, `play_log/`. The installed
examples, each `pkg://swb_examples/projects/<name>`:

| Example | Simulator | Shows |
|---|---|---|
| `first_drive` | CARLA | one Autoware ego: a drive to a goal, a red light, a pedestrian crossing |
| `multi_pilot` | CARLA | four pilots in one run: Autoware, a second Autoware, CARLA's BasicAgent and a program cutting in |
| `safety_suite` | CARLA | the Autoware Foundation's ACC and AEB use cases as a nightly suite, with an expected failure and a contact judge |
| `isaac_first_drive` | Isaac Sim | one Autoware ego on CARLA's Town01 in USD |
| `road_physics` | Isaac Sim | mud, gravel and a wet road under the ego's wheels |

Each copy has a README naming its commands, what a passing run prints, and the hardware
it needs.

Then, in the project:

```bash
ros2 run swb assets fetch
ros2 run swb project check
```

## What goes in it

```
my_project/
  swb.yaml          the simulator, the assets, the pilots, logging, judges   (commit)
  swb.lock          the assets' exact commits and checksums, written by fetch (commit)
  scenarios/        .xosc and TIER IV .yaml scenarios                         (commit)
  suites/           suite files                                               (commit)
  pilots/, judges/  your own programs, if any                                 (commit)
  .gitignore        assets/ state/ runs .swb/ play_log/
  assets/           links into swb's cache, made by fetch                     (per host)
  state/            what the stack writes for a map: signal tables, ruts      (per host)
  runs/             a link to this project's runs under the log root          (per host)
  .swb/local.yaml   host settings for this checkout only                      (per host)
  play_log/         play_launch's own logs, when it runs in the project       (per host)
```

Every key of `swb.yaml` and of a suite: [The project file](../reference/project-file.md).

## Your own maps

A map is a directory holding `lanelet2_map.osm` (and the point cloud, `map_config.yaml`,
`map_projector_info.yaml`, in Autoware's layout). One of your own goes in `assets:` by
path, relative to the project, and is linked into `assets/maps/` like a dataset's:

```yaml
assets:
  - path: ../shared/campus_map     # a directory of your own, linked as is
    as: campus
```

A scenario then names it `$(env SWB_MAPS)/campus`. A map not drawn in CARLA's frame needs
`carla/map_origin.yaml` in it (the map-frame position of CARLA's world origin); on Isaac a
map needs `isaac/world.yaml` ([Maps and assets](../reference/maps-and-assets.md)).

A dataset of your own on Hugging Face works like the public ones: `{dataset: <name>,
repo: <you>/<name>, take: [...]}`, with `revision:` a tag or commit.

## Which maps a stack serves

Without `maps:`, a stack serves every map the project's scenarios name. Name them yourself
when runs use maps the project's own scenarios do not (a scenario run by path), or to fix
their order:

```yaml
maps: [Town01, Town05]
```

On CARLA the pilots move to a run's map when it differs (a restart of the pilots, about a
minute); a suite is ordered by map so that it moves at most once per map. On Isaac a stack
serves one world for its life.

## Settings that are not the project's

Seats, GPUs, install paths, the log root and the cache are the host's, never in
`swb.yaml`: `~/.config/swb/host.yaml` for you, `<project>/.swb/local.yaml` for this
checkout only (git ignores it). For example, a checkout whose logs go to a second disk:

```yaml
# my_project/.swb/local.yaml
logs: /data/swb-logs
```

Every key: [Host settings](../reference/host-settings.md).

## Share it

Commit `swb.yaml`, `swb.lock` and the project's files. On another machine:

```bash
git clone <your repository> my_project
cd my_project
ros2 run swb assets fetch      # exactly the locked files
ros2 run swb project check
```

`fetch` keeps the lock's commits; `ros2 run swb assets fetch --update` resolves each
dataset again (its `revision:`, else the newest) and rewrites the lock. A run records the
lock's hash and the workspace's commits in its `result.json`, so two hosts' results say
whether they ran the same thing.

A project can also be a ROS package that installs its `swb.yaml` and files to its share
directory; `project:=pkg://<package>` on the stack and `--project pkg://<package>` on the
commands find it there, and `project new --from pkg://<package>` copies it.
