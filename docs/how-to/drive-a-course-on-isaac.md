# Drive a course on Isaac Sim

Worlds need not be CARLA towns. This page adds a course built from open map and elevation
data to an Isaac Sim project and runs Autoware on it. It assumes the project of
[Tutorial 6](../tutorials/06-carla-assets-on-isaac.md), `~/isaac_first_drive`, with its
assets fetched and its stack working.

## The dataset

The `haruna-tsukuba-courses` dataset (ODbL and GSI data, see its notice) holds two
courses: the mountain road on Mount Haruna, Japan, and Tsukuba Circuit. Each is a map
directory with its Autoware map, Isaac world and a scenario. Add the Haruna course to the
project's `assets:` in `swb.yaml`:

```yaml
  - dataset: haruna-tsukuba-courses
    take: [haruna]
```

The course's guardrails and trees are CARLA props from `assets/carla-usd`, which the
project already has. Fetch the dataset and copy the course's scenario into the project:

```bash
ros2 run scensim assets fetch
cp assets/maps/haruna/scenarios/haruna_down.xosc scenarios/
chmod u+w scenarios/haruna_down.xosc
```

Files from scensim's cache are read-only, and `cp` keeps the mode, hence the `chmod`. The
scenario's ego is the catalog car `drift_mustang`: set its `entryName` to `tesla_model3`,
the car of the project's pilot (otherwise `project check` names the mismatch and exits 1).

## The stack on the course

An Isaac stack serves one world for its whole life, so it is stopped (Ctrl-C) and started
again on the course:

```bash
ros2 launch scensim_launch scensim.launch.xml \
    map:=haruna
```

**What you see**: the stack's lines with `pilots on haruna`, and in time `pilot ego ready
on haruna`.

## The run

```bash
ros2 run scensim run scenarios/haruna_down.xosc
```

**Check**: the run ends with a verdict line; a run on Town01 against this stack is
refused, naming the `map:=` to restart with.

The course is long (its scenario allows 600 s of simulation time), and it was made for a
drift controller. Whether the Model 3 under Autoware completes the course has not been
measured.
