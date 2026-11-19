# Drive a course on Isaac Sim

Worlds need not be CARLA towns. This page adds a course built from open map and elevation
data to an Isaac Sim testbench and runs Autoware on it. It assumes the testbench of
[Tutorial 6](../tutorials/06-carla-assets-on-isaac.md), `~/isaac_first_drive`, with its
assets fetched and its stack working.

## The dataset

The `haruna-tsukuba-courses` dataset (ODbL and GSI data, see its notice) holds two
courses: the mountain road on Mount Haruna, Japan, and Tsukuba Circuit. Each is a map
directory with its Autoware map, Isaac world and a scenario. Add the Haruna course to the
testbench's `assets:` in `testbench.yaml`:

```yaml
  - dataset: haruna-tsukuba-courses
    take: [haruna]
```

The course's guardrails and trees are CARLA props from `assets/carla-usd`, which the
testbench already has. Fetch the dataset, copy the course's scenario into the testbench,
and build, since the scenario is a new file:

```bash
scensim assets fetch isaac_first_drive
w=~/.local/state/scensim/logs/.testbenches/isaac_first_drive   # the work directory
cd ~/my_ws/src/isaac_first_drive
cp "$w"/assets/maps/haruna/scenarios/haruna_down.xosc scenarios/
chmod u+w scenarios/haruna_down.xosc
cd ~/my_ws && colcon build --symlink-install
```

Files from scensim's cache are read-only, and `cp` keeps the mode, hence the `chmod`. The
scenario's ego is the catalog car `drift_mustang`: set its `entryName` to `tesla_model3`,
the car of the testbench's pilot (otherwise `testbench check` names the mismatch and exits 1).
The dataset's scenario predates scensim's names: `testbench check` names its two old lines
with their fix, the map as `$(env SCENSIM_MAPS)/haruna` and the vehicle catalog as
`$(env SCENSIM_VEHICLES)`:

```bash
f=src/isaac_first_drive/scenarios/haruna_down.xosc   # from ~/my_ws
sed -i -e 's|entryName="drift_mustang"|entryName="tesla_model3"|' \
    -e 's|$(env SWB_MAPS)/haruna|$(env SCENSIM_MAPS)/haruna|' \
    -e 's|<Directory path="$(find-pkg-share sim_vehicle_spec)/catalogs" */>|'\
'<Directory path="$(env SCENSIM_VEHICLES)" />|' "$f"
scensim testbench check isaac_first_drive
```

## The stack on the course

An Isaac stack serves one world for its whole life, so it is stopped, and the run below
starts it again on the course's map (a stack a run starts is on that run's map):

```bash
scensim stack isaac_first_drive --stop
```

## The run

```bash
scensim run isaac_first_drive/scenarios/haruna_down.xosc
```

**Check**: the run ends with a verdict line; a run on Town01 against this stack is
refused, naming the `map:=` to restart with.

The course is long (its scenario allows 600 s of simulation time), and it was made for a
drift controller. Whether the Model 3 under Autoware completes the course has not been
measured.
