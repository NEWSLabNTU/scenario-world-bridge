# Tutorial 7: Road physics

**Goal**: Autoware drives a Model 3 on Isaac Sim across a wet stretch, 12.8 m of wet mud
and a gravel patch on Town01, on ground that the project defines, while each wheel's
surface, slip and sinkage are observed as it crosses.

<p align="center">
  <img src="../media/gravel.webp" alt="A car at 60 km/h on a gravel road with ruts, potholes and loose rocks in Isaac Sim" width="640">
</p>

## You will learn

- what the Isaac world server models under a wheel: surface classes, soil with sinkage and
  ruts, a rough road;
- to give a map its ground from the project (`ground:` in `swb.yaml`), without editing
  the shared map;
- to check a surface map without Isaac;
- to change the road's state from a scenario with `RoadCondition`;
- to watch `/swb/wheels`, and record it with a run.

## Before you start

- [Tutorial 6](06-carla-assets-on-isaac.md): Isaac Sim and an Isaac stack that became
  ready on Town01. Stop that stack (Ctrl-C), since this tutorial uses a project of its own.
- The same hardware as Tutorial 6. The ruts of a soil zone occupy memory, about 100 MB per
  km of tracks, for as long as the world server runs.
- About 45 minutes.

## 1. Copy the example

```bash
ros2 run swb project new ~/road_physics \
    --from pkg://swb_examples/projects/road_physics
cd ~/road_physics
ros2 run swb assets fetch
cat swb.yaml ground/town01.yaml
```

**What you see**: the Isaac project of Tutorial 6 with a third, small dataset and a key of
its own:

```yaml
ground:                              # per map: this project's surface files
  Town01:
    surfaces: ground/town01.yaml
```

followed by the **surface map**, the file that assigns a surface class to the ground. It
gives defaults per semantic label (roads asphalt, pavements concrete, verges grass) and
three **zones**, regions that override those defaults along the scenarios' route:
`wet_stretch`, `mud_road` (wet mud with soil on its own ground, so that it ruts) and
`gravel_patch`.

The ground is a file of the project rather than of the map, because the fetched map is
read-only and shared by every project; another project on Town01 drives on its own
ground.

**Check**, without Isaac:

```bash
ros2 run swb_isaac_world surfaces check ground/town01.yaml
```

```
warning: zone mud_road: soil on unvalidated mud_wet parameters: drawbar pull fitted ...
ground/town01.yaml: 3 zone(s), 1 with soil; 12 classes
```

The warning is expected: the sinkage of `mud_wet` lies outside its validation tolerance,
and the map accepts it explicitly by name (`allow_unvalidated: [mud_wet]`).

## 2. Hand the ground to the world server

The world server reads a map's surface map from the project's `state/<map>/` first. The
file is not copied there by hand: at start the stack links each file that `ground:` names
into `state/<map>/` (`surfaces` as `surfaces.yaml`, `road_detail` as `road_detail.yaml`),
so `ground/town01.yaml` remains the only file to edit.

**Check**: after step 3, `ls -l state/Town01/` shows `surfaces.yaml -> .../ground/town01.yaml`.

## 3. Bring up the stack

```bash
play_launch launch --web-addr 127.0.0.1:8080 \
    swb_launch swb.launch.xml
```

**What you see**: as in Tutorial 6, ending `swb stack ...: ready`.

**Check**: in the stack's web UI, the world server's log (the simulator group) warns about
`mud_wet` and has the lines `soil zone mud_road: cut (...) out of 8 mesh(es)` and
`soil zone mud_road: 32 tiles of 1.6 m at z 0.00`, which show that it read the project's
surface map.

## 4. Drive through the mud, and watch the wheels

In a second terminal, join the ego's ROS domain and subscribe to the wheel states:

```bash
cd ~/road_physics
eval "$(ros2 run swb seat env 0)"
ros2 topic echo /swb/wheels --field status
```

In a third terminal, in the project, start the drive:

```bash
ros2 run swb run scenarios/mud_and_gravel.xosc
```

**What you see**: in the second terminal, four statuses per sample, one per wheel
(`ego/front_left` ...), each with its `class` and `zone`, `friction`, `slip_ratio`,
`slip_angle`, `sinkage` and `normal_load`. On the wet stretch the class remains `asphalt`
at a lower friction; in the mud it becomes `mud_wet`, the slip increases and the sinkage
reaches a few centimetres; then it becomes `gravel`. The rear-driven car slows in the mud
but crosses it, and the run ends `passed`.

**Check**: the ruts the run left in the mud are saved when the world server next loads the
world, which happens at the start of the next run. After the drive of step 5,

```bash
ls state/Town01/ruts/runs/
```

lists them.

## 5. The road after rain

`scenarios/after_rain.xosc` is the same drive with the road's state set by the scenario:

```xml
<RoadCondition frictionScaleFactor="1.0" wetness="moist">
  <Properties>
    <Property name="swb.zone.gravel_patch.class" value="dirt" />
  </Properties>
</RoadCondition>
```

The whole road becomes moist and the gravel patch becomes dirt, while the mud is
unchanged. A scenario sets the state of the road; it cannot choose the surface map.

```bash
ros2 run swb run scenarios/after_rain.xosc --capture full
```

`--capture full` records the run's key topics, `/swb/wheels` among them, as a rosbag, and
keeps the recording whatever the verdict.

**What you see**: `passed   <run id>  <run directory>`; the world server's log has
`road condition: friction scale 1, ...` at the start.

**Check**:

```bash
ros2 bag info runs/*after_rain*/record/ego
```

lists `/swb/wheels` among the recorded topics.

## What happened

The Isaac world server assigns every piece of ground a surface class, from its semantic
label or from a zone, and derives each wheel's friction and tire curve from that class at
every physics step. In a soil zone it also runs a soil model (sinkage, traction limit,
motion resistance, and ruts in which later wheels run); on the zone's own ground
(`ground_z`) the ruts also enter the collision geometry and the lidar. The project's files
live in `state/<map>/` beside what the server writes there, so the shared map is never
edited. [Road physics on Isaac Sim](../reference/road-physics.md) lists every class, zone
key, road-detail key and `RoadCondition` property.

## Exercises

1. **Easy.** Make the mud softer: set `mud_road`'s class to `mud_soft` in
   `ground/town01.yaml`; the change takes effect from the next scenario. Check:
   `mud_and_gravel.xosc` ends `failed` on its 180 s timeout with the car stalled in the
   mud, and `/swb/wheels` shows the rear wheels' slip near 1.
2. **Medium.** Lay the rough gravel road the stages dataset carries over Town01's
   straight: uncomment `road_detail:` under `ground: Town01:` in `swb.yaml` (it names
   `assets/isaac-road-physics-stages/procedural/spikes/road_town/road_detail.yaml`) and
   restart the stack. Check:
   `ros2 run swb_isaac_world road_detail check state/Town01/road_detail.yaml` prints
   `1 corridor(s)` and the corridor's 250 m, and the stack starts again without a
   problem. (Autoware has not been validated on the rough road.)
3. **Open.** Start from old ruts: copy a file of `state/Town01/ruts/runs/` to
   `state/Town01/ruts/worn.npz`, and in a copy of `mud_and_gravel.xosc` add
   `<Property name="swb.ruts" value="worn"/>` to a `RoadCondition` in the Init. Check: the
   world server logs `ruts: set worn from ...`, and the wheels' `sinkage` in the old ruts
   differs from a fresh run's.

## If it does not work

| Symptom | Fix |
|---|---|
| Every wheel's class is `asphalt` | the world server found no surface map: check `ground:` in `swb.yaml` (`ros2 run swb project check`), then restart the stack |
| The stack is refused, naming `surfaces.yaml` | the file failed its check: run `surfaces check` on it, which lists every problem |
| `/swb/wheels` shows nothing | the terminal is not on the ego's domain (`eval "$(ros2 run swb seat env 0)"`), or the stack is not up |
| `mud_and_gravel.xosc` fails, the car stopped in the mud | a car that stops in fresh wet mud can hardly start again, and Autoware's longitudinal controller does not add torque at low speed; the shipped scenarios do not stop there |

## Next

- [Road physics on Isaac Sim](../reference/road-physics.md): every key.
- [Your car](../how-to/your-car.md), to drive the same ground with a car of your own.
