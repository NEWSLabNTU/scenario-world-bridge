# Tutorial 7: Road physics: surfaces, mud and rocks

Isaac Sim only. The Isaac world server gives every piece of ground a surface class (dry or
wet asphalt, gravel, sand, mud, snow, ice, ...), runs soft soil with sinkage and ruts where a
map says so, and can lay a rough gravel road with potholes, washboard and rocks over a
town's road. This tutorial shows what each part does, how to switch it on, and what it does
not do yet.

It assumes the Isaac stack from [Getting started on Isaac Sim](../getting-started/isaac.md)
(stages 1 to 3: Town01, its world file, `~/swb.bash`). Every terminal sources `~/swb.bash`,
and `SWB` is your checkout:

```bash
source ~/swb.bash
SWB=~/scenario-world-bridge
```

Commands use `play_launch launch`; `ros2 launch` takes the same arguments, but play_launch
must still be installed.

## 1. What is modelled

| Layer | What it does | Switched on by |
|---|---|---|
| Surface layer | each wheel's friction and tire curve from the ground under it, every physics step; wetness, standing water, hydroplaning | `surfaces.yaml` in the map directory |
| Soil layer | in a soil zone: sinkage, traction limit, motion resistance, the soil against a sunk tire's side, ruts that later wheels ride in | a zone with `soil:` in `surfaces.yaml` |
| Road detail | a rough road over the town's own: roughness, crown and ditches, ruts, potholes, washboard, water bars, rocks | `road_detail.yaml` beside `surfaces.yaml` |
| The car's computers | ABS and traction control (on), stability control (off by default), on the car's own sensors | the ego config |

All of it is physics. The world looks the same, except that a soil zone with its own ground
shows its ruts (and the lidar sees them) and the road detail streams a render mesh.

## 2. The surface classes

The classes live in `src/swb_isaac_world/config/surface_classes.yaml`. `mu` is the dry peak
tire-road friction; the Model 3's own tire on dry asphalt is the reference (1.12), so a map
whose roads are all asphalt drives exactly as a map with none. The wet columns multiply `mu`
at OpenSCENARIO's wetness levels; a level a class does not list takes the next drier one's.

| Class | mu | moist | wetWithPuddles | lowFlooded | highFlooded | Soil |
|---|---|---|---|---|---|---|
| `asphalt` | 1.12 | 0.9 | 0.75 | 0.65 | 0.55 | no; hydroplanes |
| `concrete` | 1.05 | 0.9 | 0.75 | 0.65 | 0.55 | no; hydroplanes |
| `gravel` | 0.6 | 0.95 | 0.9 | 0.85 | 0.8 | no |
| `grass` | 0.55 | 0.8 | 0.6 | 0.5 | 0.45 | no |
| `dirt` | 0.68 | 0.85 | 0.7 | 0.6 | 0.5 | yes (sandy loam) |
| `sand` | 0.265 | 1.1 | 1.0 | 0.9 | 0.8 | yes |
| `sand_loose` | 0.245 | 1.1 | 1.0 | 0.9 | 0.8 | yes |
| `mud_wet` | 0.29 | 1 | 1 | 1 | 1 | yes, flagged |
| `mud_soft` | 0.26 | 1 | 1 | 1 | 1 | yes |
| `snow_packed` | 0.2 | 0.8 | 0.6 | 0.6 | 0.6 | no |
| `snow_loose` | 0.25 | 1 | 1 | 1 | 1 | yes, flagged |
| `ice` | 0.265 | 0.47 | 0.47 | 0.47 | 0.47 | no |

Each class also sets where its tire curve peaks (`peak_slip`, `peak_slip_angle`), how much
grip is left sliding past the peak (`sliding`), and how quickly (`relaxation`); ice's sliding
grip also falls with speed. The soil classes carry Bekker-Wong-Janosi parameters
(`n`, `k_c`, `k_phi`, cohesion, friction angle, Janosi `K`, slip sinkage, rolling
resistance) and a `calibration:` line naming what they were fitted to:

| Soil class | Fitted to | Notes |
|---|---|---|
| `sand` | a road tire on dry loamy sand (He 2020) | drawbar pull and sinkage within tolerance |
| `sand_loose` | the same tire on loosened sand | a rear-drive car does not start from rest on it |
| `dirt` | a textbook sandy loam, fitted to Chrono SCM | no measured tire yet |
| `mud_wet` | a smooth tire on wet clay (Wismer 1966) | pull fitted; sinkage outside tolerance, so flagged |
| `mud_soft` | the same tire on wetter clay | within tolerance; a rear-drive car stalls in it |
| `snow_loose` | a tire towed through 15 cm of fresh snow (Richmond 1995) | traction not measured, so flagged |

A soil zone on a flagged class is refused unless the map lists the class under
`allow_unvalidated:`, and the world server warns at every load.

Standing water: a zone's `water_depth` (or the depth a wetness implies) lowers grip on
paved classes with speed, toward hydroplaning near 100 km/h at full depth with the default
250 kPa tire. That model is an engineering one, not validated.

## 3. How the world assigns ground

A town's Isaac world is CARLA's SimReady export plus a physics layer,
`<Town>_physics.usda`, which gives its meshes colliders; each collider keeps its semantic
label (Road, RoadLine, SideWalk, Terrain, Ground). The surface map,
`<map dir>/surfaces.yaml`, turns labels into classes and adds zones on top:

```yaml
# $CARLA_MAPS/Town01/surfaces.yaml
allow_unvalidated: [mud_wet]   # flagged classes soil zones may use
defaults: {Road: asphalt, RoadLine: asphalt, SideWalk: concrete,
           Terrain: grass, Ground: dirt}
zones:                          # later zones lie on top
  - id: puddle_left             # [a-z0-9_]+; scenarios name it
    water_depth: 0.006          # m, or a raster (PNG or .npy)
    polygon: [[150.0, -134.0], [160.0, -134.0], [160.0, -130.0], [150.0, -130.0]]
  - id: mud_road
    class: mud_wet
    soil: grid                  # grid: ruts, multipass; closed_form: no memory
    ground_z: 0.0               # optional: the server's own ground here, ruts in it
    polygon: [[145.6, -134.4], [158.4, -134.4], [158.4, -128.0], [145.6, -128.0]]
```

| Zone key | Meaning |
|---|---|
| `class` | a class from the table |
| `wetness` | one of OpenSCENARIO's levels, over the scenario's |
| `water_depth` | m, a number or a raster with `scale`, `origin`, `resolution` |
| `soil` | `grid` (or `true`) or `closed_form`; needs a soil class |
| `ground_z` | with `soil: grid`: the zone's ground is the server's own flat tiles at this height, the town's ground cut out under them, so ruts are in the collision and the lidar |
| `polygon` | map frame, metres |

Rules that matter:

- The world server reads the surface map of a world file it was started with
  (`world:=`), from that world's map directory. A world loaded later without being in
  `world:=` has no surface map.
- It checks the file before Isaac starts and refuses it with every problem listed, then
  reads it again at every world load: an edit counts from the next scenario.
- `ground_z` needs flat ground at that height; the shipped maps use polygons on whole
  1.6 m tiles.
- Without a surface map every wheel runs on asphalt.

Check a map without Isaac (numpy and PyYAML; `plot` also needs matplotlib):

```bash
cd $SWB/src/swb_isaac_world
python3 -m swb_isaac_world.surfaces check $CARLA_MAPS/Town01
python3 -m swb_isaac_world.surfaces plot $CARLA_MAPS/Town01   # writes surfaces.png
cd -
```

The driving courses (Mount Haruna and Tsukuba,
[haruna-tsukuba-courses](https://huggingface.co/datasets/jerry73204/haruna-tsukuba-courses))
are map directories too: each has its own `surfaces.yaml` (label defaults, no zones), which
takes zones in the same format.

## 4. Every surface side by side

The [isaac-road-physics-stages](https://huggingface.co/datasets/jerry73204/isaac-road-physics-stages)
dataset holds the flat test ground with 13 surface strips (the 12 classes plus wet asphalt):

```bash
hf download jerry73204/isaac-road-physics-stages \
    --repo-type dataset \
    --local-dir ~/road-physics
```

| File | What |
|---|---|
| `materials_ground.usda` | flat ground, one 200 x 60 m strip per surface along y, each with a coloured plate |
| `materials_surfaces.yaml` | its surface map: a zone per strip, soil on the grid for the six soil classes |
| `proving_ground.usda` | the vehicle check's proving ground |
| `procedural/` | the stage files and scripts for the stages built at run time (section 7), Town01's worn ruts, a README on rebuilding each |

Check the map, and open the stage in Isaac Sim to look at it:

```bash
cd $SWB/src/swb_isaac_world
python3 -m swb_isaac_world.surfaces check ~/road-physics/materials_surfaces.yaml
cd -
```

Driving it is a script today, not the stack (the stack's autopilot needs a lanelet map,
and this ground has none). `materials_demo.py` builds the same ground, then drives three
tests on each strip: a full-throttle start, a brake from 50 km/h and a 25 deg steering
step, with no autopilot and nothing stepping in. It runs headless in Isaac's own Python
and ROS libraries, so start it from a terminal that has not sourced ROS or `~/swb.bash`:

```bash
cd ~/scenario-world-bridge
MATERIALS=mud_wet,sand,ice TESTS=start,brake \
EGO_RIG=tesla_model3.rig.yaml \
ISAAC_ROOT=~/isaac-sim \
CARLA_USD=~/carla-usd \
SWB_VEHICLE_USD=~/carla-usd/vehicles \
    src/swb_isaac_world/spikes/run_spike.sh fastrtps /tmp/materials.log \
    $PWD/src/swb_isaac_world/spikes/materials_demo.py \
    /tmp/materials_demo
```

| Switch | Default |
|---|---|
| `MATERIALS` | every strip (keys: `asphalt`, `asphalt_wet`, `concrete`, `gravel`, `grass`, `dirt`, `sand`, `sand_loose`, `mud_wet`, `mud_soft`, `snow_packed`, `snow_loose`, `ice`) |
| `TESTS` | `start,brake,turn` |
| `EGO_RIG` | `drift_mustang.rig.yaml` (no ABS, TC or ESC); `tesla_model3.rig.yaml` for the Model 3 |

`/tmp/materials_demo/tests.jsonl` has one line per test with its numbers;
`/tmp/materials_demo/record/` has the pictures and each frame's wheel state (section 9).
Paths are absolute: the script runs from `ISAAC_ROOT`.

## 5. A muddy road on Town01

The checkout's `scenarios/surfaces/` pairs each scenario with its ground. The scenario does
not choose the ground: you copy the surface map into the map directory first.

| Scenario | Ground (`.yaml` of the same name) |
|---|---|
| `town01_mud.xosc` | Town01's starter drive over 12.8 m of wet mud with its own ground |
| `town01_mud_curve.xosc` | a 90 deg bend of about 12 m radius in mud |
| `town01_mud_right.xosc` | an emergency stop with mud under the right wheels only (split grip) |
| `town01_mud_stop.xosc` | an emergency stop on 48 m of mud for a pedestrian |
| `town01_soil.xosc` | sand, then dirt, behind a lead car that lays ruts first |
| `town01_snow.xosc` | 12.8 m of loose snow |
| `town01_mixed_ground.xosc` | wet asphalt with puddles, then sand, dirt and mud |
| `town01_zones.xosc` | a wet stretch, a puddle under the left wheels, gravel, ice at the goal (`town01_route.yaml`) |
| `town01_wet.xosc` | the whole road moist, then wet with puddles (no surface map needed) |

In runs on 2026-10-26, `town01_mud_stop` stopped short of the pedestrian but did not get
going again: the rear-drive Model 3 barely starts in fresh `mud_wet`, and Autoware's stock
longitudinal controller does not raise its request at low speed. It ended on its timeout.

Put the mud in, start the stack (or leave it running: the map counts from the next
scenario), and run the scenario:

```bash
cp $SWB/scenarios/surfaces/town01_mud.yaml $CARLA_MAPS/Town01/surfaces.yaml

# Terminal 1
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch isaac.launch.xml \
    world:=$CARLA_MAPS/Town01/isaac/world.yaml \
    render:=window \
    map_path:=$CARLA_MAPS/Town01

# Terminal 2, once the stack is ready (Getting started, stage 3)
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} play_launch launch \
    --web-addr 127.0.0.1:8081 \
    swb_launch scenario.launch.xml \
    scenario:=$SWB/scenarios/surfaces/town01_mud.xosc \
    port:=${SSV2_PORT:-5555} \
    output_directory:=/tmp/mud
```

Drive it yourself instead with free drive: the ego spawns at the start of the same road,
heading west toward the mud, and you give Autoware a goal in RViz (2D Goal Pose past the
mud, around x 120, y -130), then engage:

```bash
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch isaac.launch.xml \
    world:=$CARLA_MAPS/Town01/isaac/world.yaml \
    render:=window \
    map_path:=$CARLA_MAPS/Town01 \
    spawn:=190.8,-130.1,0.3,3.14159
```

**Old ruts.** When a scenario ends, the world server saves the ruts it made to
`<map dir>/ruts/runs/<time>.npz` and logs the path; copied to `<map dir>/ruts/<name>.npz` it
becomes a set a scenario can start from. The dataset carries one, `worn`, for
`town01_soil.yaml`'s two zones:

```bash
mkdir -p $CARLA_MAPS/Town01/ruts
cp ~/road-physics/procedural/ruts/Town01/worn.npz $CARLA_MAPS/Town01/ruts/
cp $SWB/scenarios/surfaces/town01_soil.yaml $CARLA_MAPS/Town01/surfaces.yaml
```

Then set `town01_soil.xosc`'s parameter `ruts` from `fresh` to `worn` (a copy of the file)
and run it as above.

Remove `$CARLA_MAPS/Town01/surfaces.yaml` to go back to plain asphalt.

## 6. What a scenario can set

A scenario sets the road's state through OpenSCENARIO's `RoadCondition`, in the Init or in
the Story (`EnvironmentAction`). The map says where; the scenario says what state:

```xml
<RoadCondition frictionScaleFactor="1.0" wetness="moist">
  <Properties>
    <Property name="swb.zone.puddle_left.water_depth" value="0.003"/>
    <Property name="swb.zone.gravel_patch.class" value="dirt"/>
    <Property name="swb.ruts" value="worn"/>
  </Properties>
</RoadCondition>
```

| Setting | Effect on Isaac |
|---|---|
| `frictionScaleFactor` | multiplies every wheel's friction |
| `wetness` | the whole road's wetness, where a zone sets none |
| `swb.zone.<id>.class` | a zone's class (not a soil zone's: its class is its soil's) |
| `swb.zone.<id>.wetness` | a zone's wetness |
| `swb.zone.<id>.water_depth` | a zone's water depth, m |
| `swb.ruts` | `fresh` or a saved set; put it in the Init: swapping ruts once a wheel has driven on the soil is refused |

Each `RoadCondition` is the whole road state: zones it does not name go back to the map's
values. A wrong property (an unknown zone, class or field) fails the scenario with every
problem listed, rather than being ignored. A scenario cannot choose which surface map to
use, nor change the label defaults: that is the map directory's one `surfaces.yaml`.

## 7. Rocks and a rough road

A `road_detail.yaml` beside `surfaces.yaml` (or alone) lays a rough road along a
centreline over the town's own road. The tire's contact runs on it every physics step: a
pothole drops the wheel and hits it on the far edge, washboard makes the wheels hop, and a
rock lifts the wheel or, if loose, is shoved aside. The Town01 example puts a rough gravel
road with rocks on the straight at y -131, x 95 to 345, which `town01_ego_drive`'s route
runs along:

```bash
R=$SWB/src/swb_isaac_world/spikes/road_town
cp $R/road_detail.yaml $R/surfaces.yaml $CARLA_MAPS/Town01/
```

```yaml
corridors:
  - centreline: [[345.0, -131.1], [95.0, -131.1]]   # map frame; z optional
    seed: 66                       # the same seed, the same road
    roughness: {cls: C}            # ISO 8608 class A (smooth) to H
    section: {half_width: 3.0, crown: 0.03, shoulder: 0.5}
    ruts: {}                       # two ruts and a windrow, defaults
    potholes: {per_100m: 2.5}
    washboard: [{u0: 120, u1: 170, height: 0.02, pitch: 0.7}]
    water_bars: {at: [60, 200]}    # m along the centreline
    rocks: {carriageway: 0.01, shoulder: 0.01, ditch: 0.03}
```

| Key | Fields (defaults) |
|---|---|
| `roughness` | `cls` (D), or `gd0` in m^3; `w` 2.0, `n_break` 2.0 cycles/m, `w_high` 1.5 |
| `section` | `half_width` 2.5, `crown` 0.04, `shoulder` 0.5, `foreslope` 1/3, `ditch_depth` 0.30, `ditch_bottom` 0.6, `backslope` 0.5 |
| `ruts` | `depth` 0.03, `width` 0.30, `offset` 0.80, `windrow` 0.02, `windrow_width` 0.5 |
| `potholes` | `per_100m` 2.0, `depth` 0.015 to 0.08, `diameter` 0.3 to 1.0, `edge` 0.03, `sag_weight` 3.0 |
| `washboard` | a list of zones: `u0`, `u1` (m along), `height` 0.02, `pitch` 0.7, `skew` 0.3 |
| `water_bars` | `height` 0.06, `width` 0.25, `angle` 35, `at` (empty: spaced by grade) |
| `rocks` | rock cover per part of the section: `carriageway` 0.002, `shoulder` 0.01, `ditch` 0.03; `embed` 0 to 0.4 of a rock's height buried; `carriageway_protrude` 0.05 m, `carriageway_d_max` 0.30 m |
| `taper` | m over which the detail fades in and out at the ends |

Rocks run from 9 cm across (smaller is the `gravel` class) to 1.5 m; one buried more than
15 %, or over 50 kg, is static, the rest are loose rigid bodies. They are placed per 16 m
tile from the seed and streamed around each car.

Before you rely on it:

- **Autoware on the rough road is not validated yet.** The detail is not in the Autoware
  map's point cloud, and nobody has measured the ground filter or NDT over washboard, ruts
  and rocks. Use it with a scenario or free drive as above, and expect the autopilot side
  to need work.
- **It costs frame time.** On 2026-11-07, one ego with every sensor on Town01 ran at 40.5 ms
  a frame (median) on the rough road against 22.0 ms on the flat one, on a shared GPU; on
  2026-11-14, with an idle GPU, the road's own share was 5 to 7 ms a frame. The world is
  stepped in lockstep with the scenario, so a slow frame slows the clock rather than losing
  physics.
- Wheel hop (the wheel's own mass on the tire's spring), the road's bank and its edges are
  not modelled.

Without Autoware, the scripts drive these stages directly, headless (run as in
section 4; each prints one `SPIKE-RESULT` line in its log):

| Script | Stage |
|---|---|
| `rough_road.py` | 2 km of class C gravel road with ruts, potholes, washboard and water bars |
| `rock_field.py` | the same road with rocks |
| `multi_ego_road.py` | Town01 with `road_town/road_detail.yaml`, one or two egos, every sensor (`ROAD_TOWN=road_town/flat/surfaces.yaml`: the flat baseline) |
| `rut_town.py` | crossing old mud ruts on Town01 at a shallow angle |
| `limit_mud.py` | braking, split-grip braking and a steering step on 50 m of mud on Town01 |

The town scripts read Town01's world from `data/carla-autoware-bridge/Town01/isaac/world.yaml`
in the checkout; `procedural/README.md` in the dataset lists every stage and its switches.

## 8. The car's computers on low grip

The Model 3 on Isaac brakes through ABS and limits wheelspin with traction control, both
on, working from the car's own wheel speeds and accelerations, as a real brake module does.
Stability control (ESC) is off unless the ego config turns it on. Write an ego config that
uses the same car with ESC on:

```bash
AIB=$SWB/src/swb_isaac_world/deps/autoware_isaac_bridge
cat > ~/ego_esc.yaml <<EOF
rig: $AIB/src/aib/config/tesla_model3.rig.yaml
domain: 1
node_name: isaac_vehicle
vehicle: {}
ecu: {esc_enabled: true}
sensors: {}
EOF
```

and pass `ego:=$HOME/ego_esc.yaml` to `isaac.launch.xml`. `ecu:` takes any field of
`sim_vehicle_ecu`'s `Calibration` by name (for example `abs_slip_release: 0.3`). The ego
domain still comes from `EGO_ROS_DOMAIN_ID`.

## 9. What to watch

The world server publishes no wheel or soil topics. What it gives:

- **Its log** (the stack's web UI lists the node `swb_isaac_world` with its log). Each wheel
  that moves onto other ground logs one line: its position, class, wetness, water depth,
  zone and friction. The road condition, each soil zone, the ruts loaded and saved, and
  the warnings about flagged classes are logged too.
- **A recording**: the world server's `record:=<dir>` writes, for the first ego of each
  scenario, dash, wheel, chase and course pictures and `state.jsonl`, one line a frame:
  position, attitude, speed, acceleration, the command, and per wheel `class`, `zone`,
  `mu`, `slip`, `slip_angle_deg`, `load_n`, and in soil `sinkage_cm` and `resistance_n`,
  plus what ABS, TC and ESC estimated and did. `isaac.launch.xml` does not take it; give it
  through `stack.launch.py`:

```bash
W=$CARLA_MAPS/Town01/isaac/world.yaml
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch stack.launch.py \
    sim:=isaac \
    sim_args:="world:=$W render:=headless record:=$HOME/road_record" \
    map_path:=$CARLA_MAPS/Town01
```

  Recording costs frame time; keep it off for runs that are judged.
- **The ego's ROS topics** in the ego domain, as on any run: `/vehicle/status/*` for speed
  and steering, Autoware's own for what it planned.

## 10. Limits

| Not there | Today |
|---|---|
| CARLA | the same `RoadCondition` scales every vehicle's tire friction (`frictionScaleFactor` times asphalt's wet factor) and makes the road look wet; no zones, soil, ruts or rough road, and `swb.` properties are refused with that reason |
| A scenario choosing its surface map | one `surfaces.yaml` per map directory, copied in before the run |
| Deforming ground outside zones | soil only in zones; elsewhere a grip model per class |
| Soil on a slope with ruts | `ground_z` tiles are flat; a zone without it runs the soil without rut geometry |
| NPCs feeling the soil | NPC vehicles leave ruts but are not slowed or sunk by it |
| Validated mud and snow | `mud_wet` sinkage and `snow_loose` traction are flagged; hydroplaning is an engineering model |
| Rough road with Autoware | runs, not validated (section 7) |
| Wheel hop, bank and crown of a town's road, road edges | not modelled |
| Visible water, splashes, dust | none; zones are physics only |

Ruts cost memory: PhysX keeps every collision mesh it builds, about 100 MB per km of tracks
in a soil zone with its own ground, for as long as the world server runs.

## Next

- [Configure the system](04-configure.md): the world server's other arguments.
- [Run and judge](02-run-and-judge.md): run the surface scenarios as a suite.
