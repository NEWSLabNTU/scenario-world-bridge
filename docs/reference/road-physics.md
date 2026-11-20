# Road physics on Isaac Sim

What the Isaac world server models under the wheels, and every key of the files that set
it: the surface classes, a surface map's zones, the road detail, and what a scenario may
change. [Tutorial 7](../tutorials/07-road-physics.md) uses them. For developers, the
design is in [off-road surfaces](../design/off-road-surfaces.md).

## Layers

| Layer | What it does | Switched on by |
|---|---|---|
| Surface layer | each wheel's friction and tire curve from the ground under it, every physics step; wetness, standing water, hydroplaning | a surface map (`surfaces.yaml`) |
| Soil layer | in a soil zone: sinkage, traction limit, motion resistance, the soil against a sunk tire's side, ruts that later wheels ride in | a zone with `soil:` |
| Road detail | a rough road over the town's own: roughness, crown and ditches, ruts, potholes, washboard, water bars, rocks | `road_detail.yaml` |
| The car's computers | ABS and traction control (on), stability control (off by default), on the car's own sensors | the ego config of the world server |

All of it is physics. The world looks the same, except that a soil zone with its own
ground shows its ruts (and the lidar sees them) and the road detail streams a render mesh.

## Where the files come from

A testbench gives a map its ground in `testbench.yaml`, per map, with paths in the project:

```yaml
ground:
  Town01:
    surfaces: ground/town01.yaml
    road_detail: ground/town01_road.yaml     # optional
    surface_maps:                            # optional: maps a scenario names
      mud: ground/town01_mud.yaml
```

The world server reads a map's `surfaces.yaml` and `road_detail.yaml` from the testbench's
`state/<map>/` first, then from the map directory. When the stack starts it links the files
`ground:` names there (`state/<map>/surfaces.yaml`, `state/<map>/road_detail.yaml`,
`state/<map>/surfaces/<name>.yaml` for each of `surface_maps`); a file
you put there by hand under a name `ground:` also gives is refused, never replaced, so keep
your files in the testbench and name them. Without a surface map every
wheel runs on asphalt. The server checks each file before Isaac starts, refusing it with
every problem listed, and reads it again at every world load.

Check a file without Isaac:

```bash
ros2 run scensim_isaac_world surfaces check ground/town01.yaml
ros2 run scensim_isaac_world road_detail check ground/town01_road.yaml
```

Given a map directory, `surfaces check` also checks every named map in its `surfaces/`.

`surfaces plot` and `road_detail preview` draw them (matplotlib).

## Surface classes

`mu` is the dry peak tire-road friction; the Model 3's own tire on dry asphalt is the
reference (1.12), so a map whose roads are all asphalt drives exactly as a map with none.
The wet columns multiply `mu` at OpenSCENARIO's wetness levels; a level a class does not
list takes the next drier one's.

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
grip is left sliding past the peak (`sliding`), and how quickly (`relaxation`); ice's
sliding grip also falls with speed. The soil classes carry Bekker-Wong-Janosi parameters
and a `calibration:` line naming what they were fitted to:

| Soil class | Fitted to | Notes |
|---|---|---|
| `sand` | a road tire on dry loamy sand (He 2020) | drawbar pull and sinkage within tolerance |
| `sand_loose` | the same tire on loosened sand | a rear-drive car does not start from rest on it |
| `dirt` | a textbook sandy loam, fitted to Chrono SCM | no measured tire yet |
| `mud_wet` | a smooth tire on wet clay (Wismer 1966) | pull fitted; sinkage outside tolerance, so flagged |
| `mud_soft` | the same tire on wetter clay | within tolerance; a rear-drive car stalls in it |
| `snow_loose` | a tire towed through 15 cm of fresh snow (Richmond 1995) | traction not measured, so flagged |

A soil zone on a flagged class is refused unless the surface map lists the class under
`allow_unvalidated:`, and the world server warns at every load. Standing water lowers grip
on paved classes with speed, toward hydroplaning near 100 km/h at full depth with the
default 250 kPa tire: an engineering model, not validated.

## The surface map

A town's Isaac world keeps each collider's semantic label (Road, RoadLine, SideWalk,
Terrain, Ground). The surface map turns labels into classes and adds zones on top:

```yaml
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
| `ground_z` | with `soil: grid`: the zone's ground is the server's own flat tiles at this height, the town's ground cut out under them, so ruts are in the collision and the lidar; needs flat ground there, polygons on whole 1.6 m tiles |
| `polygon` | map frame, metres |

## What a scenario can set

A scenario sets the road's state through OpenSCENARIO's `RoadCondition`, in the Init or in
the Story (`EnvironmentAction`). The surface map says where; the scenario says what state:

```xml
<RoadCondition frictionScaleFactor="1.0" wetness="moist">
  <Properties>
    <Property name="scensim.surface_map" value="mud"/>
    <Property name="scensim.default.Terrain" value="mud_wet"/>
    <Property name="scensim.zone.puddle_left.water_depth" value="0.003"/>
    <Property name="scensim.zone.gravel_patch.class" value="dirt"/>
    <Property name="scensim.ruts" value="worn"/>
  </Properties>
</RoadCondition>
```

| Setting | Effect on Isaac |
|---|---|
| `frictionScaleFactor` | multiplies every wheel's friction |
| `wetness` | the whole road's wetness, where a zone sets none |
| `scensim.zone.<id>.class` | a zone's class (not a soil zone's: its class is its soil's) |
| `scensim.zone.<id>.wetness` | a zone's wetness |
| `scensim.zone.<id>.water_depth` | a zone's water depth, m |
| `scensim.ruts` | `fresh` or a saved set; in the Init: swapping ruts once a wheel has driven on the soil is refused |
| `scensim.surface_map` | the scenario's surface map, by name: `surfaces/<name>.yaml` of the map (the testbench's `surface_maps`, `state/<map>/surfaces/` first); in the Init: naming another once a wheel has driven on soil is refused |
| `scensim.default.<Label>` | one ground label's class for this scenario (`Road`, `RoadLine`, `SideWalk`, `Terrain`, `Ground`, `unlabelled`); a soil class gives its friction only, as soil goes under zones; zones stay on top |

Each `RoadCondition` is the whole road state: zones it does not name go back to the map's
values, and labels it does not name to the map's defaults. A named map stays until the next
scenario (its initialization, a new owner or a release go back to the map's own
`surfaces.yaml`), so a later `RoadCondition` in the Story need not name it again; its zone
properties and rut sets (`state/<map>/surfaces/<name>/ruts/`) are the named map's. A wrong
property (an unknown map, zone, label, class or field) fails the scenario with every problem
listed. Several scenarios with different ground run one after another on one world server,
each naming its map; nothing is copied between runs.

**Ruts.** The world server saves the ruts a run made to
`state/<map>/ruts/runs/<time>.npz` when the next run begins (each run's initialization
loads the world's state again, also on the same world), and logs the path; copied to `state/<map>/ruts/<name>.npz`
it becomes a set a scenario can start from (`scensim.ruts`). The
`isaac-road-physics-stages` dataset carries one, `worn`, for Town01
(`procedural/ruts/Town01/worn.npz`).

## The road detail

A `road_detail.yaml` lays a rough road along a centreline over the town's own road. The
tire's contact runs on it every physics step: a pothole drops the wheel and hits it on the
far edge, washboard makes the wheels hop, a rock lifts the wheel or, if loose, is shoved
aside. The `isaac-road-physics-stages` dataset's `procedural/spikes/road_town/` has one for
Town01's straight at y -131:

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

### The section along the road

The file is also the road record (roadmap 086): `schema: 1` at its top (absent means 1; a
newer one is refused, naming the update), and each corridor's cross-section either as the
`section` constants above or per station, in an `.npz` beside the file:

```yaml
schema: 1
corridors:
  - centreline: [[0.0, 0.0], [400.0, 0.0]]
    stations: pass.npz             # instead of section
```

The `.npz` holds `u`, the stations in m along the centreline (from 0, increasing, the last on
the centreline), and an array per field: one value per station for `half_width`, `bank` (rad,
positive leaning left), `crown_offset` (m) and `kappa_v` (1/m, positive in a sag), and two,
`<field>_left` and `<field>_right`, for a side's fields: `crown`, `shoulder`, `foreslope`,
`ditch_depth`, `ditch_bottom`, `backslope`, `lip_h` (m), `lip_angle` (deg), `runoff_len` (m),
`barrier_offset` (m, nan for none), the surface classes `shoulder_surface` and
`runoff_surface`, and `hazard` (`shoulder`, `runoff`, `lip`, `ditch`, `barrier` or `drop`). An
array left out takes its default everywhere. The same fields go into `section` as constants,
a side's as a number for both sides or `{left: .., right: ..}`. `road_detail keys` lists them
all with their defaults.

Today the world server shapes the road with one symmetric section: a record that is banked,
has a lip or an offset crown, or whose shape changes along the road or between its sides is
refused when the world loads, naming what differs (`road_detail check` notes it and draws the
first station's left side). The other fields (`kappa_v`, the surfaces, the barrier and the
hazard) are carried for the rails, the surface strips and the planner.

Rocks run from 9 cm across (smaller is the `gravel` class) to 1.5 m; one buried more than
15 %, or over 50 kg, is static, the rest are loose rigid bodies, placed per 16 m tile from
the seed and streamed around each car.

- **Autoware on the rough road is not validated.** The detail is not in the Autoware
  map's point cloud, and nobody has measured the ground filter or NDT over washboard,
  ruts and rocks.
- **It costs frame time.** Measured on 2026-11-14 with an idle GPU: 5 to 7 ms a frame for
  the road itself, one ego with every sensor on Town01. The world is stepped in lockstep
  with the scenario, so a slow frame slows the clock rather than losing physics.

## Watching the wheels

`/scensim/wheels`, on the ego's pilot domain: each wheel's ground and tire state as a
`diagnostic_msgs/DiagnosticArray`, sampled after a physics step at most every 0.05 s of
simulation time. One status per wheel, named `<ego>/<front_left|front_right|rear_left|
rear_right>`, its message the surface class, its values `class`, `zone`, `friction`,
`slip_ratio`, `slip_angle` (rad), `sinkage` (m) and `normal_load` (N).

```bash
eval "$(scensim seat env 0)"
ros2 topic echo /scensim/wheels --field status
```

A run at `--capture full` records it with the other key topics, into
`runs/<id>/record/<entity>/` ([Results and logs](results-and-logs.md)). The world server's
log also has one line per wheel that moves onto other ground (its class, zone, wetness and
friction), and the road condition, soil zones and ruts loaded and saved.

## Limits

| Not there | What there is |
|---|---|
| CARLA | the same `RoadCondition` scales every vehicle's tire friction and makes the road look wet; no zones, soil, ruts or rough road, and `scensim.` properties are refused with that reason |
| Deforming ground outside zones | soil only in zones; elsewhere a grip model per class |
| Soil on a slope with ruts | `ground_z` tiles are flat; a zone without it runs the soil without rut geometry |
| NPCs feeling the soil | NPC vehicles leave ruts but are not slowed or sunk by it |
| Validated mud and snow | `mud_wet` sinkage and `snow_loose` traction are flagged; hydroplaning is an engineering model |
| Wheel hop, bank and crown of a town's road, road edges | not modelled |
| Visible water, splashes, dust | none; zones are physics only |
| Stability control from a testbench | ESC is a field of the world server's ego config (`ecu: {esc_enabled: true}`), which `testbench.yaml` does not reach |

Ruts cost memory: PhysX keeps every collision mesh it builds, about 100 MB per km of tracks
in a soil zone with its own ground, for as long as the world server runs.
