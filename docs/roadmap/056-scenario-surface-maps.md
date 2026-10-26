# Phase 056: The Scenario Chooses Its Ground

Let a scenario say which ground it runs on, through the standard `RoadCondition` it already
uses: which of the map's surface maps (`swb.surface_map`), and which class each ground label
takes (`swb.default.<Label>`), so that an off-road scenario sets the ground beside the road
as well as on it, and several scenarios share one map directory without a file copied in
before each run.

**Status**: planned, 2026-10-27.

**Source**: user direction 2026-10-27: "I'd like to see how we can specify the road
attributes in openscenario and how our swb can configure it. For the off-road project, I
would also configure ground attributes out of road area. please check if it's possible",
then, on the answer, "write up the swb.surface_map and swb.default.<Label> scenario
properties as a roadmap item".

**Depends on**: [047](047-off-road-surfaces.md) S2 (the road condition from SSv2 to the world
server, `swb.zone.<id>.<field>`), D5 (`swb.ruts`). **Feeds**: [040](040-ntu-offroad-field.md),
[041](041-competition-director.md), every soil scenario (`scenarios/surfaces/`).

## What there is today

[design/off-road-surfaces.md](../design/off-road-surfaces.md), "Road condition from the
scenario". The map says where, the scenario says what state:

- **Where**: `<map>/surfaces.yaml`, read by the Isaac world server at every `LoadWorld`:
  `defaults` gives each ground label (`Road`, `RoadLine`, `SideWalk`, `Terrain`, `Ground`,
  from the town's own meshes) a class, so the ground beside the road already has
  attributes; `zones` are polygons anywhere, on or off the road, with a class, wetness, water
  depth, and optionally soil (`ground_z`: the zone's own ground, ruts in the collision).
- **What state**: OpenSCENARIO's `RoadCondition` in an `EnvironmentAction` (in the Init, or
  on a trigger in the Story): `frictionScaleFactor` and `wetness` world-wide;
  `swb.zone.<id>.class|wetness|water_depth` for one zone; `swb.ruts` for a saved rut set.
  The SSv2 fork sends it, the bridge passes `swb.` properties to a world server that offers
  `CAPABILITY_ROAD_CONDITION`, the Isaac world server checks every property against the map
  and refuses the whole condition on any problem. The CARLA world server scales tire
  friction and refuses `swb.` properties (no zones on CARLA).

What a scenario cannot do:

1. **Choose the surface map.** There is one per map directory, so each soil scenario's
   ground has to be copied to `<map>/surfaces.yaml` before its world server starts (as 053
   and 054's runs did, with a script that copies and removes it), and two scenarios with
   different ground cannot run one after the other on one world server.
2. **Set a ground label's class.** "The terrain beside the road is mud in this scenario"
   needs a map file of its own.

## Design

### `swb.surface_map=<name>`

- **Names, not paths.** The world server may run on another host than the scenario; the
  value is a name (`[a-z0-9_-]+`), looked up as `<map>/surfaces/<name>.yaml`. The map's own
  `surfaces.yaml` stays the default when no scenario names one.
- **When.** Like `swb.ruts`, only while no wheel has driven on any soil since the world was
  loaded (in practice: an `EnvironmentAction` in the Init). The world server loads the named
  map as `_load_surfaces` does at a `LoadWorld`: the zones' tiles built, the town's ground
  cut out under a `ground_z` zone and the cut settled, the previous map's tiles removed and
  its cut put back. Named later, it is refused with the reason.
- **The rest of the condition applies to the named map.** In one `RoadCondition`,
  `swb.zone.<id>.*` and `swb.ruts` are checked against the map it names (rut sets live
  beside it: `<map>/surfaces/<name>/ruts/`), and every problem is listed together.
- **Lifetime.** As every road condition: the next `Initialize` (`LoadWorld`), a new owner and
  a release go back to `<map>/surfaces.yaml`. A world server keeps the maps it has loaded
  parsed, so naming one again costs the tiles and the cut only.
- **Where the files come from.** The scenarios' surface maps stay in this repository
  (`scenarios/surfaces/<name>.yaml`); `just surfaces-install [map_path:=<dir>]` links them into
  `<map>/surfaces/` (symlinks, so an edit needs no reinstall), and `just surfaces-check`
  checks every map there, not only `surfaces.yaml`. The copying in the run scripts goes.

### `swb.default.<Label>=<class>`

- One ground label's class for this scenario: `swb.default.Terrain=mud_wet`,
  `swb.default.SideWalk=gravel`. Labels as `surfaces.yaml`'s `defaults` (`Road`, `RoadLine`,
  `SideWalk`, `Terrain`, `Ground`, `unlabelled`); the class must be the table's. A label is
  friction-only: a soil class gets its friction, not its soil (no grid under a whole label;
  a soil zone is the way to put soil off the road). Zones still take precedence where they
  are, as in the map.
- Checked with the rest of the condition; refused on CARLA with the other `swb.` properties.

### Not in this phase

- A zone made or moved by the scenario (a polygon in a property): the map's job, and a soil
  zone's tiles and cut are built per map.
- Soil under a whole label: the grid's memory and tiles are per zone and bounded; the
  town's terrain is kilometres.
- Surface attributes from the lanelet2 map (a lane's `surface` tag): lanes only, never the
  ground beside the road.
- CARLA: no ground zones there (one friction per wheel, no road model).

## Steps

- [ ] **1. Named maps.** `surfaces.road_condition` takes `swb.surface_map`; the surface layer
  loads `<map>/surfaces/<name>.yaml` before any wheel drives on soil, refuses it after; the
  zone and rut properties checked against it; back to the default at a `LoadWorld`, a new
  owner, a release. Unit tests (a named map, an unknown one, after driving, the zone
  properties against the named map, back to the default).
- [ ] **2. Label classes.** `swb.default.<Label>`: checked, applied, friction only. Unit tests.
- [ ] **3. Tooling.** `just surfaces-install`, `just surfaces-check` over `<map>/surfaces/`; the
  soil scenarios (`scenarios/surfaces/*.xosc`) name their maps in their Init, and the run
  scripts stop copying.
- [ ] **4. Docs.** design/off-road-surfaces.md's road-condition section; the user guide.
- [ ] **5. Runs.** On Isaac with Autoware, one world server, no restart between them: the
  Town01 soil scenarios one after another (053 step 5's set), each naming its map; and one
  scenario setting `swb.default.Terrain` beside a road it leaves.

## Acceptance

- [ ] A scenario names its surface map and gets it; one that names a missing map, or names
  it after the car has driven on soil, is refused with the reason; the next scenario on the
  same world server gets its own.
- [ ] `swb.default.<Label>` sets a ground label's class for one scenario (a soil class as its
  friction only); an unknown label or class is refused.
- [ ] The Town01 soil scenarios pass one after another on one world server, no file copied.
