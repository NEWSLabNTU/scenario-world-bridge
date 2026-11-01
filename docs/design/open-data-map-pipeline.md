# From Public Data to a Driven Map: the Pipeline

This document lists the steps that take a site from nothing to an Autoware map directory and
an Isaac world that Autoware drives. It is roadmap 062's pipeline, as run on the NTU box, and
the scripts are in `scripts/open_maps/`.

Each step says what it does and which of three kinds it is:

- **auto:** a command that needs no judgement;
- **check:** a person looks at a picture or a number and decides;
- **manual:** a person picks something or edits something.

Each step also says what has gone wrong there so far. The background is
[`survey-open-data-maps.md`](survey-open-data-maps.md), and the progress and the measurements
are in [roadmap 062](../roadmap/062-ntu-roads-from-open-data.md).

## At a glance

| # | step | kind | command |
|---|---|---|---|
| 0 | tools and host settings | manual, once | `uv venv data/maps/.venv`, `.env` |
| 1 | the site file | manual | `sites/<site>.yaml` |
| 2 | fetch the sources | auto | `fetch.py` |
| 3 | baseline map (route A, comparison only) | auto | `baseline.py` |
| 4 | generate the Lanelet2 map (route B) | auto | `generate.py` |
| 5 | score it | auto, with check | `check_map.py`, `route_matrix.py` |
| 6 | look at it | check | `plot_map.py` |
| 7 | audit against ground truth | auto, with check | `overlay.py`, `audit_*.py` |
| 8 | the map directory and the world | auto | `world.py meshes`, `world_usd.py`, `world.py pcd` |
| 9 | plan a drive | manual, with a tool | `route_plan.py` |
| 10 | write the scenario | auto, from step 9's picks | `scenario.py` |
| 11 | drive it in simulation | auto, with check | `just isaac-world`, `just run`, `just ego-isaac`, `just scenario` |
| 12 | diagnose a failed drive | manual | the ego's topics, the lanelets where it stopped |

Steps 2-8 and 10 rerun without a person. Steps 1 and 9 are a person's choices. Steps 6, 11
and 12 are where problems are found. Every generator fix so far came from step 6 or step 12,
not from a number in step 5.

## 0. Tools and host settings (manual, once)

- **The tool venv:** `uv venv data/maps/.venv`, then install duckdb, eclipse-sumo,
  commonroad-scenario-designer, pyproj, shapely, requests, pyyaml, pillow, scipy,
  matplotlib and pyshp into it. Every script runs with `data/maps/.venv/bin/python`, except two:
  `route_matrix.py` and `route_plan.py` need Autoware's Lanelet2 (`source
  /opt/autoware/1.5.0/setup.bash`, system Python), and `world_usd.py` needs Isaac's Python.
- **Host settings, in `.env`:**
  - `MAPILLARY_TOKEN`, a client token from mapillary.com/dashboard/developers (only the
    token, not the secret);
  - `SWB_OPEN_MAPS`, the data root: a scenario names its map as
    `$(env SWB_OPEN_MAPS)/<site>/map`;
  - `ISAAC_KIT_ARGS`, set only when a GPU must be avoided (on newslab-118-server GPU 1 is
    another user's, so Isaac is pinned to GPU 0).

## 1. The site file (manual)

`scripts/open_maps/sites/<site>.yaml` holds five things:

- **The box,** south, west, north and east in WGS84.
- **The frame:** the UTM zone, the MGRS grid square and its offset, which is the map frame
  Autoware's MGRS projector uses, and `map_origin`, which the Isaac world subtracts. On a site
  with a survey, use the survey's frame. NTU's is 059's.
- **Landmarks:** a road name and a point near it, 8 for NTU. `route_matrix.py` checks that
  every landmark routes to every other.
- **Speed zones:** an OSM area and a limit that holds everywhere inside it. Such a rule is
  local knowledge that no tag carries. NTU's campus is 15 km/h ("everywhere in the school",
  from the user). Name the source.

How to choose the box is a judgement. A box too small cuts roads at its edge, and every cut
lane is a dead end or has no entry. The survey's measurement and roadmap 062 explain why NTU
was chosen.

## 2. Fetch the sources (auto)

    data/maps/.venv/bin/python scripts/open_maps/fetch.py scripts/open_maps/sites/ntu.yaml data/maps/ntu

The five sources go into `<out>/raw/`, each cached: OSM by Overpass, Overture by DuckDB from
its S3 release, Mapillary's map features and image metadata, NLSC's 0.27 m orthophoto at
z19, and Taipei City's open data (only for a box in Taipei). Use `--force` to fetch again.

Taipei's (data.taipei, 政府資料開放授權條款 v1, free with attribution), each cut to the box:

| file | data set (agency) | used for |
|---|---|---|
| `taipei_pavement.geojson` | 臺北市道路銑鋪區塊圖資 (新工處): the road surface curb to curb, per direction, with `WIDTH` | lane widths and their place across the road |
| `taipei_parking.geojson` | 臺北市路邊停車格位 (停管處): each curb parking space | taken off the road before the lanes are fitted |
| `taipei_signs.json` | 臺北市標誌資料 (交工處): the signs of 34 arterials | turn bans |
| `taipei_signals.json` | 臺北市路口號誌時制計畫 (交通局): each signal's plans (phase green, yellow, all-red; cycle; offset) and its day schedules, updated daily | `signal_plans.json` |
| `taipei_phases/<device>.pdf` | the same signal's 三合一報表: its phase diagrams, as printed 2024-03 | the signal audit (step 7) |

What Taipei's data does not have: lane lines and per-lane arrows (its marking data set holds
crosswalks and yellow boxes only, and the 1/1000 topographic map, which has no markings either,
is sold), viaducts in 3D (the Ministry of the Interior's 3D road model is sold), and anything
inside the campus. The sign register's road names and its direction field (every sign says
往北) are not the signs'; only their positions are used.

Problems seen: Mapillary's `layers` parameter does not filter, so both layers came back the
same and are now deduplicated. Its images endpoint refuses a large box, so the box is fetched
in tiles. Nothing here needs a person unless a service changes.

## 3. The baseline, route A (auto, comparison only)

    data/maps/.venv/bin/python scripts/open_maps/baseline.py scripts/open_maps/sites/ntu.yaml data/maps/ntu

This is the off-the-shelf route: OSM, then SUMO netconvert, then OpenDRIVE, then CommonRoad
Scenario Designer, then Lanelet2. It exists to be beaten, and nothing downstream uses it.
Making it run at all took workarounds:

- crdesigner's CLI is broken, so the script calls its API;
- its signal ids were strings, so they are renumbered;
- it ignored the header's offset, so a tmerc georeference is set.

## 4. Generate the map, route B (auto)

    data/maps/.venv/bin/python scripts/open_maps/generate.py scripts/open_maps/sites/ntu.yaml data/maps/ntu

This writes `<out>/generated/`: `lanelet2_map.osm`, `map_projector_info.yaml`, `report.json`
(the build's counts), `bans.json` (each turn banned by a sign, with its evidence and source)
and `signal_plans.json` (Taipei's timing plans on the map's lights). The generator's docstring
describes what it builds:

- lanes from OSM's lane tags or the road class's defaults, as wide as Taipei's pavement less
  its parking spaces allows (`pavement.py`);
- junction paths with `turn_direction`, filtered by `turn:lanes`, Mapillary's arrows and the
  no-turn signs (Taipei's register where it has one on the approach, else Mapillary's);
- speed limits from OSM, Overture or Mapillary's signs, else by rule;
- traffic lights with stop lines;
- crosswalks;
- for every attribute, where it came from (`swb:source`, `swb:speed_source`,
  `swb:turn_source`, `swb:width_source`).

This step has no person in it. When a check below finds a problem, the fix goes into the
generator as a rule, and the map is generated again; nothing is edited by hand in the output.
The audit's corrections (step 7) are the exception: the movements a person reads off the paint,
per approach, in `sites/<site>_audit.yaml`, which the generator applies over every other source
(`swb:turn_source=audit`). It also writes `generated/audit_applied.json`: corrections it could
not apply, and approaches a ban left with a lane that has nowhere to go, for the audit to look
at.

Rules added so far, and the step that found each:

| problem | found by | rule now |
|---|---|---|
| ways split at tag changes left gaps (joints) | step 6 | joints get connectors |
| a sign or arrow beyond a line's end counted at the end | step 6 | projections are clamped and checked |
| stubs, bus-only ways and viaduct ramps stranded lanes | steps 5, 6 | dropped (`stub_chains_dropped`, `viaduct_ramps_dropped`) |
| two lanes merging into one at a 40 deg joint: the connector's bounds crossed, and Autoware's path ended there | step 12 (run 7) | a joint cuts 10 m when lane count or width changes, 6 m when it bends over 20 deg |
| a lane jogging sideways across a junction: the connector pinched to 1.5 m and its bounds crossed (19 connectors) | step 5's crossing count, after run 7 | both bounds are offset from one centre curve; separate curves only where the offset would loop |
| a driveway leaving 辛亥路 east and turning south 2 m later kept a 3 m-radius corner in its own lanelet; Autoware stopped at it | step 12 (loop run 10) | a corner within the junction's radius + 6 m of its node moves the cut past it, so the connector takes the turn |
| the same driveway turns 90 deg again in the middle of its way: a lanelet with a sharp corner has no path a car can follow | step 12 (loop run 11) | each chain's corners over 20 deg are rounded, radius 8 m or what the segments beside them allow (142 on NTU) |
| that corner rounded to 2 m only, because OSM has a node 3.9 m past it | step 12 (the loop at 15 km/h) | nodes in a straight run dropped before rounding; a fillet may use a whole segment unless the next vertex is a corner too |
| the main gate's right turn at a 2.8 m radius, two carriageways cut from different nodes; the path optimiser stopped the ego in it | step 12 (the same run) | the cut leaves the outer lane's right turn 6 m; then the map is generated again, up to 6 passes; `check_map.py` reports the turn-path radii |
| 105 turn paths still under 4 m: a cubic with equal handles bends at whichever end is nearer the corner, so where one end was 2 m from it and the other 15 m the path hooked | step 5's radius count, plotted in step 6 | a turn is a straight, the widest arc its two tangents allow, and a straight; only the chain end nearer the corner is cut further in the next pass (cutting the whole junction kept the hook) |
| the rest: the near end was a 2 m stub, a short road between two junctions (or a junction and a joint) cut from both ends, which no further cut can lengthen; also the "2 m pieces" | the same | two junctions whose cuts would leave under 6 m of road between them are one junction, and so is a joint that close to a junction (31 and 40 on NTU); 26 under 4 m remain, mostly alley to alley |
| signal detection reached from the cluster's centre by its cut, which grew with the passes: lights came and went with them | step 5 (the light count) | a signal within the cut + 20 m of any of the junction's nodes |
| no U-turns | the campaign's list | a U-turn from the leftmost lane wherever its half circle has a 4.5 m radius into some exit lane, unless OSM or a Mapillary sign bans it (63 on NTU) |
| 新生南路 bends 45 deg at the main gate: going on was a turn, and two no-turn signs there then sent all four lanes of both directions into 椰林大道 | step 7, the junction audit | a road that bends under 60 deg at a junction, onto the exit of its own name, goes straight on |
| no loop through 公館: the box stopped 40 m short of 基隆路 x 羅斯福路 | the campaign's list | the box goes south to 25.0095 |
| every lane 3.0-3.5 m wide by class, on OSM's centre line: 新生南路's paint says 3.2 m, and OSM's line need not be the road's middle | the campaign's list | Taipei's pavement blocks less its curb parking spaces give each road piece its edges; its lanes fill them, 2.6-4.0 m each, centred (425 of 825 pieces on NTU; 238 capped at 4 m, the rest of the road left at the edges for scooters; 114 whose cuts disagree by over 2 m keep the default) |
| crossings at a junction's mouth came and went with the cuts, which follow the roads' widths (62 to 56 when the widths changed) | step 5's crosswalk count | a crossing is kept when it crosses a road anywhere over its full width and length, junctions included (81 on NTU) |
| a city signal matched only to the nearest map junction: a dual carriageway's other half, a map junction of its own, had no light in the plan, and the diagrams drew movements with no light to give them to | the signal audit | a device controls every signalised map junction within 45 m of it |
| a road passing a signal with no junction of its own got no light at all (新生南路's northbound half at the T of 56巷; the city's mid-block crossings, 公館國小, 民族國中) | the signal audit | a road within 30 m of a city signal (12 m for a minor one), more than 15 m from its ends and not leaving a junction within 35 m of the signal, is split there, and the joint gets a stop line 8 m before it and a light (12 on NTU); a map junction within 25 m of a city signal is signalised (9 more) |
| each road piece now has its own sideways shift, and a joint's 4 m connector carried the lanes 2.4 m sideways: the ego stopped there on 辛亥路 | step 12 (the loop on the new widths) | a joint whose lanes jog sideways is cut 3.5 m along per metre across, up to 15 m (43 on NTU) |
| a scenario's waypoint, picked on one generation, fell off the road on the next once the lanes moved; SSv2 refused it | step 11 | `scenario.py` snaps every position to the nearest lane centre in its direction |
| turn bans only from Mapillary's detections, which once sat on the wrong approach (the main gate) | the campaign's list | Taipei's sign register where it has a sign on the approach, else Mapillary; where they disagree, `audit_applied.json` lists it (`signs_disagree`, 5 on NTU) |

Route C, lane widths fitted to NLSC's photo (`imagery.py`), is off. Its fits disagreed between
neighbouring pieces of one road, and the survey had too little paint on the arterials to
validate it. `SWB_IMAGERY=1` turns it on.

## 5. Score it (auto, with a check)

    data/maps/.venv/bin/python scripts/open_maps/check_map.py data/maps/ntu/generated/lanelet2_map.osm
    source /opt/autoware/1.5.0/setup.bash
    python3 scripts/open_maps/route_matrix.py data/maps/ntu/generated/lanelet2_map.osm scripts/open_maps/sites/ntu.yaml

`check_map.py` counts what Autoware needs:

- junction paths with `turn_direction`, and road lanelets with `speed_limit`;
- traffic lights with stop lines;
- `local_x`/`local_y` on every node;
- dead ends, lanelets with no predecessor, and stranded lanes (no way in from a way into the
  map, or no way out).

`route_matrix.py` routes every landmark to every other with Lanelet2. On NTU today:

- road lanelets: 3673;
- junction paths: 1616, every one with `turn_direction`;
- speed limits: on every lanelet;
- traffic lights: 145, each with a stop line;
- routing: 56 of 56 landmark pairs;
- stranded lanes: 320 with no way in and 332 with no way out, most at the box's edge.

**The check:** compare the numbers with the last run's. A rule that loses junction paths or
strands lanes shows here first. Example: the first version of the corner rule cost 9 junction
paths and 7 stranded lanes; step 6 showed why, and the rule was narrowed.

Two numbers mislead. The largest strongly connected part is distorted by the box: a lane that
leaves the box never comes back, which is why stranded lanes replaced it. And a high count can
still hide a bad junction, which only steps 6 and 12 find.

`check_map.py` also counts road lanelets whose left and right bounds cross (`bounds_crossing`),
which must be 0: Autoware's path ends in one. It was 19 before the paired connectors.

## 6. Look at it (check)

    data/maps/.venv/bin/python scripts/open_maps/plot_map.py data/maps/ntu/generated/lanelet2_map.osm out.png [--box x0,y0,x1,y1]

The plot draws road lanelets in grey (in the largest connected part) or red (outside it),
junction paths coloured by turn, dead ends, lanelets with no predecessor, and stop lines.

**The check is by eye,** junction by junction, zoomed in with `--box`:

- does every approach have its turns;
- do the paths look like a car's;
- is anything isolated that should not be?

Plotting the same box from two generator versions side by side is how a rule's side effects
are found. That is how the corner rule's loss on a one-way service loop was seen.
`SWB_DEBUG_AT=x,y` makes the generator dump the junction nearest that point.

## 7. Audit against ground truth (auto, with a check; needs 059's data)

    overlay.py      # the map's bounds over 059's 2 cm orthophoto, for the eye
    audit_lines.py  # lane bounds against the paint, per kind of line
    audit_sections.py  # lane counts and line positions, road piece by road piece
    audit_vendor.py    # lane centres against the vendor's surveyed lanelets

The ground truth is 059's mobile-mapping survey and its orthophoto, and the vendor's
lanelets. They are on the NAS, so the ssh master must be open. They are never an input to the
map. Results so far: the vendor's lanes are 68.5 % covered, at a median lateral offset of
0.97 m.

**The junction audit (manual, with a tool):**

    scripts/open_maps/audit_junctions.py <map> <ortho dir> <out> [--all] [--nlsc <site.yaml> <raw dir>]

It draws each signalised junction (`--all`: every junction) with each approach lane's
movements written at its end (L, S, R, U) over 059's 2 cm photo, or over NLSC's 0.27 m one
with `--nlsc`. 059's survey covers two signalised junctions only, and their arterial
approaches are at its edge, blurred; the campus roads it covers well carry almost no arrows.
NLSC's photo covers all 44, and an arrow shows on it as a white mark, but at 0.27 m a person
can tell a straight arrow from a turn only where it is clear of trees, shadows and vehicles.
A person writes what the paint says in `sites/<site>_audit.yaml`; what was looked at goes in
its `checked:` list. `generated/audit_applied.json` lists the approaches to look at first.

**The signal audit (manual, with a tool):**

    scripts/open_maps/audit_signals.py <site.yaml> <out> <sheet dir>

Taipei publishes each signal's timing (`signal_plans.json` has it on the map's junctions) but
not which approaches move in which phase: that is only drawn, in each signal's 三合一報表.
The sheet puts each junction on NLSC's photo, its approaches labelled by heading and road,
beside the report's phase diagrams. A person writes per device which approaches move in each
phase (`signals:` in the audit file, for the phase order code the diagram is drawn for), and
`scenario.py --signal-plans` cycles those junctions' lights as the city does. The reports
were printed 2024-03 and a third of today's plans use a phase order they do not draw
(`sheet.json`: `orders_now` against `orders_drawn`); a plan whose order was not read keeps a
fixed light. On NTU (2026-11-01): 46 of the box's devices are on the map's lights; the
diagrams of 30 were read (47 device and code pairs, `signals:` in `ntu_audit.yaml`); at
Monday 10:00, 24 devices cycle; 9 more have readings but keep a fixed light then, because
that hour's plan uses a code their report does not draw; 13 have no reading: six whose reports draw
none of today's codes (the main gate, 羅斯福路 x 新生南路, is one), 公館's ring (stop lines
inside it the map lacks), one that draws only flashing, and signals matched after the reading
(mid-block crossings) or with no report.

## 8. The map directory and the world (auto)

    data/maps/.venv/bin/python scripts/open_maps/world.py <site.yaml> <out> meshes
    OMNI_KIT_ACCEPT_EULA=YES $ISAAC_ROOT/venv/bin/python scripts/open_maps/world_usd.py <site.yaml> <out>
    data/maps/.venv/bin/python scripts/open_maps/world.py <site.yaml> <out> pcd

The three must run in this order, because the PCD step samples the trees the USD step places.
Run them with the host settings loaded (`set -a; source .env; set +a`): `world_usd.py` finds
CARLA's trees through `CARLA_USD`, and without it the world comes out with no trees, the PCD
with none of their points, and no `isaac/world.yaml`, with no error (2026-11-01).
They write `<out>/map/`, which is everything `just ego-isaac map_path:=` and `just isaac-world
world:=` need:

- `lanelet2_map.osm`, a copy of `generated/`;
- `map_projector_info.yaml` (MGRS);
- `traffic_lights.yaml`, the signal table: without it the Isaac world server publishes no
  light states, and Autoware stops at the first light;
- `pointcloud_map/` in 50 m tiles with `pointcloud_map_metadata.yaml`: as one file,
  compare_map's voxel grid overflowed, and the tile corners must be integers or the loader
  fails;
- the PCD is cut at 30 m height: compare_map builds one voxel grid over the whole map,
  and with 114 m buildings it overflowed int32 and stopped the car;
- `isaac/world.usdc` and `isaac/world.yaml`.

The world itself:

- NLSC's photo on a flat ground;
- the road lanelets as colliders, with their paint;
- Overture's buildings, extruded;
- signal poles, Mapillary's street lamps, and OSM's trees as CARLA's.

No prop stands within 1 m of a road lanelet, because Autoware stops for a pole in a turning
path (run 8). Ten stop lines have no room for a pole within 8 m and get none. Of Mapillary's
2,251 lamps, 970 were on the carriageway and were dropped.

The world is believable, not a twin: the layout is the real one and the paint is the map's.
**Rerun all three after every generation**, then restart the world server and the ego stack:
the server loads the USD at start, and the stack loads the map and the PCD at start.

## 9. Plan a drive (manual, with a tool)

    source /opt/autoware/1.5.0/setup.bash
    python3 scripts/open_maps/route_plan.py data/maps/ntu/map/lanelet2_map.osm \
        52247,67661,180 52200,68205,0 52265,67900,-90 52350,67657,0

A person picks the points, using the plot from step 6 and the road names. The tool snaps each
point to a lanelet and prints the snapped pose, which is what `scenario.py` takes. It then
routes the legs as Autoware will and lists each leg's junction paths, with signalised ones
starred.

The choices:

- **The junction types to cover:** signalised and not; left, right and straight.
- **Which lights start red.**
- **The goal:** it must not lie on the route's first stretch, or the goal condition fires as
  the ego drives past it at the start.

`NO ROUTE` means a point needs moving.

## 10. Write the scenario (auto, from step 9's picks)

    data/maps/.venv/bin/python scripts/open_maps/scenario.py data/maps/ntu/map/lanelet2_map.osm scenarios/ntu/<name>.xosc \
        --name <story> --map-path '$(env SWB_OPEN_MAPS)/ntu/map' \
        --start x,y,h [--via x,y,h ...] --goal x,y,h [--red x,y ...] [--timeout 900]
        [--signal-plans data/maps/ntu/generated/signal_plans.json [--at "Mon 08:30"]]

The light ids change with every generation, so the scenario is always written again from the
map, never edited by hand. Every light gets a state in Init, green unless `--red` names it. A
red light turns green once the ego has stood still for 3 s before its stop line. Reaching the
junction beyond the line while it is still red is a failure.

With `--signal-plans`, the junctions whose phases the signal audit read cycle on Taipei's plan
for that weekday and time: each phase's green, yellow and all-red, started where the plan's
offset puts the cycle then, so a green wave along an arterial is kept (SSv2's
TrafficSignalController). A protected left is red with a left arrow. Lights elsewhere keep
their fixed state, and so does a junction with a `--red` light.

The scenarios so far:

- `ntu_gate_xinsheng.xosc`: one way, about 650 m, all lights green; passed.
- `ntu_campus_loop.xosc`: about 2.2 km of right turns, with two lights that start red;
  passed.
- `ntu_left_turns.xosc`: two signalised left turns, both starting red, then 羅斯福路 to
  公館; passed.
- `ntu_campus_loop_signals.xosc`: the loop with the lights cycling on Taipei's plans for
  Monday 10:00 and none held red.

## 11. Drive it in simulation (auto, with a check)

    just isaac-world world:=$PWD/data/maps/ntu/map
    just run sim:=isaac
    just ego-isaac map_path:=$PWD/data/maps/ntu/map max_velocity:=16.7
    just scenario scenarios/ntu/<name>.xosc sim:=isaac initialize_duration:=400 global_timeout:=1200 output_directory:=<dir>

`max_velocity` is the profile's cap, upstream's 15 km/h by default. At 16.7 m/s the map's
speed limits govern: the ego reached 12.2 m/s on 新生南路, which is tagged 50 km/h.

**The check:** pass or fail in `result.junit.xml`, and the turn signals: every junction path
the ego drove, its `turn_direction` against the indicator it showed there. While the run goes,
watch:

- the ego's pose and speed;
- the stop reason (the planning factors);
- NVTL along the way (3.3-3.4 on the loop so far, above the 2.5 limit).

## 12. Diagnose a failed drive (manual)

When the ego stops short, the work is:

1. **Read why:** the planning factors (`obstacle_stop` for something physical,
   `traffic-signal` for a light, a path that ends with no factor for a map problem).
2. **Find the lanelets:** Autoware's route segments, `/planning/mission_planning/route`.
3. **Read their geometry** at the stop.
4. **Trace it back** to the OSM ways and the generator's rule.
5. **Fix the rule, not the output,** then repeat steps 4-11.

Every drive failure so far, and its fix:

| run | stop | cause | fix |
|---|---|---|---|
| 1-5 | no start, or stops at the first light | GPU pin; no signal table; one-file PCD; float tile corners | `.env`; `traffic_lights.yaml`; divided PCD |
| 6-7 | 椰林大道, before the gate | merge connectors with crossing bounds | joint cut radius (step 4) |
| 8 | in the right turn at the gate | a signal pole inside the turning path | props kept off the road (step 8) |
| 9 | passed | | |
| loop 10 | turning off 辛亥路 into campus | a 3 m-radius corner left inside the driveway's lanelet | cut past corners (step 4) |
| loop 11 | at the driveway's next bend | a 90 deg corner mid-way, no fillet | corners rounded (step 4) |
| loop at 15 km/h, 1 | the driveway's bend again | a 2 m fillet (an OSM node next to the corner) | simplify, then fillet (step 4) |
| loop at 15 km/h, 2 | in the main gate's right turn | a 2.8 m turn path | junction cuts for a 6 m turn, regenerated per junction (step 4) |
| loop at 15 km/h, 3 | on 椰林大道, emergency stop | compare_map: "Given map voxel grid is not feasible", the whole PCD's extent over int32 at its voxel | the PCD stops at 30 m height (step 8) |
| loop at 15 km/h, 4; left turns | passed | | |

This step needs a person and is where most of the time goes. The tools are temporary
scripts: an ego watcher, an NVTL logger and a route probe. Making them a standing tool, a
`just` recipe that records pose, speed, stop reason and NVTL beside the scenario's result,
would make it cheaper.

## What a person still does, in short

- Once per site: pick the box, the frame and the landmarks (step 1).
- After a generator change: compare the scores with the last run's (step 5), and look at the
  junctions the change touches, side by side (step 6).
- Once per site, while the audit's arrow check has no detector: check the junctions against
  the orthophoto by eye (step 7).
- Once per drive: pick its points and its red lights (step 9).
- After a failed drive: diagnose it and turn the cause into a generator rule (step 12).

Everything else is a command, and the whole chain from `generate.py` to the driven scenario
reruns unattended once the picks are made.
