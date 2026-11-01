# Phase 068: Viewer Camera and Lighter Paths

Small things that make a run easier to watch, to set up and to run cheaply, none of which
changes what a run measures. A design review on 2026-11-01 compared our stack against a
simpler, single-simulator layout and found five places where the simple layout does
something we do not, or does it with less: a spectator that follows the ego, walker feet
checked per model, sensors published from inside the simulator, a map whose lanelets keep
their real-world projection, and a one-page first run. Writing the comparison up showed three
gaps of our own (steps 7-9): CARLA refuses a scenario's road zones, CARLA 0.10 works only on
branches, and we need our SSv2 fork even for a plain run.

**Status**: phase doc, 2026-11-01; steps 7-9 added the same day. No code yet.

**Source**: user, 2026-11-01: "Document things we can learn ... as a phase doc."

**Depends on**: [032](032-npc-tracking-fixes.md) (walker control and ground snap, which
this phase checks rather than replaces), [031](031-maps-for-scenarios.md) (map packs, CARLA
frame), [018](018-install-and-world-config.md) (CARLA version as a Cargo feature),
[027](027-user-documentation.md) (tutorials). **Feeds**: [033](033-demo-tour.md) (the demo
tour's camera), [029](029-prebuilt-releases.md) (first-run time).

## Where it starts

| | today |
|---|---|
| Spectator | nothing moves CARLA's spectator; a person watching the CARLA window finds the ego by hand. The demo tour drives its own recording camera |
| Walkers | `WalkerControl` from the commanded velocity, a standing floor of 0.05 m/s, and the box bottom snapped to the ground (`swb_core::npc_tracking`, roadmap 032). Feet contact assumes the walker's bounding box encloses its feet; never checked per blueprint, never for the child models |
| Sensors | every sensor is a client-side CARLA sensor whose data acb receives and publishes (Rust, carla-rust), at the spec's rates and frames, on the ego's `/clock` |
| Maps | packs carry lanelets in the CARLA frame (projector origin zero); a lanelet map drawn in a real projection is regenerated, not offset |
| First run | tutorial 1: about 20 minutes, three terminals' worth of setup lines, a converted map directory first |
| Road condition | OpenSCENARIO `RoadCondition` reaches every world server declaring `ROAD_CONDITION`. Both do: on CARLA (047 S2, `swb_carla_world` `road_condition.rs`) it is a factor on every vehicle's tire friction (the scale times asphalt's wet factor) and the weather's wetness and puddles; the map's zones (`swb.zone.*`) are refused there, applied only by Isaac's surface layer |
| CARLA 0.10 | baseline passed 2026-10-11 (019) on branches `carla-0100` (acb on origin, csb local); main builds 0.9.14-0.9.16 only; release tier "source only" (029) |
| SSv2 | our fork, branch `managed-ego-unforked`: 26 commits, 44 files, +2038/-223 over its upstream merge base (2026-11-01) -- time base, unmanaged ego, agent and autopilot controllers, environment actions, bounded waits |

## Steps

### 1. Docs

- [ ] This phase doc; an entry in the roadmap README.

### 2. A spectator that follows the ego (CARLA world server)

The world server moves CARLA's spectator, since it is the only process that owns the world.

- [ ] `spectator: follow | once | off` and `spectator_height` (m above the target, default
      50) in `swb_carla_world.yaml`; default `off`, so a headless run does nothing new.
- [ ] Target: the entity whose motion is the ego's; with no ego, the first spawned vehicle,
      then the first entity. A despawned target moves the camera to the next.
- [ ] `follow` places it top-down over the target after every step; `once` places it on the
      first step after the target spawns.
- [ ] Hand-over: before each placement, compare the spectator's pose with the one the world
      server last set; a difference over 0.1 m or 1 deg means a person moved it, and the
      world server stops moving it until the next `LoadWorld`. Logged once.
- [ ] Cost: one `get_transform` and one `set_transform` per step; recorded in the frame
      stats. Acceptance: `town01_ego_drive` with `follow`, the frame-time median unchanged
      within noise; grabbing the camera mid-run leaves it where it was put.
- [ ] Isaac: the viewport camera, the same keys, later, in `swb_isaac_world`.

### 3. Walker feet, per blueprint

- [ ] A probe (`scripts/walker_feet_probe.py`): for every `walker.pedestrian.*` blueprint
      on CARLA 0.9.16, spawn on flat road, settle, and compare the bounding-box bottom with
      the lowest foot bone (`get_bones()`, where the build has it) and with a downward ray at
      the walker's position. Same on 0.10 when its branch is current.
- [ ] Table in `docs/measurements/`: blueprint, adult or child, bbox bottom error. A model
      more than 5 cm off gets a per-blueprint lift correction in the world server's walker
      table; none, if none are.
- [ ] Ground under the walker on a slope, a kerb and under a bridge deck: the snap finds
      the visible surface, not the deck above or the ground below.

### 4. Sensors from inside the simulator: measure first

CARLA can publish a sensor itself, inside the server (native ROS 2, `--ros2`, since 0.9.15;
to be confirmed on our 0.9.16 build and on 0.10). That skips the client round trip for the
two heavy sensors, lidar and camera. It also gives up what acb guarantees: the spec's
frames and rates, the ego's `/clock` rule (`nanos(elapsed) + BASE_NS`), one domain per
vehicle, and the sensor-release ordering.

- [ ] Measure on the Model 3 spec's sensor set, Town01, synchronous at 20 Hz: frame time
      (median, p95), acb CPU, end-to-end latency from the tick to the message in the ego's
      domain, for acb's path and for the native path, lidar and camera separately.
- [ ] List what the native path breaks: stamps against our `/clock`, frame ids, QoS,
      `ROS_DOMAIN_ID` per vehicle (a server publishes in one domain), several egos.
- [ ] **Decision point**: adopt per sensor (`publisher: acb | simulator` in the spec's
      CARLA block) only if it saves at least 20 % of frame time or of acb CPU and breaks
      nothing on the list, or the broken items have a fix. Otherwise record the numbers
      here and stop.

### 5. A map in its real projection

A lanelet map drawn in MGRS or UTM for a CARLA world that already matches it (a city twin
built from the same survey) needs only an offset, not a regenerated map.

- [ ] Check what the bridge, the world server and acb assume about the pack's projector
      (`map_projector_info.yaml`, `map_config.yaml`), and where an offset would have to be
      applied once: at the protocol boundary, so SSv2 and Autoware keep the real projection
      and the world server sees CARLA's frame.
- [ ] If it holds: `origin: {x, y, z}` in the pack's metadata, applied by the bridge on
      every pose in and out, refused when the projector says the map is already in the
      CARLA frame. Test: a Town map's pack re-expressed with an offset gives the same run.

### 6. First run on one page

- [ ] Time a first run from a fresh clone on a clean user account, step by step; the README's
      quickstart becomes the shortest path that passes `town01_ego_drive`, with tutorial 1 for
      the details. Target: one page, one `source` file, three commands after the build.

### 7. Road zones on CARLA

The road condition itself already works on CARLA (see "Where it starts"); what CARLA refuses
is a scenario's per-zone properties.

- [ ] `swb.zone.<id>.mu_scale` (and only that field) as a CARLA friction-trigger box over the
      zone, when the map's `surfaces.yaml` gives the zone a box; every other zone field still
      refused with the reason. Only if a CARLA scenario needs zones; until then, leave it.

### 8. CARLA 0.10 on main

- [ ] Land acb's `carla_0100` cfg and csb's 0.10 changes on main, behind the version feature
      (0.9.16 builds unchanged); a `carla-0100` CI build; the baseline scenario and its map
      pack documented in the user guide.
- [ ] 029's tier for 0.10: "built", then "tested" once a starter suite passes on it.

### 9. A plain run on upstream SSv2

- [x] Sort the fork's 26 commits: needed by every run (the time base, bounded waits), needed by
      a feature (agents, autopilot, environment actions, unmanaged ego), or fixes of general use.
      *Done 2026-11-01, below ("The fork, sorted").*
- [ ] Find the smallest set a scenario without an ego needs, and what the bridge does with an
      upstream SSv2 (fields it lacks, e.g. `simulation_time_ns`): degrade with a warning where
      the run stays correct, refuse with the reason where it would not.
- [ ] The general fixes as upstream pull requests -- each one confirmed with the user first.

#### The fork, sorted (2026-11-01)

Branch `managed-ego-unforked` at 355575c1b, merge base 3370c817e (2026-07-17, tag 25.0.22).
Upstream `master` has moved 58 commits since (d5f674cad, 2026-10-27): lane-change and
yielding fixes, a map package, `awf/universe/20250130` accepted by every launch file. None of
ours has landed there.

| group | commits | what csb loses on upstream |
|---|---|---|
| **Time base** | 357b3be15 simulation_time in replies; bd6197f42 `clock_source`; f0e480452 `simulation_time_ns` int64; a65f563f5 `/clock` continues across scenarios | SSv2 runs on its own clock; harmless with no ego (the scenario domain only), wrong with one (Autoware's time is the simulator's, 015) |
| **Failure bounds** | b0f1a75f3 bounded simulator wait; b87e83ae9 end on a failed frame; 7d5c92c59 waits bounded to what they cover | a dead bridge or CARLA hangs SSv2 instead of failing it (016 gaps 8, 9) |
| **Unmanaged ego, relay** | 4bb48c980, a04fcbce0, 59738aec4, 9ca1e7cdb, 463c02946 (`managed_ego`, inert FieldOperatorApplication); 2cddfac4d signals to Autoware; 7908d82d0 wait for localization; 7bfe6a159 reused Autoware left in STOP; 9a0e03cc1, 6e14cbb1f Autoware's time base; efe7f87f8, e27f65a9e concealer waits and retries | any scenario with an ego |
| **Features** | 4657051be `simulator_autopilot`; d990ab920 `agent` controller; a337a6064 vehicle subtype; 42f3815a9 `RoadCondition`; 355575c1b `Weather`, `TimeOfDay` | the scenarios that use them; an upstream SSv2 never sends the environment, and sends no subtype |
| **General tooling** | ac4c9e983 `$(env NAME)` and no rewrite of the user's file; a0f8f3f65 the preprocessor's refusal reason | nothing at run time |

Smallest set for a scenario without an ego: none is strictly needed. On upstream the protocol
stays compatible both ways (proto3: SSv2 ignores the reply fields it lacks, never sends the
requests it lacks), so such a scenario should run, on SSv2's own clock and without the failure
bounds. Not yet checked: that upstream's `scenario_test_runner` accepts the `clock_source` and
`managed_ego` arguments `swb_launch/launch/scenario.launch.xml` passes it (an unknown included
argument), and what the bridge does with subtype 0. Both need an upstream SSv2 build and one
NPC-only run.

Upstream candidates, in the order they stand alone (each PR only on the user's word):
1. a0f8f3f65, ac4c9e983 (tooling, no behaviour change);
2. b0f1a75f3, b87e83ae9, 7d5c92c59 (failure bounds: any external simulator gains);
3. 42f3815a9, 355575c1b (OpenSCENARIO `EnvironmentAction` forwarded to the simulator);
4. 357b3be15, f0e480452, bd6197f42, a65f563f5 (simulator as the time source; AWSIM-style
   simulators gain);
5. 4657051be (`simulator_autopilot`);
6. the unmanaged-ego group last: it is the largest and the most csb-shaped.

Before any of it: rebase the branch on upstream `master` (58 commits), rebuild, rerun the
starter suite.

## Acceptance

- Step 2: the spectator follows, hands over and costs nothing measurable.
- Step 3: a measured table; corrections only where measured.
- Step 4: numbers and a decision, either way.
- Step 5: an offset pack gives the same run as its CARLA-frame twin, or a written reason why
  not.
- Step 6: a first-run time on a clean account, and a quickstart that a new user follows
  without the tutorial.
- Step 7: done only if a CARLA scenario needs zones: a zone with `mu_scale` 0.5 stops a car longer inside it than outside.
- Step 8: 0.10 builds from main and passes the baseline scenario.
- Step 9: a sorted commit list; an NPC-only scenario's outcome on upstream SSv2, run or refused
  with the reason.

## Not in this phase

- Turning off Autoware's emergency handling or loosening its diagnostics to keep a slow host
  running. A run that a real stack would stop must fail.
- Folding the world server into the bridge, or the vehicle bridge into the simulator. The
  split is what lets another simulator, autopilot or car come in alone (051).
