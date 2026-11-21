# The truth stream (roadmap 092 S4)

Ground truth for every step of a run, which any block can read: judges, the coverage report's
failure definition, a bird's-eye renderer, an attachment such as a network simulator. It is
the world as the simulator holds it, never as a stack perceives it. Design 092
(`ml-training-interface.md`, section 3's truth stream and section 9's question 5) asked for
it; this is its specification. K4 implements it.

## 1. Where it comes from

The bridge already holds all of it, so the world protocol gains no message:

- every `StepReply` carries each entity's pose and velocity (`states`, sent when the bridge
  sets `want_states`), the step's contacts, the frame and `sim_time_ns`;
- `Spawn` and `Body` give each entity's name, role, subtype, declared box, origin offset and
  simulator id;
- the bridge sets the signal states (`SetSignals`), so it knows each change.

The bridge asks for states on every step whenever truth is recorded or published. It stays
simulator-agnostic: CARLA's and Isaac's world servers answer the same messages.

## 2. Two forms, one content

- **The per-run file.** The gap trace's JSON Lines file, schema 2: one file per episode, in the
  run's `gap_trace/` directory (`results-and-logs.md`), written by the bridge's writer thread.
  The gap trace is the run's truth file; it keeps its name, which judges use. It is recorded by
  default (`logging.gap_trace` defaults to true from K4), and follows the run's keep levels
  like any other episode file, so a suite can drop it for passed runs.
- **The live topic.** `/scensim/truth` in the scenario domain, `scensim_interfaces/msg/
  TruthFrame`, reliable, keep last 10, one message per SSv2 step. The bridge publishes it when
  the testbench asks (`logging.truth_topic: true`, or any `attachments:`), for readers that act
  during the run: an attachment's mobility model, a live judge, a renderer.

A frame carries the world after SSv2 step `k`'s tick. The file writes frame `k`'s line one
request later, when it pairs SSv2's intent with the world (the gap trace's staging); the
topic publishes the same world state as soon as the step's reply arrives. Their content for a
frame is the same.

## 3. Records (the file)

Every line is one JSON object with a `type`. Units are SI; poses are in the map frame (the
world frame moved by `map_origin`) as `[x, y, z, roll, pitch, yaw]`, metres and radians.

| type | when | fields (schema 2) |
|---|---|---|
| `episode` | first line | `schema` (2), `episode`, `start_unix`, `npc_motion`, `map` (the map's name), `world_hash` (the stage's sha256 on Isaac, `provenance.stage`; the map files' digest on CARLA), `step_time` (s) |
| `entity` | at each spawn | `frame`, `name`, `index` (stable per episode, see 5), `role` (`ego`, `npc`, or `pilot:<entity>`), `kind`, `subtype` (SSv2's: car, truck, pedestrian, ...), `declared {center, dimensions}`, `origin_offset [x, y, yaw]`, `sim_id`, `asset`, `asset_box` |
| `frame` | each SSv2 step | `frame`, `sim_time`, `sim_time_ns`, `sim_frame`, `entities {name: state}` |
| `contact` | each contact | `frame`, `sim_frame`, `sim_time`, `ego_id`, `subject`, `other_id`, `name`, `other_type`, `other_kind`, `impulse` |
| `signal` | each change | `frame`, `sim_time_ns`, `opendrive_id`, `state` (the `SignalState` the bridge set) |
| `end` | last line | `frame`, `sim_time`, `frames`, `dropped_frames`, `complete` |

An entity's state in a frame: `ssv2` (SSv2's pose), `sim_pose` (the simulator's pose: the
truth), `velocity [vx, vy, vz]`, `angular_velocity [wx, wy, wz]`, `velocity_source`
(`measured` when the world server sent it, `derived` when the bridge differenced two poses,
for a kinematic body on a server that does not measure it), and `flags` (`spawn`,
`contact_hold`, `kinematic`).

Schema 2 names the simulator's values neutrally. Schema 1 (CARLA only) named them `carla`
(`sim_pose`), `carla_frame` (`sim_frame`), `actor_id` (`sim_id`), `blueprint` (`asset`) and
`blueprint_box` (`asset_box`), and lacked `index`, `role`, `subtype`, `sim_time_ns`,
`angular_velocity`, `velocity_source`, the `episode` line's map fields and the `signal` lines.
A reader takes the names of the file's `schema`: versioned reading of a versioned format.

Signals are events, not repeated every frame: a reader keeps the last state per signal.

## 4. The topic

`TruthFrame` carries one frame: `frame`, `sim_time_ns`, `episode`, and arrays aligned by entity
`index`: `names`, `poses` (six values each, as the file's), `twists`, `velocity_measured`, plus the step's contacts and any
signal changes. Entity metadata (role, class, boxes, ids) does not repeat per frame: it is
latched on `/scensim/truth/entities` (`TruthEntities`, transient local), republished at each
spawn and despawn, with the episode's map identity.

## 5. Shaped for tensors

The batch path (092 K2, Isaac Lab) carries truth as tensors per slot, so every per-entity
field has a fixed shape and a fixed name, usable as a tensor key: `pose [6]`, `velocity [3]`,
`angular_velocity [3]`, `velocity_source` as a boolean, the box `[3]`. An entity's `index` is
its slot: assigned at spawn, never reused in an episode, so a despawned entity's slot stays
empty rather than shifting the others. No per-entity free-form map is part of the schema;
`flags` is a closed set of names (schema 1's), which a batch reads as booleans.

## 6. Absent by design

These are named so a reader knows they are missing on purpose, not by fault:

- **Route progress.** SSv2 owns the route; the bridge never sees it. A judge derives it after
  the run from the truth, the Lanelet2 map and the scenario's route (scensim_agent's router
  does lanelet routing). A later capability may publish it.
- **Occlusion.** Visibility between an observer and a target needs scene queries in the world
  server: cheap on Isaac (batched PhysX raycasts), costly on CARLA (a raycast per pair over
  RPC). A later world-server capability, `OCCLUSION`, will report it where a server offers it.
- **Lanes.** The map is static, and every reader has it.

## 7. Rate and cost

One frame per SSv2 step: 20 Hz at the usual 0.05 s step, on both simulators. About 150 bytes of
JSON per entity: twenty entities make about 60 KB/s, 216 MB per hour of simulated driving,
before compression. The bridge's writer thread takes the disk off the step's path: a frame
that does not fit its bounded channel is dropped and counted in `end`.

The cost on the simulator side is the states on every step. CARLA's world server reads every
actor from the world snapshot it takes each tick anyway; Isaac's reads the PhysX views it
already updates. K4's exit measures `tick_seconds` with and without truth at twenty NPCs on
both simulators; the budget is 1 ms per step.

## 8. Readers

`scensim_launch/scripts/gap_trace.py` reads both schemas, by the `episode` line's `schema`:
schema 1's names in a schema 1 file, schema 2's in a schema 2 file, and schema 2's additions
as unknown in a schema 1 file. The gap oracle reads through it, so it reads both. The topic's readers subscribe in the scenario domain with
the stack's DDS settings (`domains-and-dds.md`).
