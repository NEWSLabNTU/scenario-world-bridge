# 092: no per-run seed reaches the world servers, and SSv2's IMU noise seed is fixed at 0

**Severity**: low. A sample record names a world seed, and a replay records it, but nothing
applies it. The run is not less repeatable than before: a seed for randomness the world does not
have changes nothing. **Status**: open, recorded only. **Found**: 2026-11-21, swb 139, roadmap
092 K3.

## What happens

A sample record (docs/design/sample-record.md) carries seeds for the scenario, the world and
each pilot. A sampled run, and `scensim run --replay`, deliver them where something reads a seed:

| Seed | Where it goes | Applied |
|---|---|---|
| a pilot's | its entity's pilot, as `{seed}` in a `lifetime: run` pilot's arguments | yes |
| the scenario's | SSv2's parameter `scensim_seed`, when the scenario declares it | yes, for a scenario that reads it |
| the world's | `sample.json` only | **no** |

**No world server takes a per-run seed.** `scensim_world.proto` has no seed field, and neither
`scensim_carla_world` nor `scensim_isaac_world` reads one. The CARLA world server reseeds its
Traffic Manager at every scenario start, but from its config file (`traffic_manager.seed` in
`scensim_carla_world.yaml`; docs/design/scenario-authoring.md, "Repeatability"), so every run of
a stack has the same value. A world seed would mostly carry a value that already exists.

**SSv2 has no scenario-level random seed.** Nothing in `scenario.launch.xml` takes one. Its only
fixed seed is the noise of the IMU it attaches to an Autoware ego, which
`openscenario_interpreter`'s `simulator_core.hpp` sets to 0 for every run
(`configuration.set_seed(0)`). So every run sees the same IMU noise, whatever the record says.

## Why it matters

Today it matters little: nothing in the world draws at random per run. It will matter once a
world knob does, for example spawn jitter, weather noise, or Isaac's sensor noise. Then the
record's world seed is what makes such a run replayable.

## Fix (when taken)

1. **The world protocol.** Add `seed` to the episode or reset message in `scensim_world.proto`.
   The bridge fills it from the run (scensim passes it as a run setting). The CARLA world server
   seeds its Traffic Manager from it, falling back to the config's value; the Isaac world server
   uses it for any randomness it adds. This changes `scensim_world_protocol`, both world servers
   and the bridge.
2. **SSv2.** Make the IMU seed a parameter in the SSv2 fork (`managed-ego-unforked`), set from
   the record's scenario seed, or document it as fixed.

The conformance check (`scensim_world_check conformance`) should then check that a server
reports the seed it applied.
