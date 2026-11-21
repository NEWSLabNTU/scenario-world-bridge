# 095: the Isaac world server does not report the surface map it applied

**Severity**: low. A run's surface knob is applied, but not confirmed. **Status**: open.
**Found**: 2026-11-21, by 139, building 093 D7 gap 1 (knobs reach the world).

## What happens

A scenario binds a surface knob (`scensim.knob.surface = $surface`), and the world's
`knobs:` in testbench.yaml maps a value to a named surface map. On Isaac the scenario's
RoadCondition names it (`scensim.surface_map`), and the project's ground links the file where
the world server reads it (`scensim.ground`, roadmap 056, 086 C4). Gap 1 asks the runner to
read every applied knob back and to make a run whose applied values differ from its record's
an error run.

Weather has a read-back: the bridge's `get_weather`. A surface map has none. The Isaac world
server loads the named map (`scensim_isaac_world.world`, `surface_maps`), but neither its
provenance (the run's `provenance.stage`) nor any service says which map it applied. So
`result.json`'s `knobs.<knob>.surface_map.read` is `unverified`.

## Fix

The world server reports the surface map it applied, per world load: in its stage
provenance, or as the answer to a query the runner makes after the scenario's RoadCondition.
`knobs.compare` already takes a read-back (`maps_read`) and fails a run on a difference.
