# 085: land_pin does not bump a target that pins a repository its own cascade moves

**Severity**: low. The landing is refused before anything is pushed, and landing the deepest
target alone works around it. **Status**: open; worked around. **Found**: 2026-11-20, by 241,
landing roadmap 091 W19.

## What happens

Roadmap 091 W19 moved two repositories:
- `sim_vehicle_spec` f96adf5 to 69346ab;
- `autoware_isaac_bridge` (aib) 97ab2f0 to 5520655. aib's main already pins the new spec in
  `deps/sim_vehicle_spec`, and still pins `autoware_sim_vehicle` 90bff5c.

`scripts/land_pin.sh --dry-run sim_vehicle_spec 69346ab autoware_isaac_bridge 5520655` planned:

```
== autoware_sim_vehicle: deps/sim_vehicle_spec f96adf5 -> 69346ab
== src/autoware_sim_vehicle 90bff5c -> (new)
== src/autoware_isaac_bridge 97ab2f0 -> 5520655
```

`autoware_sim_vehicle` gets a new commit, because it pins the spec. aib is a target, so it lands
at 5520655 exactly, but 5520655 still pins `autoware_sim_vehicle` 90bff5c. `check_pins` would
then refuse it:

```
src/autoware_isaac_bridge/deps/autoware_sim_vehicle is 90bff5c, but src/autoware_sim_vehicle here is <new>
```

Nothing would be pushed, since the check runs first.

## Cause

`scripts/land_pin.py` treats a target as fixed: it lands the given commit and never bumps it. A
component that is not a target is bumped when it pins a moving repository. A target that pins a
repository the cascade itself moves is the case missing. Here that is aib, which pins
`autoware_sim_vehicle`.

## Workaround

Land only the deepest target: `scripts/land_pin.sh sim_vehicle_spec 69346ab`. The cascade then
takes aib from its own main (5520655, which already had the W19 code) and bumps its
`autoware_sim_vehicle`, giving aib 18d218e. That landed as dd126de, after the full `just test`
passed on the same cascade staged locally.

## Fix (not done)

When a target pins a repository that is moving to a new commit, either:
- give the target a new commit on top of the given one, bumping that dependency (what the
  workaround produces); or
- refuse in the dry run, naming the target and the dependency, and suggest landing the deeper
  target alone.

The first is what a user wants. It is the same rule as for any other component in the
cascade, applied after the target's own commit.
