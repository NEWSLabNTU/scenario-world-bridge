# 085: land_pin does not bump a target that pins a repository its own cascade moves

**Severity**: low. The landing is refused before anything is pushed, and landing the deepest
target alone works around it. **Status**: fixed (this commit). **Found**: 2026-11-20, by 241,
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

## Fix

Targets now join the dependency-ordered pass with the other moving components, read from their given
commit rather than their main. When that commit pins a repository that the landing moves elsewhere
(including one that gets a new commit in the cascade), the target gets a new commit on top of it,
bumping those pins. This is the same rule as for any other component, and what the workaround produced.

The new commit goes on the target's main, which must therefore be the given commit. A given commit behind
its main's tip would bring the commits after it along. In that case the landing is refused by name, naming
the dependencies, before anything is made.

## Tests

`scripts/test_land_pin.py` has two new cases, and both fail on the script before the fix:

- A and D land together, where D's tip pins C and the cascade moves C (C pins A). D gets a new commit
  on top of the given one, pinning C's new commit, and E follows. `check_pins` is green.
- The same, with D's main moved past the given commit. The landing is refused, exit 2, naming C, and
  nothing is pushed.

## A related guard

The pins are committed onto this repository's origin/main, never onto the checkout's HEAD. A local commit
in the checkout was therefore left out of a landing, silently: 118's W6 commit was dropped that way and
cherry-picked back afterwards. land_pin now refuses to run, a dry run included, when the checkout has
commits that are not on origin/main. It names them and says to push or land them first, or to run from a
checkout at origin/main. A detached HEAD at origin/main passes. `scripts/test_land_pin.py` covers it.
