# 083: land_pin cannot land two repositories at once, nor move a component that already pins the target

**Severity**: medium. A landing that is green on `just test` cannot be landed through the script, and pins must go
through the script. **Status**: fixed (this commit). **Found**: 2026-11-20, by 241, landing roadmap 091 step 2's pins.

## What happens

Roadmap 091 step 2 moved three of our repositories:
- `sim_vehicle_ecu` b4dda7d to 7bdcb8e;
- `sim_vehicle_spec` e827bb0 to f96adf5;
- `autoware_isaac_bridge` (aib) b05142f to 07b8081. aib's main already pins both new commits in its `deps/`.

The full `just test` was green with all three pins moved. Then `scripts/land_pin.sh sim_vehicle_ecu 7bdcb8e` moved only
`src/sim_vehicle_ecu`. Its own pin check refused the result, and nothing was pushed:

```
FAIL: src/autoware_isaac_bridge/deps/sim_vehicle_ecu is b4dda7dad7, but src/sim_vehicle_ecu here is 7bdcb8e5cc
```

Landing `sim_vehicle_spec` first fails the same way, the other way round: its cascade moves aib to a commit that pins
the new `sim_vehicle_ecu`, while this repository still pins the old one.

## Cause

Two gaps in `scripts/land_pin.py`:

1. **One repository per landing.** `check_pins` needs every component's `deps/` pin to equal this repository's pin of
   that repository, exactly; it does not accept an ancestor. A component whose main moved two dependencies therefore
   needs both moved here in one commit. The script took a single `<repo> <sha>`.
2. **A component already at the target was skipped.** The cascade moved a component only when its main pinned a
   moving repository at something other than the target, which it then bumped with a new commit. aib's main already
   pinned the target, so aib was neither bumped (correct) nor moved here to its main (the gap). This repository kept its
   old aib pin, whose `deps/` named the old commits.

## Fix

`land_pin.py` takes several targets and lands them as one cascade and one commit here:

```
scripts/land_pin.sh [--dry-run] <repo> <sha> [<repo> <sha> ...] [-- note]
scripts/land_pin.sh [--dry-run] <repo> <sha> [note]      # as before
```

- **Every target first.** Each target must be on its repository's `origin/main`.
- **The moving set.** A component moves when it pins any moving repository in `deps/`, transitively, as before.
- **In dependency order.** Each moving component either:
  - gets a new commit on its `origin/main` bumping its moving deps, when any of its `deps/` pins differs from that
    dep's target, as before; or
  - **moves here to its own `origin/main`, with no commit of its own**, when its main already pins every moving dep
    at its target. That case was missing.
- **Pushes.** Only the new commits are pushed (in dependency order), then this repository's one commit. A dry run
  prints, for each component, whether it gets a new commit or moves to its main.

## Tests

`scripts/test_land_pin.py` builds scratch repositories (pure git on the local disk, no network): a superproject with
components that pin each other under `deps/`, each with a bare "origin". Its cases:
- a single landing cascades, as before;
- two targets land in one commit;
- a component whose main already pins the targets is moved to its main, not committed;
- step 2's case: two targets, one component already at both, and its dependent bumped; `check_pins` green after;
- the dry run changes nothing.

The step 2 case fails on the old script.
