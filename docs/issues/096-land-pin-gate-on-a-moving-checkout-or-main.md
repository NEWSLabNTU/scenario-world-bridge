# 096: land_pin pushes after a gate that tested a changed checkout, or onto a moved main

**Severity**: medium. A landing can push component commits on the strength of a gate that tested
something else, or leave components pushed while this repository's pins are refused. **Status**:
fixed (this commit). **Found**: 2026-11-21, by 139, landing 092 K2's per-slot params with
`land_pin --onto` (issue 094).

## What happens

The landing ran `LAND_PIN_GATE` (the full gate) in its checkout. While the gate ran, a survey
command in the same checkout switched it to `origin/main`, so the gate built and tested a mixed
tree, and passed. land_pin then pushed the components' new commits (autoware_isaac_bridge
fac3be2, scensim_isaac_world 5975c30). Its push here was refused, because 133 had landed on
`origin/main` meanwhile. Main stayed consistent, pinning the old commits, but two component
commits had gone out on a gate that did not test them, and the landing was left half done.

Two gaps, both in what land_pin assumed between building the landing and pushing it:

- it did not check that the checkout the gate tested was the one it was about to push;
- it pushed the components before learning that this repository's push would be refused.

## The fix

- **The checkout is recorded around the gate.** Before `LAND_PIN_GATE` runs, land_pin records this
  checkout's HEAD, each landing component's checked-out commit, and the tracked files'
  uncommitted changes, which must be none. After the gate it records them again. Any difference
  refuses the push and names what moved, exit 1.
- **The mains are fetched again just before pushing.** If `origin/main` here no longer
  fast-forwards to the landing's tip, or any new component's `origin/main` no longer
  fast-forwards to its new commit, nothing is pushed, exit 3, with the moved main named. The
  check runs for every landing, with `--onto` or without.

A push can still be refused in the moment between that fetch and the push; land_pin keeps its
earlier message for that, naming what was pushed so far. The window is now a few seconds, not
the gate's length.

## Tests

`scripts/test_land_pin.py` has four new cases:

- a gate that switches the checkout, and one that edits a tracked file: refused, nothing pushed;
- an uncommitted change before the gate: refused;
- `origin/main` moved by another clone during the gate: exit 3, and no component pushed;
- the last component's main moved during the gate: exit 3, and no earlier component pushed either.
