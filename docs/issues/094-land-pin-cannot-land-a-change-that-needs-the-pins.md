# 094: land_pin cannot land a change here that needs the pins it moves

**Severity**: low. Nothing broke; the landing went by hand, as one gated tip. **Status**: fixed (this
commit). **Found**: 2026-11-21, by 243, landing roadmap 092 K4.

## What happens

092 K4 renamed the gap trace recorder's interface in `scensim_world_protocol` (scensim_core). Three
things had to land together:

- `scensim_world_protocol` 45298c3, the recorder;
- `scensim_carla_world` 3067841, whose code uses the renamed interface (and its `deps/` pin);
- this repository's bridge code, which uses it too, with the three pins.

land_pin covers the first two: several targets in one cascade (issue 083), with a target that already
pins its moving dependency landed as it is. It does not cover the third. It commits the pins onto
`origin/main` and refuses a checkout with commits that are not on it (issue 085's guard). So the
choices were to land the pins first, leaving main's bridge code built against an interface that no
longer exists, or the code first, leaving main pinned to the old recorder. Either leaves main broken
between two pushes. K4 landed by hand instead: the components fast-forwarded, then one tip here with
the code and the pins, gated.

## The fix

`--onto <branch>` puts the pins on `origin/<branch>` instead of `origin/main`. The branch must be pushed
and fast-forward `origin/main`; it carries the change here that needs the pins. The cascade runs as
before, and one tip lands, with the code under the pin commit. A branch whose pins are already at their
targets lands as it is.

Since that lands code and not only pins, the tip must have passed the full gate, in one of two ways:

- `LAND_PIN_GATE`: a shell line (the full gate's command) that land_pin runs in its checkout at the tip,
  after `check_pins`, with the components' new commits in place; or
- `--gated <sha>`: a commit the gate already passed on whose tree is the tip's. The pin commit is made
  in the landing, so its sha is new, and the content is what is compared.

With neither, the landing is refused. Nothing is pushed before the gate passes.

**A limit.** `--gated` cannot name a landing whose cascade makes new component commits: those commits
get new shas on every run, so no earlier gate saw that tree. land_pin refuses it and names the
components. Run the gate in the landing (`LAND_PIN_GATE`), or land those components on their mains
first, after which the cascade makes no new commits and `--gated` applies. K4's case, with every
component already on its main, is the one `--gated` covers. The limit is in `land_pin.py --help` (rule 5).

## Tests

`scripts/test_land_pin.py` has six new cases:

- the code and the pins land as one linear tip, with the code under the pin commit;
- refused without a gate, a dry run included, and `--gated` without `--onto`;
- refused for a branch that does not fast-forward `origin/main`;
- a failed gate pushes nothing; a `--gated` commit of other content is refused; the gated tree lands;
- `--gated` is refused when the cascade makes new commits, naming them;
- the dry run names the branch and the gate.
