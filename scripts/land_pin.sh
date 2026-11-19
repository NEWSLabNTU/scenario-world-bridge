#!/usr/bin/env bash
# Move every pin of one of our repositories to a landed commit, in the order that keeps
# `scripts/check_pins.py` green on main:
#
#   scripts/land_pin.sh [--dry-run] <repo> <sha> [note]
#
#   1. <sha> must already be on <repo>'s origin/main (push and land it there first).
#   2. Each component whose deps/<repo> pins it gets the bump first: commit, push its main.
#   3. Then this repository: src/<repo> and every bumped component, in one commit.
#   4. check_pins, unpiped (set -e): a mismatch stops before this repository's push.
#
# Plain fast-forward pushes of rebased mains, never forced. --dry-run prints what would move
# and touches nothing. LAND_PIN_TRAILER, when set, is appended to each commit message (the
# session's attribution lines, for instance).
set -euo pipefail
DRY=0
[ "${1:-}" = "--dry-run" ] && { DRY=1; shift; }
[ $# -ge 2 ] || { sed -n '2,14p' "$0" >&2; exit 2; }
REPO=$1; SHA=$2; NOTE=${3:-}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
TRAILER=${LAND_PIN_TRAILER:+$'\n\n'$LAND_PIN_TRAILER}
cd "$ROOT"
run() { if [ $DRY = 1 ]; then echo "  would: $*"; else "$@"; fi; }

git fetch -q origin
[ -z "$(git status --porcelain --untracked-files=no --ignore-submodules=all)" ] || { echo "$ROOT has local changes" >&2; exit 2; }
run git rebase -q origin/main

top=src/$REPO
[ -d "$top/.git" ] || [ -f "$top/.git" ] || { echo "no submodule $top" >&2; exit 2; }
git -C "$top" fetch -q origin
full=$(git -C "$top" rev-parse -q --verify "$SHA^{commit}") ||
    { echo "$REPO has no commit $SHA (pushed?)" >&2; exit 2; }
git -C "$top" merge-base --is-ancestor "$full" origin/main ||
    { echo "$REPO $SHA is not on its origin/main: land it there first" >&2; exit 2; }
short=$(git -C "$top" rev-parse --short=7 "$full")
subj=$(git -C "$top" log -1 --format=%s "$full")
echo "$REPO -> $short ($subj)"

bumped=()
for dep in src/*/deps/"$REPO"; do
    [ -e "$dep" ] || continue
    comp=${dep%/deps/*}
    cur=$(git -C "$comp" ls-tree HEAD "deps/$REPO" | awk '{print $3}')
    [ "$cur" = "$full" ] && continue
    echo "== $comp: deps/$REPO ${cur:0:7} -> $short"
    bumped+=("$comp")
    [ $DRY = 1 ] && continue
    git -C "$comp" fetch -q origin
    git -C "$comp" checkout -q -B main origin/main
    git -C "$comp" submodule update -q --init "deps/$REPO"
    git -C "$comp/deps/$REPO" fetch -q origin
    git -C "$comp/deps/$REPO" checkout -q "$full"
    git -C "$comp" add "deps/$REPO"
    git -C "$comp" commit -q -m "deps: $REPO $short ($subj)$TRAILER"
    git -C "$comp" push -q origin main
done
cur=$(git ls-tree HEAD "$top" | awk '{print $3}')
[ "$cur" = "$full" ] || echo "== $top ${cur:0:7} -> $short"
if [ $DRY = 1 ]; then echo "dry run: nothing changed"; exit 0; fi

git -C "$top" checkout -q "$full"
git add "$top"
for comp in "${bumped[@]}"; do
    git -C "$comp" merge-base --is-ancestor HEAD origin/main ||
        { echo "$comp's main did not reach origin" >&2; exit 3; }
    git add "$comp"
done
if git diff --cached --quiet; then
    python3 scripts/check_pins.py
    echo "pins already at $short"; exit 0
fi
names="$REPO $short"
for comp in "${bumped[@]}"; do names+=", ${comp#src/} $(git -C "$comp" rev-parse --short=7 HEAD)"; done
git commit -q -m "pins: $names${NOTE:+ ($NOTE)}$TRAILER"
python3 scripts/check_pins.py
git push -q origin HEAD:main
git log --oneline -1
