#!/usr/bin/env bash
# One package's wheel out of the release's wheels, for a trusted-publishing job that may
# upload only its own project (roadmap 082: one GitHub environment per PyPI project).
#
#   scripts/release/one_wheel.sh <wheels dir> <package> <out dir>
#
# Copies the wheel whose distribution name is <package> (scensim, scensim-ros-humble,
# colcon-scensim) into <out dir>; refuses unless there is exactly one.
set -euo pipefail
[ $# -eq 3 ] || { echo "usage: one_wheel.sh <wheels dir> <package> <out dir>" >&2; exit 2; }
src=$1 pkg=$2 out=$3
# a wheel's file name starts with the distribution name, '-' written as '_' (PEP 427)
want=${pkg//-/_}
found=()
for f in "$src"/*.whl; do
    [ -e "$f" ] || continue
    name=$(basename "$f")
    [ "${name%%-*}" = "$want" ] && found+=("$f")
done
if [ "${#found[@]}" -ne 1 ]; then
    echo "one_wheel.sh: ${#found[@]} wheels of $pkg in $src (want exactly 1): $(ls "$src")" >&2
    exit 1
fi
mkdir -p "$out"
cp "${found[0]}" "$out/"
echo "$pkg: $(basename "${found[0]}")"
