#!/usr/bin/env bash
# Isaac Lab as an overlay on an Isaac Sim pip install (roadmap 092 K2.0; batch-executor.md).
#
#   scripts/isaaclab_overlay.sh <isaac-venv> <dest>
#
# Makes <dest>/venv: a venv of its own that sees <isaac-venv>'s packages through a .pth file,
# with Isaac Lab's packages (from a checkout at ISAACLAB_TAG in <dest>/src) installed editable
# and without dependencies, then only the third-party packages Isaac Lab needs that the Isaac
# venv lacks, constrained to the versions already there. Nothing in <isaac-venv> changes: no
# isaacsim, torch or warp. `pip check` runs at the end and its output is kept.
#
# The tag: v3.0.0-EA, chosen on purpose. Isaac Lab v3.0.0 (GA) pins isaacsim==6.2.0.0; this
# overlay is for an Isaac Sim 6.1.0.0 install, which v3.0.0-EA pins (with torch 2.11.0). Move to
# GA when the Isaac Sim install moves to 6.2.
set -euo pipefail
ISAACLAB_TAG=${ISAACLAB_TAG:-v3.0.0-EA}
ISAACLAB_COMMIT=${ISAACLAB_COMMIT:-ae37b028ea415c91ea2bc32609efcd759ed2b974}
PACKAGES=(isaaclab isaaclab_physx isaaclab_newton isaaclab_contrib isaaclab_assets
          isaaclab_tasks isaaclab_rl)

isaac=${1:?usage: isaaclab_overlay.sh <isaac-venv> <dest>}
dest=${2:?usage: isaaclab_overlay.sh <isaac-venv> <dest>}
py="$isaac/bin/python"
[ -x "$py" ] || { echo "no python at $py" >&2; exit 2; }
mkdir -p "$dest"

if [ ! -d "$dest/src/.git" ]; then
    git clone -q --depth 1 --branch "$ISAACLAB_TAG" https://github.com/isaac-sim/IsaacLab.git "$dest/src"
fi
got=$(git -C "$dest/src" rev-parse HEAD)
[ "$got" = "$ISAACLAB_COMMIT" ] || { echo "$dest/src is $got, not $ISAACLAB_TAG ($ISAACLAB_COMMIT)" >&2; exit 2; }

[ -x "$dest/venv/bin/python" ] || "$py" -m venv "$dest/venv"
ov="$dest/venv/bin/python"
site=$("$ov" -c 'import site; print(site.getsitepackages()[0])')
isite=$("$py" -c 'import site; print(site.getsitepackages()[0])')
echo "$isite" > "$site/isaac_venv.pth"     # after the overlay's own site-packages

# the Isaac venv's versions, as constraints: what it has stays what it has
"$py" -m pip freeze --exclude-editable 2>/dev/null | grep -v '^-e ' | grep '==' > "$dest/constraints.txt"

for p in "${PACKAGES[@]}"; do
    "$ov" -m pip install -q --no-deps --no-build-isolation -e "$dest/src/source/$p"
done

# Isaac Lab's third-party requirements the Isaac venv does not have
"$ov" - "$dest/src/pyproject.toml" > "$dest/missing.txt" <<'EOF'
import re, sys, tomllib
from importlib import metadata
from packaging.markers import default_environment
from packaging.requirements import Requirement
deps = tomllib.load(open(sys.argv[1], "rb"))["project"]["dependencies"]
env = default_environment()
for d in deps:
    r = Requirement(d)
    if r.name.lower().startswith("isaaclab") or (r.marker and not r.marker.evaluate(env)):
        continue
    try:
        metadata.version(r.name)
    except metadata.PackageNotFoundError:
        print(str(r).split(";")[0])
EOF
if [ -s "$dest/missing.txt" ]; then
    echo "installing what the Isaac venv lacks:"; sed 's/^/  /' "$dest/missing.txt"
    "$ov" -m pip install -q -c "$dest/constraints.txt" -r "$dest/missing.txt"
fi
"$ov" -m pip check > "$dest/pip-check.txt" 2>&1 || true
echo "pip check: $(wc -l < "$dest/pip-check.txt") line(s) in $dest/pip-check.txt"
"$ov" -c 'import isaaclab, torch, isaacsim; print("isaaclab", isaaclab.__file__, "torch", torch.__version__)'
