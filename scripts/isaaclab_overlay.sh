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
# The tag: v3.0.0, the release, which pins isaacsim==6.2.0.0, torch 2.11.0 and warp-lang 1.17.0
# (roadmap 095). Its packages are a uv workspace of pyproject.toml members (no setup.py), which
# import one another (isaaclab imports isaaclab_ov, for one): every member under source/ is
# installed editable, and the third-party dependencies are the root pyproject's (isaaclab-dev)
# and the members' (isaaclab_ov's ovstage is only there). The Isaac venv's own versions win
# (constraints): isaacsim 6.2.0.0 brings its own warp-lang, which `pip check` then reports
# against the 1.17.0 pin. Isaac Lab's own sources take torch and torchvision from the cu130
# index; the Isaac venv has the cu128 builds (install_isaac.sh), which win here too.
# For an Isaac Sim 6.1.0.0 install: ISAACLAB_TAG=v3.0.0-EA
# ISAACLAB_COMMIT=ae37b028ea415c91ea2bc32609efcd759ed2b974 (092 K2.0).
set -euo pipefail
ISAACLAB_TAG=${ISAACLAB_TAG:-v3.0.0}
ISAACLAB_COMMIT=${ISAACLAB_COMMIT:-e3f7d9ff1f5d7f8280de3ece9000b7117f66bcf4}

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

# the Isaac venv's versions, as constraints: what it has stays what it has; and its whole freeze,
# compared at the end
"$py" -m pip freeze 2>/dev/null > "$dest/isaac-freeze-before.txt"
grep -v '^-e ' "$dest/isaac-freeze-before.txt" | grep '==' > "$dest/constraints.txt"

# every workspace member: they import one another
mapfile -t PACKAGES < <(cd "$dest/src/source" && for d in */; do [ -f "$d/pyproject.toml" ] && echo "${d%/}"; done)
echo "Isaac Lab packages: ${PACKAGES[*]}"
for p in "${PACKAGES[@]}"; do
    "$ov" -m pip install -q --no-deps --no-build-isolation -e "$dest/src/source/$p"
done

# Isaac Lab's third-party requirements the Isaac venv does not have
"$ov" - "$dest/src/pyproject.toml" "$dest"/src/source/*/pyproject.toml > "$dest/missing.txt" <<'EOF'
import re, sys, tomllib
from importlib import metadata
from packaging.markers import default_environment
from packaging.requirements import Requirement
deps = [d for f in sys.argv[1:] for d in tomllib.load(open(f, "rb"))["project"].get("dependencies", [])]
env = default_environment()
seen = set()
for d in deps:
    r = Requirement(d)
    if r.name.lower().startswith("isaaclab") or (r.marker and not r.marker.evaluate(env)):
        continue
    try:
        metadata.version(r.name)
    except metadata.PackageNotFoundError:
        line = str(r).split(";")[0]
        if line not in seen:
            seen.add(line)
            print(line)
EOF
if [ -s "$dest/missing.txt" ]; then
    echo "installing what the Isaac venv lacks:"; sed 's/^/  /' "$dest/missing.txt"
    "$ov" -m pip install -q -c "$dest/constraints.txt" -r "$dest/missing.txt"
fi
"$py" -m pip freeze 2>/dev/null > "$dest/isaac-freeze-after.txt"
if ! diff -u "$dest/isaac-freeze-before.txt" "$dest/isaac-freeze-after.txt"; then
    echo "the Isaac venv changed (above): it must not; nothing more is done" >&2
    exit 1
fi
echo "the Isaac venv is unchanged ($(wc -l < "$dest/isaac-freeze-after.txt") packages)"
"$ov" -m pip check > "$dest/pip-check.txt" 2>&1 || true
echo "pip check: $(wc -l < "$dest/pip-check.txt") line(s) in $dest/pip-check.txt"
PYTHONNOUSERSITE=1 "$ov" -c 'import isaaclab, isaaclab_tasks, torch; from isaaclab.envs import DirectRLEnv; print("isaaclab", isaaclab.__file__, "torch", torch.__version__)'
