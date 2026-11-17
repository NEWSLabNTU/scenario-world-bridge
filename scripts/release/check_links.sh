#!/usr/bin/env bash
# The link check (roadmap 082): on a clean ROS image with only the tarball and its rosdep
# packages installed, every shared object and executable in it resolves every library and
# symbol (`ldd -r`), and every script's interpreter exists. Fails when one does not,
# naming it.
#
#   scripts/release/check_links.sh <scensim-sim-....tar.zst> [--image ros:humble-ros-core]
#       [--out DIR]
#
# Needs docker. Writes to --out (default: next to the tarball, <name>.links/): the rosdep
# keys, the apt packages rosdep installs (apt.txt, and apt_exec.txt for run-time only), and
# the ldd findings before and after.
set -euo pipefail

TARBALL=""
IMAGE=""
OUT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --image) IMAGE=$2; shift 2 ;;
        --out) OUT=$2; shift 2 ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) TARBALL=$1; shift ;;
    esac
done
[ -f "$TARBALL" ] || { echo "check_links.sh: no tarball (see --help)" >&2; exit 2; }
TARBALL=$(readlink -f "$TARBALL")
name=$(basename "$TARBALL" .tar.zst)
distro=$(echo "$name" | sed -E 's/^scensim-sim-.*-([a-z]+)-[a-z0-9_]+$/\1/')
IMAGE=${IMAGE:-ros:$distro-ros-core}
OUT=${OUT:-$(dirname "$TARBALL")/$name.links}
mkdir -p "$OUT"
command -v docker >/dev/null || { echo "check_links.sh: docker not on PATH" >&2; exit 2; }

# Keys rosdep cannot resolve for the simulation side until 082 step 4 splits the
# dependencies by side: the vehicle side's packages (swb_launch's exec_depends) and SSv2's
# pip key.
SKIP_KEYS=${SCENSIM_LINK_SKIP_KEYS:-"aib_launch acb_bridge acb_launch autoware_sim_profile czp python-yamale-pip"}
# Findings that are not the artefact's: rviz's Ogre lives in rviz_ogre_vendor's own
# library directory, which rviz adds itself at run time.
ALLOW=${SCENSIM_LINK_ALLOW:-"libopenscenario_visualization_rviz_plugin.so"}

cat > "$OUT/inside.sh" <<'EOF'
#!/bin/bash
set -o pipefail
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq && apt-get install -y -qq zstd file python3-rosdep >/dev/null 2>&1
P=/opt/scensim; mkdir -p $P
zstd -dc /in/release.tar.zst | tar -xf - -C $P
rosdep init >/dev/null 2>&1; rosdep update --rosdistro "$ROS_DISTRO" -q >/dev/null 2>&1
rosdep keys --from-paths $P/share --ignore-src 2>/dev/null | sort -u > /out/keys.txt
sim() { rosdep install --from-paths $P/share --ignore-src -s -y -r --rosdistro "$ROS_DISTRO" \
          --skip-keys "$SKIP_KEYS" "$@" 2>>/out/rosdep.err \
        | grep -oE "apt-get install -y [^ ]+" | awk '{print $4}' | sort -u; }
sim > /out/apt.txt
sim --dependency-types exec > /out/apt_exec.txt
check() {
  local tag=$1 n=0 bad=0; : > /out/ldd_$tag.txt
  source /opt/ros/$ROS_DISTRO/setup.bash; source $P/setup.bash || return 9
  while IFS= read -r -d '' f; do
    case "$(file -b "$f")" in ELF*shared\ object*|ELF*executable*) ;; *) continue ;; esac
    n=$((n+1)); o=$(ldd -r "$f" 2>&1)
    if echo "$o" | grep -qE "not found|undefined symbol"; then
      bad=$((bad+1)); { echo "== ${f#$P/}"; echo "$o" | grep -E "not found|undefined symbol" | head -5; } >> /out/ldd_$tag.txt
    fi
  done < <(find $P -type f -print0)
  echo "$tag: $n ELF files, $bad with a missing library or undefined symbol"
}
check before
rosdep install --from-paths $P/share --ignore-src -y -q -r --rosdistro "$ROS_DISTRO" \
  --skip-keys "$SKIP_KEYS" > /out/rosdep_install.log 2>&1 || echo "rosdep install exited $?"
check after
# every script's interpreter exists here (a build-host python in a shebang fails as
# "bad interpreter" on every other host)
: > /out/shebangs.txt
while IFS= read -r -d '' f; do
  head -n1 "$f" 2>/dev/null | grep -qE '^#! ?/' || continue
  interp=$(head -n1 "$f" | sed -E 's/^#! *//; s/ .*//')
  [ -x "$interp" ] || echo "${f#$P/}: $interp" >> /out/shebangs.txt
done < <(find $P -type f -perm -u+x -print0)
echo "scripts with a missing interpreter: $(wc -l < /out/shebangs.txt)"
EOF

docker run --rm --name "scensim-links-$$" \
    -e SKIP_KEYS="$SKIP_KEYS" \
    -v "$TARBALL":/in/release.tar.zst:ro -v "$OUT/inside.sh":/in/inside.sh:ro -v "$OUT":/out \
    "$IMAGE" bash /in/inside.sh 2>&1 | grep -v "^debconf" | tee "$OUT/summary.txt"

echo "keys: $(wc -l < "$OUT/keys.txt"); apt packages: $(wc -l < "$OUT/apt.txt") ($(wc -l < "$OUT/apt_exec.txt") at run time)" | tee -a "$OUT/summary.txt"
if [ -s "$OUT/shebangs.txt" ]; then
    echo "check_links.sh: FAILED, scripts whose interpreter does not exist on the image:" | tee -a "$OUT/summary.txt"
    head -20 "$OUT/shebangs.txt" | sed 's/^/  /' | tee -a "$OUT/summary.txt"
    exit 1
fi
failed=$(grep '^== ' "$OUT/ldd_after.txt" | sed 's/^== //' | while read -r f; do
    allowed=0; for a in $ALLOW; do [ "$(basename "$f")" = "$a" ] && allowed=1; done
    [ $allowed = 0 ] && echo "$f"; done || true)
if [ -n "$failed" ]; then
    echo "check_links.sh: FAILED, unresolved after rosdep:" | tee -a "$OUT/summary.txt"
    echo "$failed" | sed 's/^/  /' | tee -a "$OUT/summary.txt"
    exit 1
fi
echo "check_links.sh: passed (allowed: $ALLOW)" | tee -a "$OUT/summary.txt"
