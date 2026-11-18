#!/usr/bin/env bash
# The link check (roadmap 082): on a clean ROS image with only the tarball and its rosdep
# packages installed, every shared object and executable in it resolves every library and
# symbol (`ldd -r`), every script's interpreter exists, every symbolic link resolves
# inside the prefix, every RPATH/RUNPATH entry is $ORIGIN-relative or a system directory, no
# file of a source build only is left in (built_from), and setup.sh sources from sh once
# relocate.sh has run. Fails when one does not, naming it.
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
OUT=$(cd "$OUT" && pwd)   # docker -v takes an absolute path only
command -v docker >/dev/null || { echo "check_links.sh: docker not on PATH" >&2; exit 2; }

# Keys to skip (none: since 082 step 4 every key of the simulation side resolves, and the
# check fails on one that does not, as the line `scensim install` prints names no skips).
SKIP_KEYS=${SCENSIM_LINK_SKIP_KEYS:-""}
# Findings that are not the artefact's: rviz's Ogre lives in rviz_ogre_vendor's own
# library directory, which rviz adds itself at run time.
ALLOW=${SCENSIM_LINK_ALLOW:-"libopenscenario_visualization_rviz_plugin.so"}

cat > "$OUT/inside.sh" <<'EOF'
#!/bin/bash
set -o pipefail
export DEBIAN_FRONTEND=noninteractive
# python3-pip as Install has it first: rosdep's pip keys (huggingface-hub, yamale) need it
apt-get update -qq && apt-get install -y -qq zstd file binutils python3-rosdep python3-pip >/dev/null 2>&1
P=/opt/scensim; mkdir -p $P
zstd -dc /in/release.tar.zst | tar -xf - -C $P
# the release where the front end puts it: relocated, then sourced from sh as from bash
$P/relocate.sh
if sh -c ". $P/setup.sh && test -n \"\$AMENT_PREFIX_PATH\"" > /out/setup_sh.txt 2>&1; then echo "setup.sh from sh: ok"
else echo "setup.sh from sh: FAILED"; fi
rosdep init >/dev/null 2>&1; rosdep update --rosdistro "$ROS_DISTRO" -q >/dev/null 2>&1
rosdep keys --from-paths $P/share --ignore-src 2>/dev/null | sort -u > /out/keys.txt
sim() { rosdep install --from-paths $P/share --ignore-src -s -y -r --rosdistro "$ROS_DISTRO" \
          --skip-keys "$SKIP_KEYS" "$@" 2>>/out/rosdep.err \
        | grep -oE "apt-get install -y [^ ]+" | awk '{print $4}' | sort -u; }
# the line `scensim install` prints, as a user types it: every key resolves
if rosdep install --from-paths $P/share --ignore-src -s -y --rosdistro "$ROS_DISTRO" > /out/rosdep_line.txt 2>&1
then echo "rosdep line: every key resolves"
else echo "rosdep line: UNRESOLVED"; fi
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
  head -n1 "$f" | grep -qE '^#! *[^ ]*/env +python' && echo "${f#$P/}: $(head -n1 "$f" | cut -c3-) (the first python on PATH)" >> /out/shebangs.txt
done < <(find $P -type f -perm -u+x -print0)
echo "scripts with a missing interpreter: $(wc -l < /out/shebangs.txt)"
# every symbolic link stays inside the prefix and resolves (Python's tarfile data filter,
# which the front end unpacks with, refuses an archive over one that does not)
: > /out/links_stray.txt
while IFS= read -r -d '' l; do
  t=$(readlink -m "$l")
  case "$t" in $P/*) [ -e "$l" ] || echo "${l#$P/} -> $(readlink "$l") (dangles)" >> /out/links_stray.txt ;;
    *) echo "${l#$P/} -> $(readlink "$l") (leaves the prefix)" >> /out/links_stray.txt ;; esac
done < <(find $P -type l -print0)
echo "symbolic links that dangle or leave the prefix: $(wc -l < /out/links_stray.txt)"
# files that mean something only in a source build (stage_sim.py's SOURCE_BUILD_ONLY)
: > /out/source_only.txt
for f in share/scensim_isaac_world/built_from; do [ -e "$P/$f" ] && echo "$f" >> /out/source_only.txt; done
echo "files of a source build only: $(wc -l < /out/source_only.txt)"
# every RPATH/RUNPATH entry is $ORIGIN-relative inside the prefix or a system directory
: > /out/rpaths_bad.txt
while IFS= read -r -d '' f; do
  case "$(file -b "$f")" in ELF*) ;; *) continue ;; esac
  rp=$(readelf -d "$f" 2>/dev/null | sed -nE 's/.*\((RPATH|RUNPATH)\).*\[(.*)\]/\2/p')
  [ -n "$rp" ] || continue
  IFS=: ; for e in $rp; do
    case "$e" in
      '$ORIGIN'*) t=$(realpath -m "$(dirname "$f")/${e#\$ORIGIN}"); case "$t" in $P|$P/*) ;; *) echo "${f#$P/}: $e (leaves the prefix)" >> /out/rpaths_bad.txt ;; esac ;;
      /opt/ros/$ROS_DISTRO/*|/usr/lib*|/lib*) ;;
      *) echo "${f#$P/}: $e" >> /out/rpaths_bad.txt ;;
    esac
  done; unset IFS
done < <(find $P -type f -print0)
echo "RPATH/RUNPATH entries outside the prefix and the system: $(wc -l < /out/rpaths_bad.txt)"
EOF

docker run --rm --name "scensim-links-$$" \
    -e SKIP_KEYS="$SKIP_KEYS" \
    -v "$TARBALL":/in/release.tar.zst:ro -v "$OUT/inside.sh":/in/inside.sh:ro -v "$OUT":/out \
    "$IMAGE" bash /in/inside.sh 2>&1 | grep -v "^debconf" | tee "$OUT/summary.txt"

echo "keys: $(wc -l < "$OUT/keys.txt"); apt packages: $(wc -l < "$OUT/apt.txt") ($(wc -l < "$OUT/apt_exec.txt") at run time)" | tee -a "$OUT/summary.txt"
if grep -q "rosdep line: UNRESOLVED" "$OUT/summary.txt"; then
    echo "check_links.sh: FAILED, rosdep does not resolve every key of the release:" | tee -a "$OUT/summary.txt"
    grep -A20 "ERROR" "$OUT/rosdep_line.txt" | head -12 | sed 's/^/  /' | tee -a "$OUT/summary.txt"
    exit 1
fi
if grep -q "rosdep install exited" "$OUT/summary.txt"; then
    echo "check_links.sh: FAILED, rosdep install failed:" | tee -a "$OUT/summary.txt"
    grep -A6 "ERROR" "$OUT/rosdep_install.log" | head -10 | sed 's/^/  /' | tee -a "$OUT/summary.txt"
    exit 1
fi
if grep -q "setup.sh from sh: FAILED" "$OUT/summary.txt"; then
    echo "check_links.sh: FAILED, setup.sh does not source from sh after relocate.sh:" | tee -a "$OUT/summary.txt"
    head -5 "$OUT/setup_sh.txt" | sed 's/^/  /' | tee -a "$OUT/summary.txt"
    exit 1
fi
if [ -s "$OUT/rpaths_bad.txt" ]; then
    echo "check_links.sh: FAILED, RPATH/RUNPATH entries outside the prefix and the system:" | tee -a "$OUT/summary.txt"
    head -20 "$OUT/rpaths_bad.txt" | sed 's/^/  /' | tee -a "$OUT/summary.txt"
    exit 1
fi
if [ -s "$OUT/links_stray.txt" ]; then
    echo "check_links.sh: FAILED, symbolic links that dangle or leave the prefix:" | tee -a "$OUT/summary.txt"
    head -20 "$OUT/links_stray.txt" | sed 's/^/  /' | tee -a "$OUT/summary.txt"
    exit 1
fi
if [ -s "$OUT/source_only.txt" ]; then
    echo "check_links.sh: FAILED, files of a source build only (stage_sim.py leaves them out):" | tee -a "$OUT/summary.txt"
    sed 's/^/  /' "$OUT/source_only.txt" | tee -a "$OUT/summary.txt"
    exit 1
fi
if [ -s "$OUT/shebangs.txt" ]; then
    echo "check_links.sh: FAILED, scripts whose interpreter does not exist on the image, or is env python:" | tee -a "$OUT/summary.txt"
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
