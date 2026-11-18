#!/usr/bin/env bash
# The simulation side's release build (roadmap 082 step 2): a merged, release-profile
# install of the simulation packages and the vendored Autoware message packages, built
# from a clean environment with only ROS sourced (no Autoware), then staged into
# `scensim-sim-<version>-<distro>-<arch>.tar.zst` by stage_sim.py.
#
#   scripts/release/build_sim.sh [--version V] [--out DIR] [--jobs N] [--keep-build]
#   (ROS_DISTRO_BUILD=jazzy for Jazzy; default humble)
#
# Run it in a checkout meant for the release (a fresh clone or a worktree, submodules
# initialised): it builds in this checkout's build/, install/ and log/, which it clears
# first, and fetches the vendored packages into build/release-vendor/. Needs: ROS
# (/opt/ros/<distro>), colcon with colcon-cargo-ros2, Rust (cargo on PATH), vcstool,
# zstd, patchelf (on PATH, or PATCHELF=<path>). The tarball and its SHA256SUMS line go to
# --out (default dist/).
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
HERE="$ROOT/scripts/release"
VERSION=""
OUT="$ROOT/dist"
JOBS=""
KEEP_BUILD=0
while [ $# -gt 0 ]; do
    case "$1" in
        --version) VERSION=$2; shift 2 ;;
        --out) OUT=$2; shift 2 ;;
        --jobs) JOBS=$2; shift 2 ;;
        --keep-build) KEEP_BUILD=1; shift ;;
        -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
        *) echo "build_sim.sh: unknown argument $1 (see --help)" >&2; exit 2 ;;
    esac
done

DISTRO=${ROS_DISTRO_BUILD:-humble}
ROS_SETUP=/opt/ros/$DISTRO/setup.bash
[ -f "$ROS_SETUP" ] || { echo "build_sim.sh: no $ROS_SETUP" >&2; exit 2; }
for tool in vcs zstd cargo; do
    command -v "$tool" >/dev/null || { echo "build_sim.sh: $tool not on PATH" >&2; exit 2; }
done
# colcon runs as `python3 -m colcon`: the python3 first on PATH picks colcon and its
# colcon-cargo-ros2 (a virtual environment with a newer plugin, say)
python3 -m colcon --help >/dev/null 2>&1 || { echo "build_sim.sh: python3 -m colcon fails (colcon not installed for $(command -v python3))" >&2; exit 2; }
if [ -z "$VERSION" ]; then
    VERSION=$(git -C "$ROOT" describe --tags --always --dirty)
fi

# The packages of the simulation side (082's table) and everything they build against;
# the vehicle side is left out even where a simulation package names it (082 fix 3).
SIM_PACKAGES=(scenario_world_bridge scenario_agent_relay scensim scensim_launch
    scensim_interfaces scensim_examples scensim_carla_world scensim_isaac_world
    scensim_world_check scenario_test_runner
    colcon-scensim  # colcon's scensim_testbench build type (081): no ROS package depends on it
    # SSv2 loads its entity behaviours with pluginlib at run time; nothing depends on them
    behavior_tree_plugin do_nothing_plugin
    # the vehicle side every non-Autoware pilot on CARLA starts (133, 2026-11-18): acb's
    # bridge (with the blueprint pedal maps) and the car's descriptions
    acb_bridge tesla_model3_description tesla_model3_sensor_kit_description)
VEHICLE_SIDE=(acb_launch aib_launch autoware_sim_profile individual_params tesla_model3_launch
    tesla_model3_sensor_kit_launch)

cd "$ROOT"
VENDOR=build/release-vendor
rm -rf build install log
mkdir -p "$VENDOR"
vcs import --shallow --input "$ROOT/sim.repos" "$VENDOR" >/dev/null

# A clean environment: what the shell had (an Autoware, another workspace) must not leak
# into the release. Only ROS, then this build.
clean_env=(env -i HOME="$HOME" USER="${USER:-}" TERM=dumb LANG=C.UTF-8
    PATH="$HOME/.cargo/bin:$(dirname "$(command -v python3)"):/usr/local/bin:/usr/bin:/bin")
# carla-sys 0.15.1 (carla-rust) generates its C++ bindings with autocxx, which reads the
# headers through libclang. With libclang 18 (Ubuntu 24.04's default) the generated wrappers
# drop `const` and fail to compile ("invalid conversion from 'const ...*'"); libclang 14
# (Ubuntu 22.04's default, and apt's libclang-14-dev on 24.04) works. Use 14 when it is
# there, unless LIBCLANG_PATH names another.
LIBCLANG_PATH=${LIBCLANG_PATH:-}
if [ -z "$LIBCLANG_PATH" ] && [ -d /usr/lib/llvm-14/lib ]; then
    LIBCLANG_PATH=/usr/lib/llvm-14/lib
fi
if [ -n "$LIBCLANG_PATH" ]; then
    clean_env+=(LIBCLANG_PATH="$LIBCLANG_PATH")
else
    echo "build_sim.sh: no libclang 14 (/usr/lib/llvm-14/lib); carla-sys may not compile with a newer one (apt install libclang-14-dev)" >&2
fi

cmake_args=(-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF)
# 082 fix 2: the SSv2 fork builds in Release once its traffic_simulator keeps Boost's
# maybe-uninitialized a warning (fork 00e12ea); an older pin needs the flag here.
if ! grep -q "maybe-uninitialized" src/scenario_simulator_v2/simulation/traffic_simulator/CMakeLists.txt; then
    echo "build_sim.sh: the SSv2 pin predates its Release fix; building with -Wno-error=maybe-uninitialized" >&2
    cmake_args+=("-DCMAKE_CXX_FLAGS=-Wno-error=maybe-uninitialized")
fi

start=$(date +%s)
"${clean_env[@]}" bash -c '
    set -e
    source "$1"; shift
    export CMAKE_POLICY_VERSION_MINIMUM=3.5
    # 082 fix 1: colcon-cargo-ros2 before 889cb5c links from install/<pkg>/lib only.
    if ! python3 -c "from colcon_cargo_ros2.workspace_bindgen import install_layout" 2>/dev/null; then
        echo "build_sim.sh: colcon-cargo-ros2 predates --merge-install support (889cb5c); putting install/ on AMENT_PREFIX_PATH" >&2
        mkdir -p install
        export AMENT_PREFIX_PATH="$PWD/install:$AMENT_PREFIX_PATH"
    fi
    exec "$@"' _ "$ROS_SETUP" \
    python3 -m colcon build --base-paths src "$VENDOR" --merge-install \
        --packages-up-to "${SIM_PACKAGES[@]}" --packages-ignore "${VEHICLE_SIDE[@]}" \
        ${JOBS:+--parallel-workers "$JOBS"} \
        --cargo-args --release \
        --cmake-args "${cmake_args[@]}" \
        --event-handlers console_cohesion- summary+ status-
build_s=$(( $(date +%s) - start ))
# colcon-cargo-ros2 rewrites tracked Cargo.lock files ([[patch.unused]] entries; roadmap
# 077 G79). Put back those that changed and nothing else, so scensim-release.yaml names
# clean commits; any other change stays and marks the commit +dirty.
for repo in $(git submodule foreach -q --recursive 'echo $displaypath') .; do  # submodules first
    changed=$(git -C "$repo" diff --name-only)
    [ "$changed" = "Cargo.lock" ] && git -C "$repo" checkout -- Cargo.lock
done
echo "build_sim.sh: built in ${build_s} s"

"${clean_env[@]}" python3 "$HERE/stage_sim.py" --root "$ROOT" --install install \
    ${PATCHELF:+--patchelf "$PATCHELF"} \
    --vendor "$VENDOR" --repos "$ROOT/sim.repos" --version "$VERSION" --distro "$DISTRO" \
    --out "$OUT" --build-seconds "$build_s"

if [ "$KEEP_BUILD" = 0 ]; then
    rm -rf build log
fi
