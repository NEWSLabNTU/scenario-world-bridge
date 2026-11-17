#!/bin/bash
# The environment an autoware pilot runs in: what `just ego-av` builds, in the same order
# (roadmap 077, G1), then the pilot's own command.
#
#   autoware_env.bash <workspace> <autoware setup> <overlay dir|""> <overlay on: true|false>
#                     <cuda ndt dir|""> <tensorrt lib dir|""> [--overlay <setup.bash>]...
#                     -- <command> [args...]
#
# 1. Autoware (its setup.bash); 2. the Autoware overlay (roadmap 042: the blind_spot, NDT step
# and pose_instability fixes); 3. CUDA NDT, when built (its CUDA libraries first); 4. acb's and
# this workspace's installs; 5. the pilot's own workspaces (`overlays:`, G60), in order;
# 6. TensorRT 10.8.0 first on LD_LIBRARY_PATH, after every setup that rewrites it. A missing
# piece warns, as the developer recipes do, and the pilot starts.
# no set -e: a setup script that returns non-zero must not stop the pilot
ws="$1" setup="$2" overlay="$3" overlay_on="$4" ndt="$5" trt="$6"
shift 6
extra=()
while [ "$1" = "--overlay" ]; do
    extra+=("$2")
    shift 2
done
[ "$1" = "--" ] && shift

# The stack's DDS settings win: Autoware's setup.bash sets its own CYCLONEDDS_URI (and RMW),
# and a pilot on another profile than the stack's is never discovered (seen 2026-11-15: ego_ready
# found nothing on /api/operation_mode/state for 600 s).
keep_dds=()
for v in CYCLONEDDS_URI RMW_IMPLEMENTATION ROS_DOMAIN_ID; do
    [ -n "${!v+x}" ] && keep_dds+=("$v=${!v}")
done

# shellcheck disable=SC1090
source "$setup"
source "$ws/scripts/autoware_overlay.bash"
scensim_autoware_overlay_source "${overlay:-$ws/third_party/autoware_overlay}" "$overlay_on"
ndt="${ndt:-$ws/third_party/cuda_ndt_matcher}"
if [ -f "$ndt/install/setup.bash" ]; then
    source "$ndt/install/setup.bash"
    [ -s "$ndt/install/.scensim_cuda_path" ] && \
        export LD_LIBRARY_PATH="$(cat "$ndt/install/.scensim_cuda_path")/lib64:${LD_LIBRARY_PATH:-}"
    # roadmap 080: a build from before the rename has the stamp under the old name, not read
    [ ! -e "$ndt/install/.scensim_cuda_path" ] && [ -s "$ndt/install/.swb_cuda_path" ] && \
        echo "scensim: cuda_ndt_matcher's stamp has the old name and is not read;" \
             "move it: mv $ndt/install/.swb_cuda_path $ndt/install/.scensim_cuda_path" >&2
fi
[ -f "$ws/src/autoware_carla_bridge/install/setup.bash" ] && \
    source "$ws/src/autoware_carla_bridge/install/setup.bash"
source "$ws/install/setup.bash"
for f in "${extra[@]}"; do
    # shellcheck disable=SC1090
    source "$f"
done
source "$ws/scripts/autoware_tensorrt.bash"
scensim_tensorrt_source "${trt:-$ws/third_party/tensorrt/lib}"
for kv in "${keep_dds[@]}"; do export "$kv"; done
exec "$@"
