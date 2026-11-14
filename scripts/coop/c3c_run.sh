#!/bin/bash
# Roadmap 074 C3c: one batch of the cut-in with both cars Autoware (c3c.py), start to finish, on this host.
#
#   OUT=<dir> [C3_VB=6.0] [SEEDS=2] [CELLS='6,0;14,-3'] [ONLY=n] [CARLA_PORT=2400] \
#   [EXTRA="publish_ground_truth_objects:=true occupancy_grid_map_method:=laserscan_based"] [GPU=1] \
#   [EGO_EXTRA="obstacle_cruise_param_path:=... obstacle_stop_param_path:=..."] [ARENA=1] scripts/coop/c3c_run.sh
#
# Starts its own CARLA (CARLA_DIR, port CARLA_PORT) on Town03 and the stack with the ego (B:
# `just run`, max_velocity C3_VB, the spec's physics, EXTRA as further ego arguments); runs
# scripts/coop/c3c.py, which starts A, a background Autoware (`just bg-av`, the same EXTRA),
# fresh for each run; stops the stack; runs the arena's cells (A_aw_perm against B_aw_react) on
# the same CARLA; stops CARLA; touches $OUT/DONE. Everything it writes is under $OUT. Run it
# detached: nohup setsid scripts/coop/c3c_run.sh > log 2>&1 &
#
# The default EXTRA is ground truth for both stacks: a second CenterPoint beside the ego's needs
# a GPU with room for two TensorRT engines and CARLA. EGO_EXTRA goes to B's stack only (E3's knob
# settings: the profile's obstacle_cruise_param_path and obstacle_stop_param_path); A stays stock.
set -u
R="$(cd "$(dirname "$0")/../.." && pwd)"
: "${OUT:?OUT=<output dir>}"
VB=${C3_VB:-6.0}; export C3_VB=$VB
PORT=${CARLA_PORT:-2400}; export CARLA_PORT=$PORT
EXTRA=${EXTRA:-publish_ground_truth_objects:=true occupancy_grid_map_method:=laserscan_based}
CARLA_DIR=${CARLA_DIR:?CARLA_DIR=<CARLA 0.9.16 release>}
GPU_ARGS=""
if [ -n "${GPU:-}" ]; then GPU_ARGS="-graphicsadapter=$GPU"; export CUDA_VISIBLE_DEVICES=$GPU; fi
mkdir -p "$OUT"
ENVSH="$OUT/env.sh"
cat > "$ENVSH" <<EOF
[ -f $R/.env ] && { set -a; source $R/.env; set +a; }   # this checkout's ports and domains
source /opt/autoware/1.5.0/setup.bash
source $R/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=\${EGO_ROS_DOMAIN_ID:-1}
export CYCLONEDDS_URI="file://$R/src/swb_launch/config/cyclonedds-localhost.xml"
${GPU:+export CUDA_VISIBLE_DEVICES=$GPU}
EOF
in_env() { bash -c "source $ENVSH >/dev/null 2>&1; $*"; }

cd "$CARLA_DIR"
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json setsid env -u DISPLAY ./CarlaUE4.sh \
    -carla-rpc-port=$PORT -nosound -RenderOffScreen -quality-level=Epic $GPU_ARGS > "$OUT/carla.log" 2>&1 < /dev/null &
CPID=$!
export PYTHONPATH="$CARLA_DIR/PythonAPI/carla:${PYTHONPATH:-}"
until python3 -c "import carla;c=carla.Client('localhost',$PORT);c.set_timeout(3);c.get_server_version()" 2>/dev/null; do sleep 3; done
python3 -c "import carla;c=carla.Client('localhost',$PORT);c.set_timeout(200);c.load_world('Town03')"

cd "$R"
setsid just run map_path:="$R/data/carla-autoware-bridge/Town03" \
    vehicle_physics:="$R/src/sim_vehicle_spec/generated/tesla_model3/carla_physics.yaml" \
    gap_trace_dir:="$OUT/traces" ego_launch_args:="max_velocity:=$VB $EXTRA ${EGO_EXTRA:-}" > "$OUT/stack.console" 2>&1 < /dev/null &
SPID=$!
for i in $(seq 1 120); do
    in_env ros2 run swb_launch ego_ready --timeout 10 > "$OUT/ego_ready.log" 2>&1 && break
    sleep 10
done
export EXTRA_BG="$EXTRA"
in_env python3 scripts/coop/c3c.py --out "$OUT" --carla-port $PORT --seeds ${SEEDS:-2} ${CELLS:+--cells "$CELLS"} ${ONLY:+--only $ONLY} > "$OUT/stdout.log" 2>&1
kill -TERM -$SPID; sleep 20
[ "${ARENA:-1}" = 1 ] && in_env python3 scripts/coop/c3c.py --out "$OUT" --arena-port $PORT ${CELLS:+--cells "$CELLS"} > "$OUT/arena.log" 2>&1
kill -TERM -$CPID 2>/dev/null; sleep 5
for p in $(ps -eo pid,comm --no-headers | awk '$2 ~ /^CarlaUE4/ {print $1}'); do
    tr '\0' ' ' < /proc/$p/cmdline | grep -q "carla-rpc-port=$PORT" && kill $p
done
touch "$OUT/DONE"
