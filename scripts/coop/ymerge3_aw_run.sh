#!/bin/bash
# Roadmap 074 N5 with stock Autoware as B (scripts/coop/ymerge3_aw.py), start to finish, on this host:
# its own CARLA on Town03 (CARLA_PORT), the stack with the ego (`just run`, max_velocity VB, the
# spec's physics, EXTRA as further ego arguments), the runs, then the stack and CARLA stopped and
# $OUT/DONE touched. Everything it writes is under $OUT. Run it detached:
#   OUT=<dir> LANES=<lanes.json> CARLA_DIR=<CARLA 0.9.16> [CARLA_PORT=2010] [VB=10] [GPU=0] [PY=python3] \
#     [EXTRA="lidar_detection_model:=centerpoint"] [ARGS="--seeds 2"] setsid scripts/coop/ymerge3_aw_run.sh > log 2>&1 &
# ARGS may hold several passes separated by ';' (e.g. "--baseline;--seeds 2"), run in order on one stack.
set -u
R="$(cd "$(dirname "$0")/../.." && pwd)"
: "${OUT:?OUT=<output dir>}"; : "${LANES:?LANES=<ymerge3_town03 lanes.json>}"
CARLA_DIR=${CARLA_DIR:?CARLA_DIR=<CARLA 0.9.16 release>}
PORT=${CARLA_PORT:-2010}; export CARLA_PORT=$PORT CARLA_DIR
VB=${VB:-10}; EXTRA=${EXTRA:-lidar_detection_model:=centerpoint}; PY=${PY:-python3}
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
export CARLA_PORT=$PORT PYTHONPATH="$CARLA_DIR/PythonAPI/carla:\${PYTHONPATH:-}"
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
    gap_trace_dir:="$OUT/traces" ego_launch_args:="max_velocity:=$VB $EXTRA" > "$OUT/stack.console" 2>&1 < /dev/null &
SPID=$!
for i in $(seq 1 120); do
    in_env ros2 run swb_launch ego_ready --timeout 10 > "$OUT/ego_ready.log" 2>&1 && break
    sleep 10
done
echo "ego ready after $i tries $(date +%T)" | tee "$OUT/start.log"
IFS=';' read -ra PASSES <<< "${ARGS:- }"
for pass in "${PASSES[@]}"; do
    in_env $PY scripts/coop/ymerge3_aw.py --out "$OUT" --lanes "$LANES" --carla-port $PORT $pass >> "$OUT/stdout.log" 2>&1
    echo "runs [$pass] rc=$? $(date +%T)" | tee -a "$OUT/start.log"
done
kill -TERM -$SPID; sleep 20
kill -TERM -$CPID 2>/dev/null; sleep 5
for p in $(ps -eo pid,comm --no-headers | awk '$2 ~ /^CarlaUE4-Linux/ {print $1}'); do
    tr '\0' ' ' < /proc/$p/cmdline | grep -q "carla-rpc-port=$PORT" && kill $p
done
touch "$OUT/DONE"
