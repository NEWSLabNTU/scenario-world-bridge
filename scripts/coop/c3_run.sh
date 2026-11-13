#!/bin/bash
# Roadmap 074 C3: one batch of the cut-in with B the Autoware ego, start to finish, on this host.
#
#   OUT=<dir> [RUN=c3b] [C3_VB=6.0] [SEEDS=2] [ONLY=n] [CARLA_PORT=2400] \
#   [EXTRA="lidar_detection_model:=centerpoint"] scripts/coop/c3_run.sh
#
# Starts its own CARLA (CARLA_DIR, port CARLA_PORT) on Town03 and the stack with the ego
# (`just run`, max_velocity C3_VB, the spec's physics, EXTRA as further ego arguments); checks
# that the ego holds C3_VB on the west lane change (a bag through ros2 bag); runs
# scripts/coop/c3.py (24 runs at SEEDS=2); stops the stack; runs the arena's cells at C3_VB
# on the same CARLA (c3.py --arena-port); stops CARLA; touches $OUT/DONE. Everything it
# writes is under $OUT. Run it detached: nohup setsid scripts/coop/c3_run.sh > log 2>&1 &
#
# EXTRA presets: C3a (ground-truth objects, no CUDA) "publish_ground_truth_objects:=true
# occupancy_grid_map_method:=laserscan_based"; C3b (the stack's own perception, needs the GPU)
# "lidar_detection_model:=centerpoint".
set -u
R="$(cd "$(dirname "$0")/../.." && pwd)"
: "${OUT:?OUT=<output dir>}"
RUN=${RUN:-c3b}; VB=${C3_VB:-6.0}; export C3_VB=$VB
PORT=${CARLA_PORT:-2400}; export CARLA_PORT=$PORT
EXTRA=${EXTRA:-lidar_detection_model:=centerpoint}
CARLA_DIR=${CARLA_DIR:?CARLA_DIR=<CARLA 0.9.16 release>}
mkdir -p "$OUT"
ENVSH="$OUT/env.sh"
cat > "$ENVSH" <<EOF
[ -f $R/.env ] && { set -a; source $R/.env; set +a; }   # this checkout's ports and domains
source /opt/autoware/1.5.0/setup.bash
source $R/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=\${EGO_ROS_DOMAIN_ID:-1}
export CYCLONEDDS_URI="file://$R/src/swb_launch/config/cyclonedds-localhost.xml"
EOF
in_env() { bash -c "source $ENVSH >/dev/null 2>&1; $*"; }

cd "$CARLA_DIR"
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json setsid env -u DISPLAY ./CarlaUE4.sh \
    -carla-rpc-port=$PORT -nosound -RenderOffScreen -quality-level=Epic > "$OUT/carla.log" 2>&1 < /dev/null &
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

# the check: the west lane change, the ego must hold max_velocity
CHK="$OUT/check"; rm -rf "$CHK"; mkdir -p "$CHK"
( source "$ENVSH" >/dev/null 2>&1
  exec ros2 bag record -s mcap -o "$CHK/bag" -e '^/localization/kinematic_state$' ) > "$CHK/bag.log" 2>&1 &
BAG=$!; sleep 5
just scenario "$R/scenarios/coop/town03_e2_lane_change_west.xosc" output_directory:="$CHK" > "$CHK/scenario.log" 2>&1
kill -INT $BAG; wait $BAG
VMAX=$(in_env python3 - "$CHK/bag" <<'PY'
import sys, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
ty = {t.name: t.type for t in r.get_all_topics_and_types()}
v = 0.0
while r.has_next():
    tp, d, _ = r.read_next()
    if tp == "/localization/kinematic_state":
        v = max(v, deserialize_message(d, get_message(ty[tp])).twist.twist.linear.x)
print(round(v, 2))
PY
)
echo "check vmax ${VMAX:-none} (limit $VB)" | tee "$OUT/check.log"
if python3 -c "import sys; sys.exit(0 if 0.5 < float('${VMAX:-99}') < $VB + 0.6 else 1)"; then
    in_env python3 scripts/coop/c3.py --out "$OUT" --carla-port $PORT --seeds ${SEEDS:-2} ${ONLY:+--only $ONLY} > "$OUT/stdout.log" 2>&1
else
    echo "check failed: not running C3" | tee -a "$OUT/check.log"
fi
kill -TERM -$SPID; sleep 20
in_env python3 scripts/coop/c3.py --out "$OUT" --arena-port $PORT > "$OUT/arena.log" 2>&1
kill -TERM -$CPID 2>/dev/null; sleep 5
for p in $(ps -eo pid,comm --no-headers | awk '$2 ~ /^CarlaUE4-Linux/ {print $1}'); do
    tr '\0' ' ' < /proc/$p/cmdline | grep -q "carla-rpc-port=$PORT" && kill $p
done
touch "$OUT/DONE"
