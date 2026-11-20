#!/bin/bash
# 074 U3 on Autoware: the four-way stop at Town05 j979, czp's all-way-stop row against stock, with the
# Autoware ego on CARLA against crossing_agent.py's A (078 P2g's grid.sh and env.sh, committed).
#
#   scripts/coop/u3_aw_grid.sh <host env file> <stage>
#
# The host env file (not committed: host settings) sets WORK (outputs, logs), CARLA_PORT, the run's ports
# and domains (SSV2_PORT AGENT_PORT WORLD_PORT SCENSIM_WEB_PORT EGO_ROS_DOMAIN_ID SCENARIO_ROS_DOMAIN_ID
# BG_ROS_DOMAIN_ID), U3_MAP (the j979 stop-sign map's Town05 directory, NAS dataset "2026-11-15 Town05 j979
# all-way stop map") and optionally CUDA_MPS_PIPE_DIRECTORY. CARLA must already serve CARLA_PORT.
#
# Stages (each its own output directory under $WORK, resumable: crossing_aw.py skips finished runs):
#   pace    stage 1's pacing check: arm off, A stop1y_c1.5 from the left, dt -1/0/1, seed 9 (rest_d against dt)
#   smoke   stage 0: the stop-sign map's first runs on a stack; A stop1y_c1.5 from the left, dt 0, seed 0,
#           arm off (stock) and on (czp)
#   s1      stage 1: arms on/off x A stop1y_c1.5/fcfse_c0 x left/right x dt -1/-0.3/0/0.3/1 x seeds 0,1 (80)
#   ref     stage 1's reference: stock on the plain Town05 map, A t1.5p5, left/right, the same dts, seed 0 (10)
#   rerun   stage 1's runs named in $U3_RERUN (arm:dt:rule:seed:side,...; e.g. its errored czp tags), on the
#           stop-sign map, into rerun/
#
# One simulation side (just run ego_launch:=none, the stage's map, the spec's physics) per stage, in its
# own process group, stopped at the end. czp's zones come from the stage's map (czp-zones), once.
set -uo pipefail
HOSTENV=$(realpath "$1"); STAGE=$2
R=$(cd "$(dirname "$0")/../.." && pwd)
set -a; source "$R/.env"; source "$HOSTENV"; set +a
export SCENSIM_LOG_ROOT=$WORK/play_log SCENSIM_CONFIG_HOME=$WORK/scensim-config
mkdir -p "$SCENSIM_CONFIG_HOME"
unset CARLA_DIR
set +u   # the ROS setup scripts read unset variables
source /opt/autoware/1.5.0/setup.bash
source "$R/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=$EGO_ROS_DOMAIN_ID
export CYCLONEDDS_URI="file://$R/src/scensim_launch/config/cyclonedds-localhost.xml"

PLAIN=$R/data/carla-autoware-bridge/Town05
case $STAGE in
  smoke|s1|pace|rerun) MAPDIR=$U3_MAP ;;
  ref) MAPDIR=$PLAIN ;;
  *) echo "stage: pace, smoke, s1, ref or rerun" >&2; exit 2 ;;
esac
OUT=$WORK/$STAGE; mkdir -p "$OUT"
ZONES=$WORK/zones_$(basename "$(dirname "$MAPDIR")")_$(basename "$MAPDIR").yaml
[ -s "$ZONES" ] || ros2 run czp czp-zones --map "$MAPDIR/lanelet2_map.osm" --out "$ZONES" > "$OUT/zones.log" 2>&1 ||
    { echo "czp-zones failed: $OUT/zones.log" >&2; exit 1; }
echo "zones $ZONES: $(grep -c "kind: all_way_stop" "$ZONES") all_way_stop"

ss -ltn | grep -q ":$CARLA_PORT " || { echo "no CARLA on $CARLA_PORT" >&2; exit 1; }
ss -ltn | grep -q ":$SSV2_PORT " && { echo "something already holds $SSV2_PORT" >&2; exit 1; }
cd "$R"; echo "scensim $(git rev-parse --short HEAD) stage $STAGE map $MAPDIR $(date +%T)"
setsid just run ego_launch:=none map_path:="$MAPDIR" \
    vehicle_physics:="$R/src/sim_vehicle_spec/generated/tesla_model3/carla_physics.yaml" > "$OUT/stack.log" 2>&1 < /dev/null &
STACK=$!; echo $STACK > "$OUT/stack.pid"
stop_stack() {
    kill -TERM -"$STACK" 2>/dev/null
    for _ in $(seq 30); do kill -0 "$STACK" 2>/dev/null || return 0; sleep 2; done
    kill -KILL -"$STACK" 2>/dev/null
}
trap 'stop_stack; exit 130' INT TERM
for _ in $(seq 120); do ss -ltn | grep -q ":$SSV2_PORT " && break; sleep 5; done
ss -ltn | grep -q ":$SSV2_PORT " || { echo "stack not up" >&2; stop_stack; exit 1; }
echo "stack up $(date +%T)"

P=(python3 scripts/coop/crossing_aw.py --out "$OUT" --carla-port "$CARLA_PORT" --zones "$ZONES" --map "$MAPDIR"
   --max-load "${U3_MAX_LOAD:-$(( $(nproc) * 3 / 2 ))}" --min-mem "${U3_MIN_MEM:-12}")   # 133: 1.5x the cores; above that 088's verdicts moved
DTS=-1,-0.3,0,0.3,1
case $STAGE in
  pace)
    "${P[@]}" --arms off --rules stop1y_c1.5 --dts=-1,0,1 --seeds 9; echo "pace rc=$? $(date +%T)" ;;
  smoke)
    "${P[@]}" --arms off,on --rules stop1y_c1.5 --dts=0 --seeds 0; echo "smoke rc=$? $(date +%T)" ;;
  s1)
    for side in left right; do
      "${P[@]}" --arms on,off --rules stop1y_c1.5,fcfse_c0 --side $side --dts=$DTS --seeds 0,1
      echo "s1 $side rc=$? $(date +%T)"
    done ;;
  rerun)
    "${P[@]}" --only "${U3_RERUN:?U3_RERUN: arm:dt:rule:seed:side,...}"; echo "rerun rc=$? $(date +%T)" ;;
  ref)
    for side in left right; do
      "${P[@]}" --arms off --rules t1.5p5 --side $side --dts=$DTS --seeds 0
      echo "ref $side rc=$? $(date +%T)"
    done ;;
esac
# final results, and every failed attempt apart (082's signature per one; cells with no verdict)
python3 scripts/coop/crossing_aw.py --out "$OUT" --summary > "$OUT/summary.json" || echo "summary failed" >&2
stop_stack
echo "ALLDONE $STAGE $(date +%T)"
touch "$OUT/DONE"
