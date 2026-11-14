#!/bin/bash
# Roadmap 074 E3: Autoware's knobs. Runs C3a (B the Autoware ego on ground-truth objects, A a
# light agent) once per knob setting, each a fresh stack with its own obstacle_cruise and
# obstacle_stop parameter files, and records what the running planner loaded.
#
#   PARAMS=<dir with <v>.obstacle_{cruise,stop}.param.yaml> OUT=<dir> [VARIANTS="stock hold ..."]
#   [CELLS='6,0;14,-3;10,0'] [SEEDS=1] [SEED0=0] [GPU=0] [CARLA_PORT=2010] scripts/coop/e3_knobs.sh
#
# The parameter files reach the stack through the profile's obstacle_cruise_param_path and
# obstacle_stop_param_path launch arguments (autoware_sim_profile, autoware_sim_vehicle 2c02f08). Most of these parameters are read once at start (only the PID planner's gains and
# min_accel_during_cruise update at run time), so every setting is its own stack.
set -uo pipefail
R=$(cd "$(dirname "$0")/../.." && pwd)
cd "$R"
: "${PARAMS:?}" "${OUT:?}"
VARIANTS=${VARIANTS:-$(python3 -c "import yaml,sys; print(' '.join(yaml.safe_load(open('$PARAMS/variants.yaml'))))")}
CELLS=${CELLS:-6,0;14,-3;10,0}
C3A="publish_ground_truth_objects:=true occupancy_grid_map_method:=laserscan_based"
MVP=/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner
KEYS="obstacle_cruise.cruise_planning.idling_time obstacle_cruise.cruise_planning.min_ego_accel_for_rss
obstacle_cruise.cruise_planning.safe_distance_margin obstacle_cruise.obstacle_filtering.max_lat_margin
obstacle_cruise.obstacle_filtering.outside_obstacle.ego_obstacle_overlap_time_threshold
obstacle_cruise.cruise_planning.pid_based_planner.min_accel_during_cruise obstacle_stop.stop_planning.stop_margin"
mkdir -p "$OUT"
for v in $VARIANTS; do
    O="$OUT/$v"
    [ -e "$O/DONE" ] && { echo "skip $v (done)"; continue; }
    echo "== $v $(date +%T)"
    cp "$PARAMS/$v".obstacle_*.param.yaml "$OUT/" 2>/dev/null || true
    # record what the planner loaded, once the ego is up (env.sh is written by c3_run.sh)
    # (the planner answers once the ego stack is up; ros2's daemon is not used, so nothing is left)
    ( for i in $(seq 1 120); do
        if [ -f "$O/env.sh" ] && bash -c "set +u; source $O/env.sh >/dev/null 2>&1; ros2 param get --no-daemon $MVP obstacle_cruise.cruise_planning.idling_time" > /dev/null 2>&1; then
          bash -c "set +u; source $O/env.sh >/dev/null 2>&1; for k in $(echo $KEYS); do echo \"\$k \$(ros2 param get --no-daemon $MVP \$k 2>/dev/null | tail -1)\"; done" > "$O/loaded_params.txt" 2>&1
          break
        fi
        sleep 10
      done ) &
    W=$!
    OUT="$O" RUN=e3 C3_VB=6.0 SEEDS=${SEEDS:-1} SEED0=${SEED0:-0} ARENA=0 GPU=${GPU:-0} \
        CARLA_PORT=${CARLA_PORT:-2010} CARLA_DIR=${CARLA_DIR:-$HOME/Downloads/CARLA_0.9.16} CELLS="$CELLS" \
        EXTRA="$C3A obstacle_cruise_param_path:=$PARAMS/$v.obstacle_cruise.param.yaml obstacle_stop_param_path:=$PARAMS/$v.obstacle_stop.param.yaml" \
        scripts/coop/c3_run.sh > "$OUT/$v.log" 2>&1 < /dev/null
    rc=$?
    kill $W 2>/dev/null; wait $W 2>/dev/null
    echo "   $v rc=$rc rows=$(wc -l < "$O/c3.jsonl" 2>/dev/null || echo 0)"
done
echo E3_DONE; touch "$OUT/E3_DONE"
