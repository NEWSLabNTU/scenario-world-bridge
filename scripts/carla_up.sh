#!/usr/bin/env bash
# Bring up what a scenario on CARLA needs, reusing whatever already runs (roadmaps 046, 073
# S9): CARLA itself (`just carla-start`, a systemd unit), then the stack (`just run
# sim:=carla`, web SWB_WEB_PORT+4: the CARLA world server, the bridge, the agent relay and
# the ego -- Autoware and its vehicle agent, nested, respawned -- in one launch), each waited
# for. Run by `just scenario|acceptance|e2e ... sim:=carla` (the default). Each simulator's
# bring-up is scripts/<sim>_up.sh with this contract; isaac_up.sh is Isaac's; both share
# scripts/stack_up.bash.
#
#   scripts/carla_up.sh <map dir> [<worlds>]
#
# <worlds> is accepted for the contract's sake and unused: CARLA loads a scenario's town when
# the scenario asks. A stack on +4 that is not `just run sim:=carla` (an Isaac one, or the
# old carla_simulation.launch.xml) is refused, not replaced. Stacks started here stay up for
# the next run; their pids are appended to $SIM_UP_STARTED when that is set, so a caller can
# take them down (`just e2e`).
#
# The ego is the stack's (ego_launch, the default); with SIM_UP_EGO=0 (`_scenario-has-ego`) a
# stack started here has none (ego_launch:=none). A separate ego stack already up on +2 (`just
# ego-av`, as two-av and demo-tour start one) is used instead, and the stack is started
# without its own. The stack's ego ends with its episode (a world load, a restarted CARLA:
# acb_bridge exits) and the stack starts it again. A stack found on another map than the
# scenario's, or without an ego the scenario needs, is replaced.
#
# Environment, host settings all (from the justfile): CARLA_PORT, WORLD_PORT, SSV2_PORT,
# AGENT_PORT, SWB_WEB_PORT, SWB_LOG_ROOT, and the waits CARLA_READY_TIMEOUT (s, default 300:
# a first start on a host serves RPC in 50-180 s) and EGO_READY_TIMEOUT (s, default 1200).
set -euo pipefail
map_dir="$1"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$here"
carla_port="${CARLA_PORT:-2000}"
say() { echo "[carla-up] $*" >&2; }
source "$here/scripts/stack_up.bash"
carla_ok() { "$here/src/autoware_carla_bridge/scripts/carla_health.py" --port "$carla_port" > /dev/null 2>&1; }

# 1. CARLA: ready when it serves RPC with a map loaded.
if carla_ok; then
    say "CARLA on $carla_port is up"
else
    if systemctl --user is-active --quiet "carla-run-$carla_port" 2>/dev/null; then
        say "CARLA's unit on $carla_port is starting: waiting"
    else
        say "starting CARLA on $carla_port: just carla-start"
        just carla-start > "$logs/carla.console" 2>&1
    fi
    deadline=$((SECONDS + ${CARLA_READY_TIMEOUT:-300}))
    until carla_ok; do
        [ $SECONDS -gt $deadline ] && {
            say "CARLA did not serve RPC within ${CARLA_READY_TIMEOUT:-300} s (just carla-logs)"; exit 1; }
        sleep 5
    done
    say "CARLA on $carla_port is up ($SECONDS s)"
fi

# 2. The stack on CARLA, with or without its ego, then each part waited for.
stack_up carla "$map_dir"
say "ready: CARLA, the stack (CARLA world server, bridge, relay$([ "$want_ego" = 1 ] && echo ", ego and its agent"))"
