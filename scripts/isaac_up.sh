#!/usr/bin/env bash
# Bring up what a scenario on Isaac Sim needs, reusing whatever already runs (roadmaps 043,
# 073 S9): the stack (`just run sim:=isaac`, web SWB_WEB_PORT+4: the Isaac world server, the
# bridge, the agent relay and the ego -- aib's Autoware and its vehicle agent, nested,
# respawned -- in one launch), waited for part by part. Run by `just scenario|acceptance|e2e
# ... sim:=isaac` (roadmap 046: each simulator's bring-up is scripts/<sim>_up.sh, with this
# contract; carla_up.sh is CARLA's; both share scripts/stack_up.bash).
#
#   scripts/isaac_up.sh <map dir> [<world>[,<world>...]]
#
# The worlds are what a world server started here preloads (default: the map dir's); a
# running one is used as it is. A stack on +4 that is not `just run sim:=isaac` is refused, not
# replaced, and so is a separate world server (`just isaac-world`, +5): the stack starts its
# own on the same port. SIM_UP_EGO=0 and a separate ego on +2 as in carla_up.sh. Stacks
# started here stay up for the next run; their pids are appended to $SIM_UP_STARTED when that
# is set, so a caller can take them down (`just e2e`).
#
# Environment, host settings all (from the justfile): ISAAC_PORT, SSV2_PORT, AGENT_PORT,
# SWB_WEB_PORT, SWB_LOG_ROOT (the stack's console goes there; default the repo's play_log),
# and the waits ISAAC_READY_TIMEOUT (s, default 1800: a town's first render on a host compiles
# its materials, so the budget is the host's) and EGO_READY_TIMEOUT (s, default 1200).
set -euo pipefail
map_dir="$1"
worlds="${2:-$map_dir}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$here"
say() { echo "[isaac-up] $*" >&2; }
source "$here/scripts/stack_up.bash"

if pid=$(stack_on_port "$web_isaac"); then
    say "a separate Isaac world server (\`just isaac-world\`, pid $pid) runs on $web_isaac;"
    say "\`just run sim:=isaac\` starts its own on the same port: stop it (kill $pid) and run again"
    exit 1
fi

# Only worlds that exist: a map with no isaac/world.yaml runs on a ground plane.
world_args=()
if [ -n "$worlds" ]; then
    keep=()
    IFS=',' read -ra entries <<< "$worlds"
    for w in "${entries[@]}"; do
        if [ -d "$w" ]; then
            [ -f "$w/isaac/world.yaml" ] && keep+=("$w") \
                || say "no $w/isaac/world.yaml: not preloaded (a ground plane)"
        elif [ -e "$w" ]; then keep+=("$w")
        else say "no world $w: not preloaded"; fi
    done
    [ ${#keep[@]} -gt 0 ] && world_args=("world:=$(IFS=,; echo "${keep[*]}")")
fi

stack_up isaac "$map_dir" "${world_args[@]}"
say "ready: the stack (Isaac world server, bridge, relay$([ "$want_ego" = 1 ] && echo ", ego and its agent"))"
