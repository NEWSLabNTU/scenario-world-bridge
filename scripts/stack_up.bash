# The stack's bring-up, shared by scripts/carla_up.sh and scripts/isaac_up.sh (roadmap 073
# S9): find the stack `just run sim:=<sim>` started (web SWB_WEB_PORT+4), start or replace it,
# and wait until the bridge and relay listen and, when the scenario has an ego, the ego's
# ADAPI answers and its agent is registered. Sourced; the caller sets $here and say().
#
# Environment (host settings, from the justfile): SSV2_PORT, AGENT_PORT, SWB_WEB_PORT,
# SWB_LOG_ROOT, SIM_UP_EGO (0: the scenario has no ego), SIM_UP_STARTED (a file the pids of
# stacks started here are appended to), EGO_READY_TIMEOUT (s, default 1200), and on Isaac
# ISAAC_READY_TIMEOUT (s, default 1800).

logs="${SWB_LOG_ROOT:-$here/play_log}"
mkdir -p "$logs"
ssv2_port="${SSV2_PORT:-5555}"
agent_port="${AGENT_PORT:-5560}"
web_base="${SWB_WEB_PORT:-8080}"
web_ego=$((web_base + 2)); web_stack=$((web_base + 4)); web_isaac=$((web_base + 5))

# The play_launch serving web port $1, if any: stacks are told apart by their port. Its Python
# wrapper counts too (`comm` python3): on a loaded host it ran three minutes before the real
# play_launch started, and a run that looked only for the latter started a second simulation
# side, whose bridge then fought the first for the world server.
stack_on_port() {
    local f p
    for f in /proc/[0-9]*/cmdline; do
        p=${f#/proc/}; p=${p%/cmdline}
        { tr '\0' ' ' < "$f"; } 2>/dev/null \
            | grep -qE -- "play_launch launch .*--web-addr [0-9.]*:$1 " \
            && { echo "$p"; return 0; }
    done
    return 1
}
cmdline() { tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null; }
started() { [ -n "${SIM_UP_STARTED:-}" ] && echo "$1" >> "$SIM_UP_STARTED"; return 0; }
alive() { kill -0 "$1" 2>/dev/null; }
listening() { ss -ltn 2>/dev/null | grep -q ":$1 "; }
# `setsid` so the stack is its own process group (kill -TERM -<pid> takes the whole tree).
start() {
    local name="$1"; shift
    say "starting $name: just $*"
    setsid just "$@" > "$logs/$name.console" 2>&1 < /dev/null &
    local pid=$!
    started "$pid"
    echo "$pid"
}
# Stop a stack by its process group, and wait for it.
stop_stack() {
    local pgid
    pgid=$(ps -o pgid= "$1" 2>/dev/null | tr -d ' ')
    [ -n "$pgid" ] || return 0
    kill -TERM -"$pgid" 2>/dev/null
    for _ in $(seq 1 30); do alive "$1" || return 0; sleep 2; done
    kill -KILL -"$pgid" 2>/dev/null
    return 0
}

# stack_up SIM MAP_DIR [just run arguments ...]: the stack up and ready; sets want_ego.
stack_up() {
    local sim="$1" map_dir="$2"; shift 2
    local pid="" line ego="" deadline
    want_ego="${SIM_UP_EGO:-1}"
    map_dir=$(realpath -ms "$map_dir")
    if pid=$(stack_on_port "$web_stack"); then
        line="$(cmdline "$pid") "
        if ! grep -qF -- " swb_launch $sim.launch.xml " <<< "$line"; then
            say "the stack on $web_stack (pid $pid) is not \`just run sim:=$sim\`:"
            say "  $(cut -c1-200 <<< "$line")"
            say "stop it (kill $pid) and run again"
            exit 1
        fi
        if [ "$want_ego" = 1 ] && grep -qF "ego_launch:=none " <<< "$line"; then
            if ego=$(stack_on_port "$web_ego"); then
                say "stack (pid $pid) is up; the ego is the separate stack on $web_ego (pid $ego)"
            else
                say "the stack (pid $pid) has no ego and the scenario needs one: restarting it"
                stop_stack "$pid"; pid=""
            fi
        elif [ "$want_ego" = 1 ] && ! grep -qF "map_path:=$map_dir " <<< "$line"; then
            # A stack's map is its ego's episode's (roadmap 051): one on another map starts again.
            say "the stack (pid $pid) has another map ($(grep -oE 'map_path:=[^ ]*' <<< "$line")): restarting it on $map_dir"
            stop_stack "$pid"; pid=""
        else
            say "stack (pid $pid) is up"
        fi
    fi
    if [ -z "$pid" ]; then
        if listening "$ssv2_port"; then
            say "something outside \`just run\` holds port $ssv2_port:"
            ss -ltnp 2>/dev/null | grep ":$ssv2_port " || true
            exit 1
        fi
        local ego_arg=()
        if [ "$want_ego" = 0 ]; then
            ego_arg=(ego_launch:=none)
        elif ego=$(stack_on_port "$web_ego"); then
            say "a separate ego stack (pid $ego) is up on $web_ego: the stack starts without its own"
            ego_arg=(ego_launch:=none)
        fi
        pid=$(start stack run "sim:=$sim" "map_path:=$map_dir" "${ego_arg[@]}" "$@")
    fi

    # The world server (Isaac's preloads its worlds for minutes), then the bridge and relay.
    if [ "$sim" = isaac ]; then
        deadline=$((SECONDS + ${ISAAC_READY_TIMEOUT:-1800}))
        until just world-health sim:=isaac > /dev/null 2>&1; do
            alive "$pid" || { say "the stack exited; see $logs/stack.console and $logs/bridge/latest/"; exit 1; }
            [ $SECONDS -gt $deadline ] && {
                say "the world server did not answer within ${ISAAC_READY_TIMEOUT:-1800} s"; exit 1; }
            sleep 10
        done
    fi
    # 120 s was not enough on a loaded host (the play_launch wrapper alone took three minutes).
    deadline=$((SECONDS + 300))
    until listening "$ssv2_port" && listening "$agent_port"; do
        alive "$pid" || { say "the stack exited; see $logs/stack.console and $logs/bridge/latest/"; exit 1; }
        [ $SECONDS -gt $deadline ] && {
            say "bridge/relay not listening after 300 s: $(just world-health sim:="$sim" 2>&1 | tail -1)"; exit 1; }
        sleep 2
    done
    say "bridge and relay listen"
    [ "$want_ego" = 1 ] || return 0

    # The ego: its ADAPI and planning up (`_require-ego-stack`, ego_ready), its agent registered.
    deadline=$((SECONDS + ${EGO_READY_TIMEOUT:-1200}))
    until just _require-ego-stack > /dev/null 2>&1 && just _require-agent ego > /dev/null 2>&1; do
        alive "$pid" || { say "the stack exited; see $logs/stack.console and $logs/ego/latest/"; exit 1; }
        if [ $SECONDS -gt $deadline ]; then
            say "the ego's ADAPI or its agent is not ready after ${EGO_READY_TIMEOUT:-1200} s:"
            just _require-ego-stack || true; just _require-agent ego || true
            exit 1
        fi
        sleep 10
    done
    say "ego up and its agent registered"
}
