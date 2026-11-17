# A separate ego or background-AV stack serves one scenario (roadmap 077, G64). `just ego-av`
# and `just bg-av` run outside `just run`, and nothing resets them between scenarios:
# `just scenario-direct` sends no episode_end (G57 is `scensim run`'s). After one scenario, the
# next one on the same stack failed 3 of 3 on 2026-11-16: Autoware stayed in PLANNING after
# its reset, "ego pose is beyond goal", and once the ego hit the background AV. So
# scenario-direct marks the separate stacks a scenario runs against (as it starts, so an
# interrupted run counts), and refuses a marked one.
#
# A marker is a file named <pid>-<start time> in /tmp/scensim-ego-used-<uid> (one per user: /tmp
# is the host's; a test sets ego_used_dir after sourcing this):
# the stack's play_launch and its start time (field 22 of /proc/<pid>/stat, in clock ticks
# since boot). The start time tells a restarted stack that got the same pid from the one
# that ran the scenario, so it is fresh without any sweep. Markers whose process is gone are
# removed when checked. Sourced (scripts/stack_up.bash, the justfile's scenario-direct).

ego_used_dir="/tmp/scensim-ego-used-$(id -u)"

# The start time of process $1 (field 22 of its stat; the comm field may hold spaces and
# parentheses, so count from the last ')'), or nothing when it is gone.
ego_used_start() {
    local s
    s=$(cat "/proc/$1/stat" 2>/dev/null) || return 1
    s=${s##*) }
    # after the comm: state is field 3, so the start time is the 20th word here
    set -- $s
    [ -n "${20:-}" ] && echo "${20}"
}

# The play_launch serving web port $1: of the processes whose command line runs
# `play_launch launch ... --web-addr <ip>:$1`, the one started first (the Python wrapper
# starts the real binary), so the check and the mark name the same process.
# The processes whose command line has `play_launch launch` (pgrep leaves itself out), one
# "<pid> <command line>" each: what the two lookups below read, instead of every process.
ego_used_play_launches() {
    local p
    for p in $(pgrep -f -- "play_launch launch"); do
        echo "$p $({ tr '\0' ' ' < "/proc/$p/cmdline"; } 2>/dev/null)"
    done
}

ego_used_pid() {
    local p line t best="" best_t=""
    while read -r p line; do
        [ -n "$p" ] || continue
        grep -qE -- "play_launch launch .*--web-addr [0-9.]*:$1 " <<< "$line " || continue
        t=$(ego_used_start "$p") || continue
        if [ -z "$best" ] || [ "$t" -lt "$best_t" ]; then best=$p; best_t=$t; fi
    done < <(ego_used_play_launches)
    [ -n "$best" ] && echo "$best"
}

ego_used_key() {
    local t
    t=$(ego_used_start "$1") || return 1
    echo "$1-$t"
}

# Whether the stack whose play_launch is $1 already ran a scenario (exit 0) or not (1).
# Removes markers of processes that are gone or were replaced.
ego_used_check() {
    local key m p
    key=$(ego_used_key "$1") || return 1
    for m in "$ego_used_dir"/*; do
        [ -e "$m" ] || continue
        m=${m##*/}; p=${m%%-*}
        [ "$m" = "$key" ] && return 0
        [ "$(ego_used_key "$p" 2>/dev/null)" = "$m" ] || rm -f "$ego_used_dir/$m"
    done
    return 1
}

# Mark the stack whose play_launch is $1 as having run a scenario.
ego_used_mark() {
    local key
    key=$(ego_used_key "$1") || return 0
    mkdir -p "$ego_used_dir"
    : > "$ego_used_dir/$key"
}

# The background-AV stacks (`just bg-av`: background_av.launch.xml), one line each,
# "<entity> <pid>": the first-started play_launch of each entity's stack.
ego_used_bg() {
    local p t line ent
    declare -A pid start
    while read -r p line; do
        [ -n "$p" ] || continue
        grep -qE -- "play_launch launch .* background_av\.launch\.xml " <<< "$line " || continue
        ent=$(grep -oE -- " entity:=[^ ]+" <<< "$line" | head -1); ent=${ent# entity:=}
        [ -n "$ent" ] || continue
        t=$(ego_used_start "$p") || continue
        if [ -z "${pid[$ent]:-}" ] || [ "$t" -lt "${start[$ent]}" ]; then pid[$ent]=$p; start[$ent]=$t; fi
    done < <(ego_used_play_launches)
    for ent in "${!pid[@]}"; do echo "$ent ${pid[$ent]}"; done
}

# The separate stacks a scenario would run against, one line each, "ego <pid>" or
# "bg <entity> <pid>". $1: the scenario file; $2: whether it has an ego (1 or 0); $3: the
# web port base (SCENSIM_WEB_PORT). The ego on +2 serves it when the stack on +4 has no ego of its
# own (`ego_launch:=none`) or is not up yet (stack_up then starts it without one).
ego_used_separate() {
    local scenario="$1" has_ego="$2" base="$3" e st ent b
    if [ "$has_ego" = 1 ] && e=$(ego_used_pid $((base + 2))); then
        if ! st=$(ego_used_pid $((base + 4))) \
            || grep -qF " ego_launch:=none " <<< " $(tr '\0' ' ' < "/proc/$st/cmdline" 2>/dev/null) "; then
            echo "ego $e"
        fi
    fi
    while read -r ent b; do
        [ -n "$ent" ] || continue
        grep -qE "<ScenarioObject +name=\"$ent\"" "$scenario" && echo "bg $ent $b"
    done < <(ego_used_bg)
    return 0
}

# Refuse (exit 1) when a separate stack the scenario would run against already ran one.
ego_used_refuse() {
    local kind a b
    while read -r kind a b; do
        case "$kind" in
            ego) ego_used_check "$a" && {
                echo "scenario: this ego stack already ran a scenario (pid $a); restart it: \`just ego-av ...\` (roadmap 077, G64)" >&2
                return 1; } ;;
            bg) ego_used_check "$b" && {
                echo "scenario: this background AV stack already ran a scenario (pid $b, entity $a); restart it: \`just bg-av $a ...\` (roadmap 077, G64)" >&2
                return 1; } ;;
        esac
    done <<< "$1"
    return 0
}

# Mark every stack of ego_used_separate's list $1.
ego_used_mark_all() {
    local kind a b
    while read -r kind a b; do
        case "$kind" in ego) ego_used_mark "$a" ;; bg) ego_used_mark "$b" ;; esac
    done <<< "$1"
}
