#!/usr/bin/env bash
# test_relocate.sh: relocate.sh on a real sim tarball, under prefixes a sed replacement
# would break (& | and a space), then after a move: setup.sh sources from sh and setup.bash
# from bash each time, and no text file keeps the placeholder or the old place.
#
#   scripts/release/test_relocate.sh <scensim-sim-....tar.zst> [--relocate <relocate.sh>]
#
# --relocate replaces the tarball's own relocate.sh (to test a change before a rebuild).
# Needs ROS at /opt/ros/<distro>; writes only under a temporary directory.
set -euo pipefail
TARBALL="" RELOC=""
while [ $# -gt 0 ]; do
    case "$1" in
        --relocate) RELOC=$(readlink -f "$2"); shift 2 ;;
        -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
        *) TARBALL=$(readlink -f "$1"); shift ;;
    esac
done
[ -f "$TARBALL" ] || { echo "test_relocate.sh: no tarball (see --help)" >&2; exit 2; }
distro=$(basename "$TARBALL" .tar.zst | sed -E 's/^scensim-sim-.*-([a-z]+)-[a-z0-9_]+$/\1/')
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
first="$tmp/a&b|c d/sim"
second="$tmp/moved & again|x y/sim"
mkdir -p "$first"
zstd -dc "$TARBALL" | tar -xf - -C "$first"
[ -n "$RELOC" ] && cp "$RELOC" "$first/relocate.sh"

check() {  # check <prefix> <what>
    local p=$1 fail=0
    "$p/relocate.sh"
    if env -i HOME="$HOME" PATH=/usr/bin:/bin sh -c ". /opt/ros/$distro/setup.sh && . \"\$1/setup.sh\" && case \":\$AMENT_PREFIX_PATH:\" in *\":\$1:\"*) ;; *) exit 1 ;; esac" _ "$p"; then
        echo "ok   $2: setup.sh sources from sh, AMENT_PREFIX_PATH holds the prefix"
    else
        echo "FAIL $2: setup.sh from sh"; fail=1
    fi
    if env -i HOME="$HOME" PATH=/usr/bin:/bin bash -c "source /opt/ros/$distro/setup.bash && source \"\$1/setup.bash\" && case \":\$AMENT_PREFIX_PATH:\" in *\":\$1:\"*) ;; *) exit 1 ;; esac" _ "$p"; then
        echo "ok   $2: setup.bash sources from bash"
    else
        echo "FAIL $2: setup.bash from bash"; fail=1
    fi
    local left
    left=$(grep -rlIF -e '@SCENSIM_PREFIX@' "$p" --exclude=relocate.sh | head -3 || true)
    [ -z "$left" ] || { echo "FAIL $2: the placeholder is left in: $left"; fail=1; }
    [ "$(cat "$p/.scensim-prefix")" = "$p" ] || { echo "FAIL $2: .scensim-prefix"; fail=1; }
    return $fail
}

rc=0
check "$first" "unpacked under '$first'" || rc=1
mkdir -p "$(dirname "$second")"; mv "$first" "$second"
check "$second" "moved to '$second'" || rc=1
old=$(grep -rlIF -e "$first" "$second" | head -3 || true)
[ -z "$old" ] || { echo "FAIL the old place is left in: $old"; rc=1; }
[ $rc = 0 ] && echo "test_relocate.sh: passed" || echo "test_relocate.sh: FAILED"
exit $rc
