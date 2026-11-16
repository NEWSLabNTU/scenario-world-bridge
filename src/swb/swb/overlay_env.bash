#!/bin/bash
# The environment a launch pilot with `overlays:` runs in (roadmap 077, G60): the stack's,
# then each of the pilot's workspaces sourced in order, then the pilot's own command.
#
#   overlay_env.bash [--overlay <setup.bash>]... -- <command> [args...]
#
# no set -e: a setup script that returns non-zero must not stop the pilot

# The stack's DDS settings win over what a workspace's setup.bash sets (as autoware_env.bash).
keep_dds=()
for v in CYCLONEDDS_URI RMW_IMPLEMENTATION ROS_DOMAIN_ID; do
    [ -n "${!v+x}" ] && keep_dds+=("$v=${!v}")
done
while [ "$1" = "--overlay" ]; do
    # shellcheck disable=SC1090
    source "$2"
    shift 2
done
[ "$1" = "--" ] && shift
for kv in "${keep_dds[@]}"; do export "$kv"; done
exec "$@"
