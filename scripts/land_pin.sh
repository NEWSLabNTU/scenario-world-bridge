#!/usr/bin/env bash
# Move one of our repositories to a landed commit and every pin that follows from it (the cascade),
# then this repository's pins in one commit, check_pins green before anything is pushed:
#
#   scripts/land_pin.sh [--dry-run] <repo> <sha> [note]
#
# scripts/land_pin.py does the work; its docstring has the rules.
exec python3 "$(dirname "$0")/land_pin.py" "$@"
