#!/usr/bin/env bash
# Obtain pre-converted CARLA maps for Autoware (lanelet2 + PCD) in data/carla-autoware-bridge/.
# Source: the map pack of TUM's CARLA-Autoware-Bridge (Town01, 02, 03, 05, 10; LGPL-3.0).
#
# TUM's own LRZ Sync+Share links are dead (HTTP 404 since 2026-07). NEWSLabNTU keeps their
# zip, byte for byte, as a GitHub release; the checksum below is the one TUM's download had.
# The pack is unpacked into data/tum-carla-autoware-maps/ and left as it came; the towns in
# data/carla-autoware-bridge/ are per-file links into it, repaired copies where a repair
# changes a file.
#
# SWB_MAP_URL=<url> fetches the same zip from a mirror (the checksum still applies).
# SWB_MAP_SOURCE=<dir holding Town01/, Town02/, ...> uses a copy you already have instead.
# SWB_XODR_DIR=<CARLA>/CarlaUE4/Content/Carla/Maps/OpenDrive: the CARLA release's road data,
# which the speed-limit and walkway repairs read.
set -e

MAP_SOURCE="${SWB_MAP_SOURCE:-}"

URL="${SWB_MAP_URL:-https://github.com/NEWSLabNTU/carla-autoware-maps/releases/download/tum-2024-06/carla-autoware-bridge.zip}"
SHA256="6e582242198ae50aa1d1fd410b53609c8b7ecfc6b9696294c498646783ed4838"

DIR="$(cd "$(dirname "$0")/.." && pwd)/data"
ZIP="$DIR/carla-autoware-bridge.zip"
PACK="$DIR/tum-carla-autoware-maps"
OUT="$DIR/carla-autoware-bridge"

# The map repairs, in order. Each is idempotent, so they also run on maps already present:
# a repair added after a host fetched its maps reaches that host the next time this runs.
repair_maps() {
    # Four of the five towns ship their traffic lights with the two tags lanelet2 keys
    # on left empty, which makes every signal invisible to Autoware and unaddressable
    # from SSv2. Repair in place -- the map file is a per-file symlink into the source
    # pack, so replace it with a real file first, exactly as the projector yaml below.
    for town in "$1"/*/; do
        [ -f "$town/lanelet2_map.osm" ] || continue
        if [ -L "$town/lanelet2_map.osm" ]; then
            src="$(readlink -f "$town/lanelet2_map.osm")"
            rm -f "$town/lanelet2_map.osm"
            cp "$src" "$town/lanelet2_map.osm"
        fi
    done
    python3 "$(dirname "$0")/repair_lanelet_traffic_lights.py" "$1"
    # Some lanelets ship with no subtype at all (kerb strips, mostly), which makes
    # Autoware's map_based_prediction abort as soon as a pedestrian is near one.
    python3 "$(dirname "$0")/repair_lanelet_subtypes.py" "$1"
    # Lane lines between two lanes of one direction come as `dashed_dashed`, which lanelet2
    # does not know and so lets no car cross: Town05's cars never reach their inner lanes,
    # and its left turns are never routed. Make them `dashed`.
    python3 "$(dirname "$0")/repair_lanelet_line_types.py" "$1"
    # Those inner lanes take the opposite lane's left bound as their own, reversed, and
    # Autoware's behavior_path_planner dies on a route through one. Give each its own way.
    python3 "$(dirname "$0")/repair_lanelet_bound_directions.py" "$1"
    # The pack's local_x/local_y (what swb reads) are a spherical Mercator, 0.7 % long in
    # y against the TransverseMercator Autoware and SSv2 project lat/lon with: 2.2 m on
    # Town01's southern road. Rewrite every node's from its lat/lon.
    python3 "$(dirname "$0")/repair_lanelet_local_coords.py" "$1"
    # Some bounds repeat a point as a second node at the same place; Autoware's fine centre
    # line then has NaN points and length, and behavior_path_planner's walk back over the
    # preceding lanelets never ends. Drop the repeats (after the local coordinates).
    python3 "$(dirname "$0")/repair_lanelet_duplicate_points.py" "$1"
    # No lanelet carries turn_direction, so Autoware's intersection and blind_spot
    # modules never launch. Tag the junction lanelets from their heading change.
    python3 "$(dirname "$0")/repair_lanelet_turn_direction.py" "$1"
    # Some road lanelets start at fresh nodes where their predecessor ends, so Lanelet2
    # finds no successor and routing stops there (Town03's roundabout to its south exit).
    python3 "$(dirname "$0")/repair_lanelet_joins.py" "$1"
    # Town05 and Town10 each carry a right_of_way element with no right-of-way lanelet,
    # which scenario_simulator_v2 refuses to load the map over.
    python3 "$(dirname "$0")/repair_lanelet_right_of_way.py" "$1"
    # No lanelet carries a speed_limit, so Autoware drives every road at lanelet2's urban
    # default, 50 km/h (Town05's 65 mph ring too). Tag each road lanelet from its OpenDRIVE
    # road's speed record (after the local coordinates: it matches in OpenDRIVE's frame).
    # SWB_XODR_DIR: <Town>.xodr or <Town>/main.xodr (default CARLA_SRC's OpenDrive).
    python3 "$(dirname "$0")/repair_lanelet_speed_limits.py" "$1" \
        ${SWB_XODR_DIR:+--xodr-dir "$SWB_XODR_DIR"}
    # The maps are flat, so a walkway under a bridge crosses the bridge's lanes: Autoware's
    # walkway module stops on Town05's raised ring highway for footpaths 10 m below.
    python3 "$(dirname "$0")/repair_lanelet_walkways_under_roads.py" "$1" \
        ${SWB_XODR_DIR:+--xodr-dir "$SWB_XODR_DIR"}
}

# Skip if already present
if [ -d "$OUT" ] && [ "$(ls -A "$OUT" 2>/dev/null)" ]; then
    echo "Maps already available at $OUT"
    ls "$OUT"
    repair_maps "$OUT"
    exit 0
fi

mkdir -p "$DIR"

# --- Download -------------------------------------------------------------------
# Fetch TUM's pack into $PACK, unchanged, unless a source was given or it is already there.
download_pack() {
    if [ -d "$PACK" ] && [ "$(ls -A "$PACK" 2>/dev/null)" ]; then
        return
    fi
    echo "Downloading TUM's CARLA maps for Autoware from $URL"
    if command -v aria2c &>/dev/null; then
        aria2c -c -x4 -s4 --file-allocation=none --auto-file-renaming=false \
            -d "$DIR" -o "$(basename "$ZIP")" "$URL"
    else
        wget -c -O "$ZIP" "$URL"
    fi
    if ! echo "$SHA256  $ZIP" | sha256sum -c --quiet 2>/dev/null; then
        echo "ERROR: checksum mismatch on $ZIP (a cut-short or wrong download)." >&2
        echo "       Delete it and run this again." >&2
        exit 1
    fi
    echo "Checksum verified."
    mkdir -p "$PACK"
    unzip -q -o "$ZIP" -d "$PACK"
    rm -f "$ZIP"
    echo "TUM's pack unpacked to $PACK"
}

if [ -z "$MAP_SOURCE" ]; then
    download_pack
    MAP_SOURCE="$PACK"
fi

# --- Link the source's towns in, then repair ------------------------------------
if [ -n "$MAP_SOURCE" ]; then
    if [ ! -d "$MAP_SOURCE" ]; then
        echo "ERROR: SWB_MAP_SOURCE='$MAP_SOURCE' is not a directory." >&2
        exit 1
    fi

    echo "Linking maps from $MAP_SOURCE"
    mkdir -p "$OUT"
    found=0
    for town in "$MAP_SOURCE"/*/; do
        name="$(basename "$town")"
        if [ -f "$town/lanelet2_map.osm" ]; then
            # Per-file symlinks rather than a directory link, so the projector
            # yaml can be corrected without touching the source pack.
            mkdir -p "$OUT/$name"
            for f in "$town"/*; do
                ln -sfn "$(cd "$(dirname "$f")" && pwd)/$(basename "$f")" "$OUT/$name/$(basename "$f")"
            done
            # SSv2's lanelet_loader accepts TransverseMercator and MGRS and nothing
            # else -- it throws "Unsupported projector type" and the scenario dies
            # before Initialize. Packs in the wild declare other things: the NEWSLab
            # pack says LocalCartesianUTM, the TUM pack says local. Neither matches
            # CARLA's +proj=tmerc +lat_0=0 +lon_0=0 geolocation that acb_bridge's GNSS
            # path assumes anyway. Same origin either way, so the numbers are
            # unchanged; only the declared projector differs. Rewrite anything that is
            # not already one of the two supported types.
            declared="$(sed -n 's/^projector_type:[[:space:]]*//p' \
                "$town/map_projector_info.yaml" 2>/dev/null | tr -d '"'"'"' ')"
            case "$declared" in
              TransverseMercator|MGRS)
                echo "  $name"
                found=$((found + 1))
                continue
                ;;
            esac
            rm -f "$OUT/$name/map_projector_info.yaml"
            cat > "$OUT/$name/map_projector_info.yaml" <<'YAML'
# Rewritten by download_maps.sh: the source pack declares a projector SSv2 rejects
# (it supports TransverseMercator and MGRS only), and one the CARLA GNSS pipeline
# does not use either. CARLA exports geolocation against +proj=tmerc +lat_0=0 +lon_0=0.
projector_type: TransverseMercator
vertical_datum: WGS84
map_origin:
  latitude: 0.0
  longitude: 0.0
  altitude: 0.0
YAML
            echo "  $name (projector '$declared' rewritten to TransverseMercator)"
            found=$((found + 1))
        else
            echo "  skipping $name (no lanelet2_map.osm)"
        fi
    done

    repair_maps "$OUT"

    if [ "$found" -eq 0 ]; then
        echo "ERROR: no town directories with lanelet2_map.osm under $MAP_SOURCE" >&2
        exit 1
    fi
    echo "Linked $found map(s) into $OUT"
    exit 0
fi
