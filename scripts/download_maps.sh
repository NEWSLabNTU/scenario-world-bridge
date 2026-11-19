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
# SCENSIM_MAP_URL=<url> fetches the same zip from a mirror (the checksum still applies).
# SCENSIM_MAP_SOURCE=<dir holding Town01/, Town02/, ...> uses a copy you already have instead.
# SCENSIM_XODR_DIR=<CARLA>/CarlaUE4/Content/Carla/Maps/OpenDrive: the CARLA release's road data,
# which the speed-limit and walkway repairs read.
set -e

MAP_SOURCE="${SCENSIM_MAP_SOURCE:-}"

URL="${SCENSIM_MAP_URL:-https://github.com/NEWSLabNTU/carla-autoware-maps/releases/download/tum-2024-06/carla-autoware-bridge.zip}"
SHA256="6e582242198ae50aa1d1fd410b53609c8b7ecfc6b9696294c498646783ed4838"

DIR="$(cd "$(dirname "$0")/.." && pwd)/data"
ZIP="$DIR/carla-autoware-bridge.zip"
PACK="$DIR/tum-carla-autoware-maps"
OUT="$DIR/carla-autoware-bridge"

# The map repairs live in acb's map tools (carla_map_gen.repairs, roadmap 087), run here in their
# order by carla_map_gen.repair. Each is idempotent, so they also run on maps already present: a
# repair added after a host fetched its maps reaches that host the next time this runs.
# SCENSIM_XODR_DIR: <Town>.xodr or <Town>/main.xodr for the OpenDRIVE repairs (default CARLA_SRC's
# OpenDrive).
ACB_MAP_TOOLS="$(cd "$(dirname "$0")/.." && pwd)/src/autoware_carla_bridge/src/carla_map_gen"
repair_maps() {
    # The map file is a per-file symlink into the source pack: replace it with a real file
    # first, exactly as the projector yaml below.
    for town in "$1"/*/; do
        [ -f "$town/lanelet2_map.osm" ] || continue
        if [ -L "$town/lanelet2_map.osm" ]; then
            src="$(readlink -f "$town/lanelet2_map.osm")"
            rm -f "$town/lanelet2_map.osm"
            cp "$src" "$town/lanelet2_map.osm"
        fi
    done
    # CARLA's towns are georeferenced as +proj=tmerc +lat_0=0 +lon_0=0 +k=1, but a projector
    # yaml without scale_factor makes Autoware and SSv2 take 0.9996: the map 0.04 % small
    # against CARLA, 0.13 m at 330 m (Town01's lanelets 0.159 m p95 off their OpenDRIVE lanes;
    # with 1.0, 0.007 m; roadmap 087). Declare k 1 on every town at 0,0 that names none; the
    # local_coords repair below then rewrites local_x/local_y in that frame.
    for town in "$1"/*/; do
        y="$town/map_projector_info.yaml"
        [ -f "$y" ] || continue
        grep -q '^projector_type:[[:space:]]*TransverseMercator' "$y" || continue
        grep -q '^scale_factor:' "$y" && continue
        grep -qE '^[[:space:]]+latitude:[[:space:]]*0(\.0*)?[[:space:]]*$' "$y" || continue
        grep -qE '^[[:space:]]+longitude:[[:space:]]*0(\.0*)?[[:space:]]*$' "$y" || continue
        if [ -L "$y" ]; then
            src="$(readlink -f "$y")"; rm -f "$y"; cp "$src" "$y"
        fi
        echo "scale_factor: 1.0" >> "$y"
        echo "  $(basename "$town"): projector scale_factor 1.0 (CARLA's k)"
    done
    [ -d "$ACB_MAP_TOOLS/carla_map_gen" ] || {
        echo "ERROR: $ACB_MAP_TOOLS missing: git submodule update --init src/autoware_carla_bridge" >&2
        exit 1
    }
    PYTHONPATH="$ACB_MAP_TOOLS${PYTHONPATH:+:$PYTHONPATH}" python3 -m carla_map_gen.repair "$1" \
        ${SCENSIM_XODR_DIR:+--xodr-dir "$SCENSIM_XODR_DIR"}
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
        echo "ERROR: SCENSIM_MAP_SOURCE='$MAP_SOURCE' is not a directory." >&2
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
scale_factor: 1.0
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
