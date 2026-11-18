#!/bin/bash
# check_autoware_side.sh: the car's Autoware packages against one Autoware release, as a user
# gets them: that Autoware's binaries (NEWSLab's localrepo, CPU build), the release's
# simulation tarball (the cars' descriptions), and autoware-side.repos imported and built in a
# workspace over that Autoware. Then launch resolution, nothing run: `play_launch dump` of
#   - stock: the user's autoware_launch with what scensim's autoware pilot passes it;
#   - scensim: autoware_sim_profile, the optional tuning.
# Both must resolve (every package, argument, parameter file and component found) and hold the
# sensor kit's nodes. Runs as root in ros:<distro> (CI); on a lab host, in that container too.
#
#   check_autoware_side.sh <autoware release> <localrepo .deb URL> <sim tarball> <repos file> <out>
#
# Autoware's ML models are not installed (gigabytes): the parameter files the stock perception
# loads are stubbed, empty, under a scratch data_path; launch resolution needs them to exist,
# not their values.
set -eu
REL=$1; DEB_URL=$2; TARBALL=$3; REPOS=$4; OUT=$5
mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)
export DEBIAN_FRONTEND=noninteractive
step() { echo "== $* ($(date +%T))"; }

step "Autoware $REL from $DEB_URL"
apt-get update -qq >/dev/null
apt-get install -y -qq --no-install-recommends wget zstd python3-pip python3-vcstool \
    python3-colcon-common-extensions >/dev/null
wget -q -O /tmp/localrepo.deb "$DEB_URL"
apt-get install -y -qq /tmp/localrepo.deb >/dev/null
apt-get update -qq >/dev/null 2>&1
dash=${REL//./-}
pkg=$(apt-cache search --names-only "^ros-$ROS_DISTRO-autoware-launch-$dash\$" | awk '{print $1}')
[ -n "$pkg" ] || { echo "no autoware_launch package for $REL in the localrepo"; exit 1; }
apt-get install -y -qq --no-install-recommends "$pkg" > "$OUT/apt.log" 2>&1
du -sm "/opt/autoware/$REL" | awk '{print "Autoware installed: "$1" MB"}'

step "the simulation side"
mkdir -p /opt/scensim-sim
zstd -dc "$TARBALL" | tar -xf - -C /opt/scensim-sim
/opt/scensim-sim/relocate.sh

step "autoware-side.repos into a workspace over Autoware $REL"
WS=/tmp/aw-ws; rm -rf "$WS"; mkdir -p "$WS/src"
vcs import --shallow "$WS/src" < "$REPOS"
PKGS="tesla_model3_launch tesla_model3_sensor_kit_launch individual_params autoware_sim_profile"
set +u
source "/opt/autoware/$REL/setup.bash"
source /opt/scensim-sim/setup.bash
set -u
(cd "$WS" && colcon build --packages-up-to $PKGS --event-handlers console_cohesion- summary+) \
    > "$OUT/build.log" 2>&1 || { tail -30 "$OUT/build.log"; exit 1; }
tail -1 "$OUT/build.log"
set +u; source "$WS/install/setup.bash"; set -u

step "launch resolution"
python3 -m pip install -q $([ "$ROS_DISTRO" = humble ] || echo --break-system-packages) \
    'play_launch>=0.40'
MAP=$OUT/map; DATA=$OUT/data; mkdir -p "$MAP" "$DATA"
: > "$MAP/lanelet2_map.osm"; : > "$MAP/pointcloud_map.pcd"
CAR="map_path:=$MAP vehicle_model:=tesla_model3 sensor_model:=tesla_model3_sensor_kit \
use_sim_time:=true vehicle:=true rviz:=false"
# what scensim_launch's pilots/autoware.launch.xml passes in stock mode (test_pilots checks
# the two lists are the same)
STOCK="launch_vehicle_interface:=false launch_sensing_driver:=false \
system_run_mode:=logging_simulation traffic_light_recognition/use_high_accuracy_detection:=false"
resolve() {   # <name> <package> <file> <args...>
    local name=$1 pkgname=$2 file=$3; shift 3
    for _ in $(seq 1 40); do
        if play_launch dump -o "$OUT/model-$name.yaml" launch "$pkgname" "$file" "$@" \
                > "$OUT/$name.log" 2>&1; then
            break
        fi
        f=$(grep -oP 'Parameter file not found: \K\S+' "$OUT/$name.log" | head -1 || true)
        case "$f" in
            "$DATA"/*) mkdir -p "$(dirname "$f")"; printf '/**:\n  ros__parameters: {}\n' > "$f"
                       echo "  stubbed ${f#$DATA/}";;
            *) tail -5 "$OUT/$name.log"; echo "$name: does not resolve"; return 1;;
        esac
    done
    for node in crop_box_filter_self gnss_poser imu_corrector vehicle_velocity_converter; do
        grep -q "$node" "$OUT/model-$name.yaml" \
            || { echo "$name: the sensor kit's $node is missing"; return 1; }
    done
    grep -oP 'SystemModel: \S+ \(\K[0-9]+ nodes' "$OUT/$name.log" | sed "s/^/  $name: /"
}
resolve stock autoware_launch autoware.launch.xml $CAR $STOCK data_path:=$DATA
resolve scensim autoware_sim_profile autoware.launch.xml $CAR autoware_release:=$REL data_path:=$DATA
step "Autoware $REL on ROS $ROS_DISTRO: the Autoware side builds and resolves in both modes"
