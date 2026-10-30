#!/usr/bin/env python3
"""Route A, the baseline (roadmap 062, O2): OSM -> SUMO netconvert -> OpenDRIVE ->
CommonRoad Scenario Designer -> Lanelet2.

    scripts/open_maps/baseline.py <site.yaml> <out>

Writes <out>/baseline/: `roads.osm` (Overpass, drivable ways and turn restrictions, XML),
`netconvert.xodr` and its log, `lanelet2_map.osm` (crdesigner, Autoware option on). The frame
is the site's: UTM, less the MGRS square and the map origin, so the maps share 059's frame.
Run with the tool venv (`data/maps/.venv`: eclipse-sumo, commonroad-scenario-designer).
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

import requests
import yaml

UA = {"User-Agent": "swb-open-maps/0.1 (research; NEWSLabNTU)"}
DRIVABLE = ("motorway|trunk|primary|secondary|tertiary|unclassified|residential|service|"
            "living_street|motorway_link|trunk_link|primary_link|secondary_link|tertiary_link")


def fetch_roads(bbox, dest):
    s, w, n, e = bbox
    q = (f'[timeout:300];(way["highway"~"^({DRIVABLE})$"]({s},{w},{n},{e});'
         f'relation["type"="restriction"]({s},{w},{n},{e}););(._;>;);out body;')
    r = requests.post("https://overpass-api.de/api/interpreter", data={"data": q}, headers=UA,
                      timeout=400)
    r.raise_for_status()
    dest.write_bytes(r.content)


def netconvert(osm, xodr, frame, log):
    utm_zone = frame["utm_epsg"] - 32600
    ox = frame["mgrs_offset"][0] + frame["map_origin"][0]
    oy = frame["mgrs_offset"][1] + frame["map_origin"][1]
    venv_bin = Path(sys.executable).parent
    cmd = [str(venv_bin / "netconvert"), "--osm-files", str(osm),
           "--proj", f"+proj=utm +zone={utm_zone} +datum=WGS84 +units=m +no_defs",
           "--offset.disable-normalization", "true",
           "--offset.x", str(-ox), "--offset.y", str(-oy),
           # CARLA's Osm2Odr choices, plus OSM's turn lanes and levels (viaducts)
           "--osm.turn-lanes", "true", "--osm.layer-elevation", "6",
           "--geometry.remove", "true", "--ramps.guess", "true", "--edges.join", "true",
           "--junctions.join", "true", "--roundabouts.guess", "true",
           "--tls.discard-simple", "true", "--tls.join", "true",
           "--keep-edges.by-vclass", "passenger", "--remove-edges.isolated", "true",
           "--default.lanewidth", "3.25",
           "--numerical-ids", "true",  # crdesigner reads ids as integers
           "--opendrive-output", str(xodr)]
    with open(log, "w") as f:
        r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
    return r.returncode


def numeric_signal_ids(xodr):
    """netconvert names signals after their junction cluster ("joinedS_..."), even with
    --numerical-ids; crdesigner parses a signal id as an integer. Renumber them, and their
    references, from 1 000 000 000."""
    import re
    text = xodr.read_text()
    ids = {}
    for m in re.finditer(r'<signal\s[^>]*?\bid="([^"]+)"', text):
        if not m.group(1).isdigit() and m.group(1) not in ids:
            ids[m.group(1)] = str(1_000_000_000 + len(ids))
    if not ids:
        return 0

    def sub(m):
        return m.group(1) + ids.get(m.group(2), m.group(2)) + '"'
    text = re.sub(r'(<signal\s[^>]*?\bid=")([^"]+)"', sub, text)
    text = re.sub(r'(<signalReference\s[^>]*?\bid=")([^"]+)"', sub, text)
    text = re.sub(r'(\bsignalId=")([^"]+)"', sub, text)
    xodr.write_text(text)
    return len(ids)


def local_georeference(xodr, frame):
    """netconvert writes the UTM proj string and puts the shift in a header <offset>, which
    crdesigner ignores (its nodes landed at 24.8 S, 114.6 E). Fold the shift into a
    transverse Mercator's false easting and northing instead, and zero the offset."""
    import re
    ox = frame["mgrs_offset"][0] + frame["map_origin"][0]
    oy = frame["mgrs_offset"][1] + frame["map_origin"][1]
    lon0 = (frame["utm_epsg"] - 32600) * 6 - 183
    proj = (f"+proj=tmerc +lat_0=0 +lon_0={lon0} +k=0.9996 +x_0={500000 - ox} +y_0={-oy} "
            f"+datum=WGS84 +units=m +no_defs")
    text = xodr.read_text()
    text = re.sub(r"<geoReference>.*?</geoReference>",
                  f"<geoReference><![CDATA[{proj}]]></geoReference>", text, count=1, flags=re.S)
    text = re.sub(r'<offset x="[^"]*" y="[^"]*"', '<offset x="0" y="0"', text, count=1)
    xodr.write_text(text)
    return proj


def to_lanelet2(xodr, dest):
    from crdesigner.common.config.lanelet2_config import lanelet2_config
    from crdesigner.map_conversion.map_conversion_interface import opendrive_to_lanelet
    lanelet2_config.autoware = True
    lanelet2_config.use_local_coordinates = True
    opendrive_to_lanelet(xodr, dest, lanelet2_config=lanelet2_config)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("site", type=Path)
    ap.add_argument("out", type=Path)
    a = ap.parse_args()
    site = yaml.safe_load(a.site.read_text())
    d = a.out / "baseline"
    d.mkdir(parents=True, exist_ok=True)
    roads = d / "roads.osm"
    if not roads.exists():
        fetch_roads(site["bbox"], roads)
    t0 = time.time()
    rc = netconvert(roads, d / "netconvert.xodr", site["frame"], d / "netconvert.log")
    print(f"netconvert: exit {rc} ({time.time() - t0:.0f} s), log {d / 'netconvert.log'}")
    if rc:
        sys.exit(rc)
    print(f"signal ids renumbered: {numeric_signal_ids(d / 'netconvert.xodr')}")
    print(f"georeference: {local_georeference(d / 'netconvert.xodr', site['frame'])}")
    t0 = time.time()
    to_lanelet2(d / "netconvert.xodr", d / "lanelet2_map.osm")
    print(f"crdesigner: {d / 'lanelet2_map.osm'} ({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
