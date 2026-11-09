"""The site a world is built for: one YAML per site under sites/, read by every script of the
pipeline instead of the NTU constants they once carried.

    SWB_TWIN_SITE=coss python3 ground_ortho.py ...     (default: ntu)

Keys: `shift` (the tiles' frame minus the map frame), `map_origin` (the Isaac world is the map
frame less it), `ortho_m` (orthophoto resolution), `lanelet_xy` (`local`: lanelet nodes'
local_x/local_y tags, plus `lanelet_offset` when given, are the map frame; `projector`: their
lat/lon through the map's map_projector_info.yaml, under `map_dir`, which needs pyproj). Environment variables in a value expand.
"""
import os
from pathlib import Path

import yaml

SITES = Path(__file__).resolve().parent / "sites"


def load(name):
    path = SITES / f"{name}.yaml"
    if not path.is_file():
        known = sorted(p.stem for p in SITES.glob("*.yaml"))
        raise SystemExit(f"no site {name!r}: known sites are {', '.join(known)}")
    site = yaml.safe_load(path.read_text())
    if "map_dir" in site:
        site["map_dir"] = os.path.expandvars(site["map_dir"])
    return site


def current():
    return load(os.environ.get("SWB_TWIN_SITE", "ntu"))


def map_frame(site):
    """Autoware's TransverseMercatorProjector for the site's map: GeographicLib's exact TM,
    k0 0.9996, about map_projector_info.yaml's origin (autoware_lanelet2_extension)."""
    from pyproj import Transformer

    info = yaml.safe_load(open(os.path.join(site["map_dir"], "map_projector_info.yaml")))
    if info.get("projector_type") != "TransverseMercator":
        raise SystemExit(f"site {site['name']}: projector {info.get('projector_type')!r}, not TransverseMercator")
    o = info["map_origin"]
    tm = (f"+proj=tmerc +lat_0={o['latitude']} +lon_0={o['longitude']} +k=0.9996 "
          "+x_0=0 +y_0=0 +ellps=WGS84 +units=m")
    return Transformer.from_crs("EPSG:4326", tm, always_xy=True)


def lanelet_nodes(root, site):
    """{node id: (x, y, ele)} in the map frame, from a Lanelet2 file's parsed root."""
    tags = {n.get("id"): ({t.get("k"): t.get("v") for t in n.findall("tag")}, n)
            for n in root.findall("node")}
    if site["lanelet_xy"] == "local":
        dx, dy = site.get("lanelet_offset", (0.0, 0.0))
        return {i: (float(t["local_x"]) + dx, float(t["local_y"]) + dy, float(t.get("ele", 0)))
                for i, (t, _) in tags.items()}
    tf = map_frame(site)
    ids = list(tags)
    lon = [float(tags[i][1].get("lon")) for i in ids]
    lat = [float(tags[i][1].get("lat")) for i in ids]
    xs, ys = tf.transform(lon, lat)
    return {i: (float(x), float(y), float(tags[i][0].get("ele", 0))) for i, x, y in zip(ids, xs, ys)}
