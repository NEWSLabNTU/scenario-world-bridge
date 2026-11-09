"""site_params: one file per site, NTU's values exactly as the scripts had them."""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import site_params  # noqa: E402

OSM = """<osm><node id="1" lat="25.0202319785" lon="121.54273096174">
<tag k="local_x" v="304774.8604"/><tag k="local_y" v="2768128.0634"/><tag k="ele" v="8.9764"/></node></osm>"""


def test_ntu_is_what_the_scripts_had():
    s = site_params.load("ntu")
    assert s["shift"] == [-0.54, 0.20, 17.30]
    assert s["map_origin"] == [52500.0, 67700.0, 0.0]
    assert s["ortho_m"] == 0.02
    assert s["lanelet_xy"] == "local"


def test_the_default_site_is_ntu(monkeypatch):
    monkeypatch.delenv("SWB_TWIN_SITE", raising=False)
    assert site_params.current()["name"] == "ntu"


def test_an_unknown_site_names_the_known_ones():
    with pytest.raises(SystemExit, match="ntu"):
        site_params.load("nowhere")


def test_local_lanelets_read_their_local_tags():
    nodes = site_params.lanelet_nodes(ET.fromstring(OSM), site_params.load("ntu"))
    assert nodes["1"] == (304774.8604, 2768128.0634, 8.9764)


def test_projected_lanelets_use_autoware_transverse_mercator(tmp_path, monkeypatch):
    (tmp_path / "map_projector_info.yaml").write_text(
        "projector_type: TransverseMercator\nvertical_datum: WGS84\n"
        "map_origin: {latitude: 25.0201, longitude: 121.5423, altitude: 25.0}\n")
    monkeypatch.setenv("COSS_MAP_DIR", str(tmp_path))
    x, y, z = site_params.lanelet_nodes(ET.fromstring(OSM), site_params.load("coss"))["1"]
    # ~0.000431 deg east, ~0.000132 deg north of the origin, at k0 0.9996
    assert x == pytest.approx(0.00043096174 * math.radians(1) * 6378137 * math.cos(math.radians(25.02)) * 0.9996, abs=0.1)
    assert y == pytest.approx(0.0001319785 * math.radians(1) * 6335439 * 0.9996, abs=0.1)
    assert z == 8.9764


def test_the_scripts_take_ntu_by_default(monkeypatch):
    import importlib
    monkeypatch.delenv("SWB_TWIN_SITE", raising=False)
    mods = {m: importlib.reload(importlib.import_module(m))
            for m in ("ground_ortho", "smooth_dtm", "build_meshes", "world_usd")}
    for m in ("smooth_dtm", "build_meshes", "world_usd"):
        assert mods[m].SHIFT.tolist() == [-0.54, 0.20, 17.30], m
    assert tuple(mods["world_usd"].MAP_ORIGIN) == (52500.0, 67700.0, 0.0)
    assert mods["ground_ortho"].ORTHO == 0.02
    assert mods["smooth_dtm"].PIT_DEPTH == 0.15


def test_the_scripts_take_the_site_they_are_given(monkeypatch, tmp_path):
    import importlib
    monkeypatch.setenv("SWB_TWIN_SITE", "coss")
    monkeypatch.setenv("COSS_MAP_DIR", str(tmp_path))
    mods = {m: importlib.reload(importlib.import_module(m))
            for m in ("ground_ortho", "smooth_dtm", "build_meshes", "world_usd")}
    for m in ("smooth_dtm", "build_meshes", "world_usd"):
        assert mods[m].SHIFT.tolist() == [0.0, 0.0, 0.0], m
    assert tuple(mods["world_usd"].MAP_ORIGIN) == (0.0, 0.0, 0.0)
    assert mods["ground_ortho"].ORTHO == 0.04
    assert mods["smooth_dtm"].PIT_DEPTH == 0.04


def test_a_local_offset_lanelet_frame_needs_no_projection(monkeypatch):
    # COSS: map = local_x/local_y + lanelet_offset, the same for all 2276 nodes (to 0.1 mm),
    # so world_usd under Isaac's Python (no pyproj) reads the lanelets as Autoware does
    monkeypatch.setenv("COSS_MAP_DIR", "/nonexistent")
    site = site_params.load("coss")
    x, y, z = site_params.lanelet_nodes(ET.fromstring(OSM), site)["1"]
    assert (x, y) == pytest.approx((304774.8604 - 304731.3793, 2768128.0634 - 2768113.4495))
    assert z == 8.9764
