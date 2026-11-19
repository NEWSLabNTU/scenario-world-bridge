"""Unit tests for scripts/svl_import/svl_to_carla.py's OpenDRIVE repairs (roadmap 087), on a synthetic file shaped
as SVL writes one: a signal id that is not a number (and a controller naming it), a lane width record past its
road's end, and a transverse Mercator about the origin with the origin's UTM position as its false origin. No ROS:

    python3 -m pytest src/scensim_launch/test/test_svl_to_carla.py
"""

import importlib.machinery
import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

_DIR = Path(__file__).resolve().parents[3] / "scripts" / "svl_import"
sys.path.insert(0, str(_DIR))
_loader = importlib.machinery.SourceFileLoader("svl_to_carla", str(_DIR / "svl_to_carla.py"))
_spec = importlib.util.spec_from_loader("svl_to_carla", _loader)
s2c = importlib.util.module_from_spec(_spec)
_loader.exec_module(s2c)

XODR = """<?xml version="1.0" encoding="utf-8"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" vendor="LGSVL">
    <geoReference> +proj=tmerc +lat_0=22.9997300995886 +lon_0=120.227025902544 +k=1 +x_0=215743.6 +y_0=2546179 +datum=WGS84 +units=m +no_defs </geoReference>
  </header>
  <road name="" length="50" id="0" junction="-1">
    <link><successor elementType="junction" elementId="7"/></link>
    <planView><geometry s="0" x="0" y="0" hdg="0" length="50"><line/></geometry></planView>
    <lanes>
      <laneSection s="0">
        <right>
          <lane id="-1" type="driving">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
            <width sOffset="49.9995" a="3.5" b="0" c="0" d="0"/>
            <width sOffset="54.4" a="3.5" b="0" c="0" d="0"/>
            <roadMark sOffset="0" type="solid"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
    <signals>
      <signal s="45" t="-2" id="stop_sign_9" type="206" name="stop"/>
      <signal s="48" t="-2" id="12" type="1000001" name="light"/>
      <signal s="49" t="-2" id="signal_5" type="1000001" name="light2"/>
    </signals>
  </road>
  <road name="" length="10" id="1" junction="7">
    <planView><geometry s="0" x="0" y="50" hdg="0" length="10"><line/></geometry></planView>
    <signals><signalReference s="0" t="0" id="stop_sign_9"/></signals>
  </road>
  <junction id="7" name=""/>
</OpenDRIVE>
"""


def converted(tmp_path, origin=(51, 215743.6, 2546179.0)):
    src, dst = tmp_path / "in.xodr", tmp_path / "out.xodr"
    src.write_text(XODR)
    s2c.rotate_xodr(str(src), str(dst), origin)
    return ET.parse(dst).getroot()


def test_signal_ids_are_numbers_and_references_follow(tmp_path):
    root = converted(tmp_path)
    ids = {g.get("name"): g.get("id") for g in root.iter("signal")}
    assert all(i.lstrip("-").isdigit() for i in ids.values())
    assert ids["light"] == "12"  # a numeric id is kept
    assert len(set(ids.values())) == 3
    assert next(root.iter("signalReference")).get("id") == ids["stop"]
    # the junction's light controllers are rebuilt over the numeric ids
    controlled = {c.get("signalId") for c in root.iter("control")}
    assert controlled == {ids["light"], ids["light2"]}


def test_lane_records_past_their_section_are_dropped(tmp_path):
    root = converted(tmp_path)
    widths = [float(w.get("sOffset")) for w in root.iter("width")]
    assert widths == [0.0]  # 49.9995 (at the end, within 1 mm) and 54.4 (past it) dropped
    assert [m.get("type") for m in root.iter("roadMark")] == ["solid"]


def test_the_georeference_becomes_utm_with_an_offset(tmp_path):
    root = converted(tmp_path)
    header = root.find("header")
    proj = header.find("geoReference").text
    assert proj.startswith("+proj=utm +zone=51 ") and "+lat_0=22.9997300995886" in proj
    assert "+x_0" not in proj and "tmerc" not in proj
    off = header.find("offset")
    assert (float(off.get("x")), float(off.get("y")), off.get("hdg")) == (215743.6, 2546179.0, "0")
    pyproj = pytest.importorskip("pyproj")
    t = pyproj.Transformer.from_crs(pyproj.CRS.from_proj4(proj), "EPSG:4326", always_xy=True)
    lon, lat = t.transform(0 + 215743.6, 0 + 2546179.0)  # the map origin, through the offset
    assert abs(lat - 22.99973) < 1e-4 and abs(lon - 120.22703) < 1e-4


def test_no_origin_leaves_the_georeference(tmp_path):
    root = converted(tmp_path, origin=None)
    assert "tmerc" in root.find("header").find("geoReference").text
    assert root.find("header").find("offset") is None


def test_the_geometry_is_still_turned_into_the_map_frame(tmp_path):
    root = converted(tmp_path)
    g = next(root.iter("geometry"))
    assert (float(g.get("x")), float(g.get("y"))) == (0.0, -0.0)
    assert math.isclose(float(g.get("hdg")), -math.pi / 2)
