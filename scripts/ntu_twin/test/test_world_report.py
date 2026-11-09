"""world_report.py on a made-up ground: one lanelet, a hole in the mesh under it, a bump."""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

OSM = """<osm>
<node id="1"><tag k="local_x" v="2.0"/><tag k="local_y" v="2.0"/></node>
<node id="2"><tag k="local_x" v="8.0"/><tag k="local_y" v="2.0"/></node>
<node id="3"><tag k="local_x" v="2.0"/><tag k="local_y" v="4.0"/></node>
<node id="4"><tag k="local_x" v="8.0"/><tag k="local_y" v="4.0"/></node>
<way id="10"><nd ref="3"/><nd ref="4"/></way><way id="11"><nd ref="1"/><nd ref="2"/></way>
<relation id="20"><member type="way" ref="10" role="left"/><member type="way" ref="11" role="right"/>
<tag k="type" v="lanelet"/></relation></osm>"""


@pytest.fixture
def world(tmp_path, monkeypatch):
    import world_report
    import world_usd
    # a site whose lanelets carry local tags in the DTM's own frame
    monkeypatch.setattr(world_usd, "SITE", {"name": "test", "lanelet_xy": "local"})
    monkeypatch.setattr(world_usd, "SHIFT", np.zeros(3))
    g, m = tmp_path / "ground", tmp_path / "meshes"
    g.mkdir(), m.mkdir()
    json.dump({"cell": 0.25, "x0": 0.0, "y0": 0.0}, open(g / "dtm.json", "w"))
    dtm = np.zeros((40, 40))
    np.save(g / "dtm.npy", dtm)
    np.save(m / "ground_keep.npy", np.ones((40, 40), bool))
    (tmp_path / "l.osm").write_text(OSM)
    return world_report, g, m, tmp_path / "l.osm"


def test_a_flat_meshed_lane_is_clean(world):
    wr, g, m, osm = world
    r = wr.report(g, m, osm)
    assert r["lane_cells"] > 0 and r["lane_cells_meshed"] == 1.0
    assert r["roughness_p99"] == 0.0 and r["pit_depth_max"] == 0.0


def test_a_hole_and_a_bump_under_the_lane_are_reported(world):
    wr, g, m, osm = world
    keep = np.load(m / "ground_keep.npy")
    keep[12, 20] = False                       # (5.0, 3.0) m: under the lanelet
    np.save(m / "ground_keep.npy", keep)
    dtm = np.load(g / "dtm.npy")
    dtm[13, 9:31:2] = 0.03                     # a strip of 3 cm bumps along y = 3.25 m
    dtm[12, 24] = -0.2                         # a 20 cm pit at (6.0, 3.0) m
    np.save(g / "dtm.npy", dtm)
    r = wr.report(g, m, osm)
    assert r["lane_cells_meshed"] < 1.0
    assert r["roughness_p99"] == pytest.approx(0.03)
    assert r["pit_depth_max"] == pytest.approx(0.2)
