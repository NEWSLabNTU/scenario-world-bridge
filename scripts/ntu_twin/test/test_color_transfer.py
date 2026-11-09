"""color_transfer.py: a precise map cloud coloured from a coarse, offset copy of the scan."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import color_transfer as ct  # noqa: E402

OFFSET = np.array([0.4, -0.3, 0.1])


def _clouds(seed=0):
    """A precise cloud on a jittered 0.3 m grid with distinct colours, and the 'original': the
    same points shifted by OFFSET with the northing (y) quantized to 0.25 m, as float32 TWD97
    northings near 2.77e6 are."""
    rng = np.random.default_rng(seed)
    g = np.stack(np.meshgrid(np.arange(0, 12, 0.3), np.arange(0, 12, 0.3), [0.0, 0.6]), -1).reshape(-1, 3)
    P = g + rng.uniform(-0.02, 0.02, g.shape)
    rgb = rng.integers(0, 256, (len(P), 3)).astype(np.uint16)
    O = P + OFFSET
    O[:, 1] = np.round(O[:, 1] / 0.25) * 0.25
    return P, O, rgb


def _surface(seed=1, n=60_000):
    """A non-periodic surface (a scan's ground and objects are not a lattice), and its copy
    shifted by OFFSET with the northing quantized to 0.25 m."""
    rng = np.random.default_rng(seed)
    xy = rng.uniform(0, 20, (n, 2))
    z = 0.8 * np.sin(0.7 * xy[:, 0]) * np.cos(0.45 * xy[:, 1]) + 0.3 * np.sin(1.9 * xy[:, 1] + 0.4)
    P = np.c_[xy, z]
    O = P + OFFSET
    O[:, 1] = np.round(O[:, 1] / 0.25) * 0.25
    return P, O


def test_offset_is_recovered_from_a_close_guess():
    # the projection brings the originals within decimetres; ICP takes the rest
    P, O = _surface()
    est = ct.refine_offset(O, P, guess=OFFSET + np.array([0.15, -0.15, 0.15]))
    assert np.allclose(est, OFFSET, atol=0.02)


def test_anisotropic_match():
    P, O, rgb = _clouds()
    got, matched = ct.transfer(P, O - OFFSET, rgb, north_tol=0.15, tight_tol=0.03)
    own = matched & np.all(got == rgb, axis=1)
    assert own.mean() >= 0.99


def test_a_point_with_no_original_near_is_unmatched():
    P, O, rgb = _clouds()
    far = np.vstack([P, [[100.0, 100.0, 0.0]]])
    _, matched = ct.transfer(far, O - OFFSET, rgb, north_tol=0.15, tight_tol=0.03)
    assert not matched[-1]


def test_autoware_transverse_mercator_origin_is_zero():
    f = ct.MapFrame(25.0201, 121.5423)
    x, y = f.from_latlon(np.array([25.0201]), np.array([121.5423]))
    assert abs(x[0]) < 1e-6 and abs(y[0]) < 1e-6


def test_a_grid_offset_survey_frame_is_a_pure_translation(tmp_path):
    # COSS: the map frame is TWD97 less TWD97(map origin), no projection: 100 m east of the
    # origin in TWD97 is (100, 0) in the map, with no meridian-convergence rotation
    (tmp_path / "map_projector_info.yaml").write_text(
        "projector_type: TransverseMercator\nmap_origin: {latitude: 25.0201, longitude: 121.5423, altitude: 25.0}\n")
    site = {"name": "t", "map_dir": str(tmp_path), "survey_frame": "twd97_minus_origin"}
    to_map, from_map = ct.survey_frame(site)
    e0, n0 = from_map(np.array([0.0]), np.array([0.0]))
    x, y = to_map(e0 + 100.0, n0)
    assert abs(x[0] - 100.0) < 1e-6 and abs(y[0]) < 1e-6


def test_a_projected_survey_frame_rotates_by_the_convergence(tmp_path):
    (tmp_path / "map_projector_info.yaml").write_text(
        "projector_type: TransverseMercator\nmap_origin: {latitude: 25.0201, longitude: 121.5423, altitude: 25.0}\n")
    to_map, from_map = ct.survey_frame({"name": "t", "map_dir": str(tmp_path), "survey_frame": "projected"})
    e0, n0 = from_map(np.array([0.0]), np.array([0.0]))
    x, y = to_map(e0 + 100.0, n0)
    assert abs(np.degrees(np.arctan2(y[0], x[0]))) > 0.2      # TWD97's grid is ~0.23 deg off


def test_a_point_the_survey_missed_takes_the_nearest_colour_within_the_fallback_reach():
    P, O, rgb = _clouds()
    far = np.vstack([P, [[5.0, 5.0, 1.5]]])            # 0.9 m above the grid's top layer
    got, matched = ct.transfer(far, O - OFFSET, rgb, north_tol=0.15, tight_tol=0.03, fallback=1.5)
    assert matched[-1] == 2                             # matched by the fallback
    assert (matched[:-1] == 1).mean() >= 0.99           # the rest by the tight match
    got0, matched0 = ct.transfer(far, O - OFFSET, rgb, north_tol=0.15, tight_tol=0.03, fallback=0.0)
    assert matched0[-1] == 0
