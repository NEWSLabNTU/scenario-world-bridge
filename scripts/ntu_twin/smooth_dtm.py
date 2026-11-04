#!/usr/bin/env python3
"""Re-derive the DTM's non-ground cells from its ground cells alone (roadmap 059 N3/N8).

    smooth_dtm.py GROUND_DIR [--lanes MAP_DIR/lanelet2_map.osm]

ground_ortho.py's DTM keeps ground cells' heights and fills the rest, and that fill takes
heights from non-ground points (parked scooters, kerbs of planters, a stopped car): on r01 it
left 0.2-0.3 m bumps on the road where the scan has no points at all (most cells there are
empty), and a car at creep torque could not climb them (an Autoware drive stood still with
acceleration commanded). Only 17.8 % of the site's cells are ground.

A non-ground cell is capped at the ground's own height there plus CAP: the normalized
Gaussian average of the ground cells around it,
at sigma 0.5, 1, 2, 4, 8, 16, 32 m in turn, the first whose ground weight reaches WEIGHT, and
at 64 m with any ground at all. Capped, not replaced: replacing every non-ground cell made the
road rougher (single noisy ground cells stood out against smooth neighbours; |z - 3x3 median|
p99 on the roads 0.083 m against the raw 0.058), while the cap only takes out what stands above
the ground (the bumps, and the fill's roof heights inside building footprints, up to 26 m off).
On and beside the lanes (--lanes), where the scan has returns, the surface is the scan's
lowest return (zmin, GROUND_DIR/zmin.npy) taken as a ZMIN_MEDIAN x ZMIN_MEDIAN median, unless
that stands more than CANOPY above the window's lower quarter (a cell whose lowest return is
a crown). Along r01's last lanelet the road runs above a drop: SMRF's 16 m opening took the
raised road for an object, no cell there was ground, and the fill put the road 6 m down
(zmin was 11.84-11.89 in every cell, the raw DTM 5.8-7.3); the car braked for its own road,
slid towards the drop and NDT lost it (N8). Under dense canopy the crown test is not enough:
no return reaches the road, the window's lower quarter is crown too, and the lane rule put the
road 5-7 m up in the trees (seven places found from the CARLA build of the same site, e.g.
map-minus-origin (489.7, 232.3): raw 29.71, zmin median 35.51, lower quarter 35.49), and 14 m
up on a roof at (473.6, 166.4). So a median more than RAISED above the capped surface counts
only where it joins the road: where a chain of lane cells, each within STEP of the next, links
it to one within RAISED of the capped surface. A raised road runs on into the road it leaves
(r01's: 11.85 m all along, 5-6 m above the cap at the drop); a crown or a roof is an island,
meters above the road at every edge. An island is dropped only where it stands more than STEP
above every joined cell within LANE_WINDOWS[-1]: on a raised deck whose scan sees through in
places (map-minus (-124, -91), 12 m, zmin 6-12) the medians mixing the two are islands too,
but below the deck beside them, and stay. Then narrow pits close: a cell more than PIT_DEPTH below its PIT_WINDOW x PIT_WINDOW median
is raised to that median less PIT_DEPTH. Through a drain grate at r01's road edge the scanner
saw the pit under it, ground-classified 2 m below the road; the right wheels dropped in, the
car slid sideways, and the EKF, rejecting NDT's poses, ended the drive in an MRM (N8). That
pit was wider than the window: on and beside the lanes (--lanes, LANE_MARGIN around the
lanelets), where a pit is always wrong, the closing repeats with windows of LANE_WINDOWS,
raising a cell no higher than the lanelets' surface there: the ramp into the car park at
(469, 153-170) is narrower than the widest window, and closing it as a pit lifted its floor
towards the walls (close_lane_pits). Elsewhere one window keeps kerbs and ditches. Then
the whole grid, ground cells too, takes a MEDIAN x MEDIAN median and a Gaussian of
SIGMA_CELLS: the scan's ground cells carry single-cell steps of 0.1-0.2 m (sparse points, the
SMRF band), and one stopped a car at creep torque on r01 after the cap. Kerbs round off over
about a metre. Last, the lanes (ROAD_MARGIN around the lanelets) are kept off the lanelets'
surface, as roadmap 072's carla_meshes.py does with the same constants (deb22af), so both
grounds agree: a lane cell more than ELE_OFF above the median ele of the ELE_K nearest lanelet
nodes within ELE_R comes down to it, unless the lane rule above took the cell from the scan's lowest return: there
the scan saw the road, and the lanelets are what is off (on r01 at (131, 182) their ele is
11.79 m, the scan's road 12.22-12.28 m over 330 m^2; lowering it there puts the ground 0.45 m
under the point cloud map). Under dense canopy the scan has no road, the lane rule
cannot use zmin, and the fill invents a rise 0.3-0.9 m over the lanelets: the Isaac tour's
legs 07, 24, 25, 71 and 73 stopped at such rises (e.g. map-minus-origin (101, 325), and the
south-west drop at (-289, -151)). Only ever lowered: raising the road to the lanelets made 1 m
steps at the edge of the raised deck at (-124, -91). The moved cells and the lane within
BLEND_NEAR of them then take the Gaussian (BLEND_SIGMA) of the lane surface where it is lower,
and a cell off the lane within FOOT_NEAR of a moved one, more than FOOT_ABOVE over the nearest,
comes down to it. The original is kept as dtm_raw.npy (and read from it on a rerun);
dtm.npy is rewritten. Then: build_meshes.py --ground-only, world_usd.py, register_lanelets.py.
"""
import json
import os
import sys
import warnings

import numpy as np
from scipy import ndimage

SIGMAS_M = (0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0)
WEIGHT = 0.05  # of a full neighbourhood of ground (17.8 % of the site is ground)
CAP = 0.05     # m above the ground-only estimate
PIT_WINDOW = 9  # cells (2.25 m)
PIT_DEPTH = 0.15  # m
LANE_WINDOWS = (9, 17, 33)  # cells
LANE_MARGIN = 3.0  # m
ZMIN_MEDIAN = 5  # cells
CANOPY = 0.3  # m above the window's 25th percentile of zmin
RAISED = 1.0  # m above the capped surface: a zmin median this high must join the road
STEP = 0.3    # m between neighbouring lane cells' zmin medians on such a chain
MEDIAN = 5     # cells (1.25 m)
SIGMA_CELLS = 1.0
# roadmap 072's carla_meshes.py (deb22af), the same values: ROAD_MARGIN, ELE_*, BLEND_*, and its
# SPIKE_NEAR and 0.3 m for the foot
ROAD_MARGIN = 0.25  # m
ELE_K, ELE_R, ELE_OFF = 6, 6.0, 0.25  # lanelet nodes, m, m
BLEND_NEAR, BLEND_SIGMA = 1.5, 0.75  # m
FOOT_NEAR, FOOT_ABOVE = 2.0, 0.3  # m
SHIFT = np.array([-0.54, 0.20, 17.30])  # the DTM's frame = map + SHIFT (world_usd.SHIFT)


def smooth(dtm, ground, cell):
    out = dtm.copy()
    g = ground.astype(np.float32)
    z = np.where(ground, dtm, 0.0).astype(np.float32)
    todo = ~ground
    for s in SIGMAS_M:
        w = ndimage.gaussian_filter(g, s / cell, mode="nearest")
        v = ndimage.gaussian_filter(z, s / cell, mode="nearest")
        last = s == SIGMAS_M[-1]  # the widest takes any ground at all: no raw fill is left
        ok = todo & (w >= (1e-6 if last else WEIGHT))
        out[ok] = v[ok] / w[ok]
        todo &= ~ok
        print(f"sigma {s:4.1f} m: {ok.sum():,} cells, {todo.sum():,} left", flush=True)
    return out, todo


def joined(cells, z, seed):
    """`cells` linked to a `seed` cell through 4-neighbours of `cells` whose z differ by
    less than STEP."""
    from scipy import sparse
    from scipy.sparse.csgraph import connected_components

    idx = np.full(z.shape, -1, np.int64)
    n = int(cells.sum())
    idx[cells] = np.arange(n)
    a, b = [], []
    for s0, s1 in (((slice(None), slice(None, -1)), (slice(None), slice(1, None))),
                   ((slice(None, -1), slice(None)), (slice(1, None), slice(None)))):
        k = cells[s0] & cells[s1] & (np.abs(z[s0] - z[s1]) < STEP)
        a.append(idx[s0][k])
        b.append(idx[s1][k])
    a, b = np.concatenate(a), np.concatenate(b)
    _, lab = connected_components(sparse.coo_matrix((np.ones(len(a), bool), (a, b)), (n, n)),
                                  directed=False)
    ok = np.zeros(lab.max() + 1, bool)
    ok[lab[seed[cells]]] = True
    out = np.zeros(z.shape, bool)
    out[cells] = ok[lab]
    return out


def lane_ele(osm, meta, cells):
    """The lanelets' surface at each of `cells` (a mask): the median ele of the ELE_K nearest
    lanelet nodes within ELE_R, in the DTM's frame; NaN with no node near.
    Returns (rows, cols, ele) of the cells."""
    import xml.etree.ElementTree as ET

    from scipy.spatial import cKDTree

    nodes = []
    for n in ET.parse(osm).getroot().findall("node"):
        t = {x.get("k"): x.get("v") for x in n.findall("tag")}
        nodes.append((float(t["local_x"]), float(t["local_y"]), float(t.get("ele", 0))))
    nodes = np.array(nodes) + SHIFT
    c = meta["cell"]
    jj, ii = np.nonzero(cells)
    P = np.c_[meta["x0"] + (ii + 0.5) * c, meta["y0"] + (jj + 0.5) * c]
    d, k = cKDTree(nodes[:, :2]).query(P, k=ELE_K, distance_upper_bound=ELE_R)
    zz = np.where(np.isfinite(d), nodes[np.minimum(k, len(nodes) - 1), 2], np.nan)
    with warnings.catch_warnings():  # all-NaN rows (no node within ELE_R) give NaN, as meant
        warnings.simplefilter("ignore", RuntimeWarning)
        return jj, ii, np.nanmedian(zz, axis=1)


def close_lane_pits(out, lanes, ceiling):
    """Close pits on the lanes with each of LANE_WINDOWS in turn, as far as `ceiling` (the
    lanelets' surface; NaN where unknown, no limit there). A cell is raised no higher than
    the lanelets say the road is: a pit is a hole below the road (r01's drain grate, 2 m under
    lanelets at road height), while a cutting the lanelets go down into is the road itself.
    The ramp into the underground car park at map-minus-origin (469, 153-170) is 6 m wide,
    and the 33-cell (8.25 m) median there is the street above: unlimited, the closing lifted
    the ramp's floor 0.15-0.4 m towards both walls, and Autoware's obstacle stop took the
    risen floor beside the lane for an obstacle (072, tour legs 27 and 81)."""
    top = np.where(np.isfinite(ceiling), ceiling, np.inf)
    for w in LANE_WINDOWS:
        up = np.maximum(out, np.minimum(ndimage.median_filter(out, w) - PIT_DEPTH, top))
        out = np.where(lanes, up, out)
    return out


def to_lane_ele(out, osm, meta, road, scan):
    """`out` with its `road` cells kept off the lanelets' surface (the module's last step),
    except the `scan` cells, whose height is the scan's own road.
    Returns the new grid and the number of cells lowered."""
    c = meta["cell"]
    jj, ii, e = lane_ele(osm, meta, road)
    res = out.astype(np.float64)
    fix = np.isfinite(e) & (res[jj, ii] - e > ELE_OFF) & ~scan[jj, ii]
    res[jj[fix], ii[fix]] = e[fix]
    moved = np.zeros(out.shape, bool)
    moved[jj[fix], ii[fix]] = True
    if not fix.any():
        return res, 0
    near = ndimage.binary_dilation(moved, iterations=int(round(BLEND_NEAR / c))) & road
    sg = BLEND_SIGMA / c
    wsum = ndimage.gaussian_filter(road.astype(float), sg)
    zs = ndimage.gaussian_filter(np.where(road, res, 0.0), sg) / np.maximum(wsum, 1e-9)
    res = np.where(near, np.minimum(res, zs), res)
    lane_fix = moved | near
    dist, (fj, fi) = ndimage.distance_transform_edt(~lane_fix, return_indices=True)
    lane_z = res[fj, fi]
    foot = ~road & (dist * c <= FOOT_NEAR) & (res - lane_z > FOOT_ABOVE)
    res[foot] = lane_z[foot]
    lowered = int((res < out - 1e-6).sum())
    print(f"lanes: {int(fix.sum()):,} cells over the lanelets' surface, {lowered:,} lowered in all "
          f"(max {float((out - res).max()):.2f} m)")
    return res, lowered


def main():
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("ground_dir")
    ap.add_argument("--lanes", default="", help="a Lanelet2 map: close pits harder on its lanes")
    a = ap.parse_args()
    d = a.ground_dir
    raw = os.path.join(d, "dtm_raw.npy")
    if not os.path.exists(raw):
        os.replace(os.path.join(d, "dtm.npy"), raw)
    dtm = np.load(raw)
    ground = np.load(os.path.join(d, "ground_cells.npy"))
    cell = json.load(open(os.path.join(d, "dtm.json")))["cell"]
    lanes = None
    if a.lanes:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from world_usd import lane_raster

        lanes = lane_raster(a.lanes, json.load(open(os.path.join(d, "dtm.json"))), dtm.shape,
                            LANE_MARGIN)
    est, left = smooth(dtm, ground, cell)  # cells still left keep the raw fill
    out = np.where(ground, dtm, np.minimum(dtm, est + CAP))
    if lanes is not None:
        zmin = np.load(os.path.join(d, "zmin.npy"))
        seen = np.isfinite(zmin)
        zf = np.where(seen, zmin, 1e6).astype(np.float64)
        zm = ndimage.median_filter(zf, ZMIN_MEDIAN)
        low = ndimage.percentile_filter(zf, 25, size=PIT_WINDOW)
        use = lanes & seen & (zm < 1e5) & (zm <= low + CANOPY)
        road = joined(use, zm, zm <= out + RAISED)
        top = ndimage.maximum_filter(np.where(road, zm, -1e6), LANE_WINDOWS[-1])
        island = use & ~road & (zm > top + STEP)
        use &= ~island
        print(f"lanes: {int(island.sum()):,} cells raised above the road and not joined to it")
        out = np.where(use, zm, out)
        print(f"lanes: {int(use.sum()):,} of {int(lanes.sum()):,} cells from the scan's lowest return")
    out = np.maximum(out, ndimage.median_filter(out, PIT_WINDOW) - PIT_DEPTH)
    if lanes is not None:
        meta = json.load(open(os.path.join(d, "dtm.json")))
        ceiling = np.full(out.shape, np.nan)
        jj, ii, e = lane_ele(a.lanes, meta, lanes)
        ceiling[jj, ii] = e
        before = out
        out = close_lane_pits(out, lanes, ceiling)
        print(f"lanes: {int((out > before + 1e-6).sum()):,} pit cells raised")
    out = ndimage.gaussian_filter(ndimage.median_filter(out, MEDIAN), SIGMA_CELLS)
    if a.lanes:
        meta = json.load(open(os.path.join(d, "dtm.json")))
        out, _ = to_lane_ele(out, a.lanes, meta, lane_raster(a.lanes, meta, dtm.shape, ROAD_MARGIN),
                             use)
    np.save(os.path.join(d, "dtm.npy"), out.astype(dtm.dtype))
    for name, sel in (("ground", ground), ("non-ground", ~ground)):
        diff = np.abs(out - dtm)[sel]
        print(f"{name} cells changed: p50 {np.median(diff):.3f} m, p95 "
              f"{np.percentile(diff, 95):.3f} m, max {diff.max():.2f} m")
    print(f"{left.sum():,} cells without ground near")
    return 0


if __name__ == "__main__":
    sys.exit(main())
