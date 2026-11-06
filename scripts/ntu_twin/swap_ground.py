"""Roadmap 059: a built Isaac world's ground moved to a new DTM, without rebuilding the world.

    $ISAAC_ROOT/venv/bin/python scripts/ntu_twin/swap_ground.py WORLD_DIR GROUND_DIR OLD_DTM OSM MARGIN \
        [--parked PARKED.npz] [--dry]

Per tile, the /Tile/Ground vertex heights are recomputed from GROUND_DIR/dtm.npy with
build_meshes.ground_grid's corner averaging (points, normals, extent); the faces depend only on
ground_keep, so faces, UVs, the other meshes, trunks, lights and sky stay. Other/Tree faces the
new DTM would put on the road (world_usd --clear-road, MARGIN m) are dropped; faces the old DTM
(OLD_DTM) cleared and the new one would keep stay missing. Run on a copy (cp -a map/isaac
map/isaac_nN). Used 2026-11-04 for smooth_dtm.py's canopy fix (map/isaac_n11, 12 s).

`--parked` takes roadmap 072's parked_vehicles.py mask (the DTM's grid) and drops, as
carla_meshes.py --parked does, the Other/Tree faces whose centre lies under it below
PARKED_BELOW over the new DTM. The scan's parked cars and scooter rows leave the world, and the
DTM's terrain shows under them. NDT's point-cloud map keeps them, as on CARLA.
"""
import glob
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_usd as wu
from pxr import Usd, UsdGeom, Vt

world, gdir, old_path, osm, margin = sys.argv[1:6]
dry = "--dry" in sys.argv
PARKED_BELOW = 2.6  # m over the DTM (carla_meshes.PARKED_BELOW, parked_vehicles.TALL)
parked_mask = (np.load(sys.argv[sys.argv.index("--parked") + 1])["mask"]
               if "--parked" in sys.argv else None)
meta = json.load(open(os.path.join(gdir, "dtm.json")))
c = meta["cell"]
new = np.load(os.path.join(gdir, "dtm.npy"))
old = np.load(old_path)
lane = wu.lane_raster(osm, meta, new.shape, float(margin))
H, W = new.shape


def corner_z(d, V):
    v = V.astype(np.float64) + np.asarray(wu.MAP_ORIGIN) + wu.SHIFT
    gi = np.rint((v[:, 0] - meta["x0"]) / c).astype(int)
    gj = np.rint((v[:, 1] - meta["y0"]) / c).astype(int)
    z = 0
    for dj in (-1, 0):
        for di in (-1, 0):
            z = z + d[np.clip(gj + dj, 0, H - 1), np.clip(gi + di, 0, W - 1)].astype(np.float64)
    return z / 4 - wu.SHIFT[2] - wu.MAP_ORIGIN[2]


def under_parked(d, P):  # P in the world frame: centres under the mask, below PARKED_BELOW
    if parked_mask is None:
        return np.zeros(len(P), bool)
    v = P.astype(np.float64) + np.asarray(wu.MAP_ORIGIN) + wu.SHIFT
    i = np.clip(((v[:, 0] - meta["x0"]) / c).astype(int), 0, W - 1)
    j = np.clip(((v[:, 1] - meta["y0"]) / c).astype(int), 0, H - 1)
    return parked_mask[j, i] & (v[:, 2] - d[j, i] < PARKED_BELOW)


def on_road(d, P):  # P in the world frame
    v = P.astype(np.float64) + np.asarray(wu.MAP_ORIGIN) + wu.SHIFT
    i = np.clip(((v[:, 0] - meta["x0"]) / c).astype(int), 0, W - 1)
    j = np.clip(((v[:, 1] - meta["y0"]) / c).astype(int), 0, H - 1)
    return lane[j, i] & (v[:, 2] - d[j, i] < wu.CLEAR_HEIGHT)


t0 = time.time()
rep = {}
for tp in sorted(glob.glob(os.path.join(world, "tiles", "tile_*.usdc"))):
    name = os.path.basename(tp)[5:-5]
    st = Usd.Stage.Open(tp)
    gp = st.GetPrimAtPath("/Tile/Ground")
    r = {}
    if gp:
        m = UsdGeom.Mesh(gp)
        V = np.asarray(m.GetPointsAttr().Get()).astype(np.float64)
        zo, zn = corner_z(old, V), corner_z(new, V)
        r["built_vs_old"] = float(np.abs(V[:, 2] - zo).max())
        r["dz_max"] = float(np.abs(zn - V[:, 2]).max())
        r["n_dz_5cm"] = int((np.abs(zn - V[:, 2]) > 0.05).sum())
        if r["dz_max"] > 1e-3 and not dry:
            V[:, 2] = zn
            F = np.asarray(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
            V32 = V.astype(np.float32)
            m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(V32))
            m.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(wu.vertex_normals(V, F).astype(np.float32)))
            m.GetExtentAttr().Set(Vt.Vec3fArray([tuple(V32.min(0).tolist()), tuple(V32.max(0).tolist())]))
    for part in ("Other", "Tree"):
        p = st.GetPrimAtPath(f"/Tile/{part}")
        if not p:
            continue
        m = UsdGeom.Mesh(p)
        V = np.asarray(m.GetPointsAttr().Get())
        F = np.asarray(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        if not len(F):
            continue
        cen = V[F].mean(axis=1)
        a, b = on_road(new, cen), on_road(old, cen)
        drop = a & ~b
        r[f"{part}_drop"] = int(drop.sum())
        pk = under_parked(new, cen) & ~drop
        r[f"{part}_parked"] = int(pk.sum())
        drop |= pk
        r[f"{part}_would_restore"] = 0  # faces old cleared but new keeps: not in the copy, see report
        if drop.any() and not dry:
            F = F[~drop]
            m.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(F.astype(np.int32).ravel()))
            m.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
    changed = any(r.get(k, 0) for k in ("Other_drop", "Tree_drop", "Other_parked", "Tree_parked"))
    if not dry and (r.get("dz_max", 0) > 1e-3 or changed):
        st.GetRootLayer().Save()
    rep[name] = r
    if r.get("dz_max", 0) > 0.05 or changed:
        print(name, {k: round(v, 3) if isinstance(v, float) else v for k, v in r.items()
                     if "restore" not in k}, flush=True)
print("built_vs_old max over tiles:", max(v.get("built_vs_old", 0) for v in rep.values()))
print("tiles with dz>1mm:", sum(v.get("dz_max", 0) > 1e-3 for v in rep.values()),
      "dz>5cm:", sum(v.get("dz_max", 0) > 0.05 for v in rep.values()), "of", len(rep),
      f"{time.time() - t0:.0f}s")
print("faces under the parked mask:", sum(v.get("Other_parked", 0) + v.get("Tree_parked", 0)
                                          for v in rep.values()), "(dry)" if dry else "(dropped)")
json.dump(rep, open(os.path.join(world, "swap_report%s.json" % ("_dry" if dry else "")), "w"),
          indent=1)
