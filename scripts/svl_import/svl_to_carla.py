#!/usr/bin/env python3
"""A CARLA import package from an SVL map: OpenDRIVE and FBX in one frame.

Run with a Python that has Blender's `bpy` (pip `bpy`, Python 3.11):

    svl_to_carla.py DUMP_DIR XODR OUT_DIR [--name Shalun]

Writes OUT_DIR/<name>/: `<name>.xodr`, `<name>.fbx` (with its textures beside it), and
OUT_DIR/<name>.json, the package description CARLA's `make import` reads (copy OUT_DIR into
CARLA's `Import/` and run `make import` in a CARLA source build).

Frame. SVL wrote its OpenDRIVE turned 90 degrees against its own map frame: map (east,
north) = (y, -x) of the file's. The roads are turned back (each geometry's x, y and heading;
everything else in OpenDRIVE is relative to the reference line), so the OpenDRIVE, the FBX
and the Lanelet2 map share the map's local ENU frame (svl_to_usd.py). CARLA's world is that
frame with y negated, as for any OpenDRIVE map. The geoReference keeps SVL's: a transverse
Mercator at the map origin, which is what CARLA's GNSS reads (lat_0, lon_0).

The FBX is built in Blender in that frame (metres, z up) and exported with Blender's
defaults, which UE imports in centimetres with its y mirrored: CARLA's convention. Meshes
are merged per class and named as CARLA's import tags them (RoadRunner's names): Road_Road,
Road_Marking, Road_Sidewalk, Road_Curb, Road_Grass; the rest are Building_* and Prop_*.
Traffic lights and stop signs are left out: CARLA spawns its own from the OpenDRIVE signals
(SVL wrote CARLA's types, 1000001 and 206). SVL's signal controllers are rebuilt
(`fix_controllers`): they referred to signal ids that do not exist, and CARLA crashes on them.
"""
import argparse
import json
import math
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from svl_to_usd import unity_to_map  # noqa: E402

CLASSES = [  # (material pattern, CARLA object name), the first match wins
    (r"TrafficSignal|SignTrafficStop|SignalLight", None),  # CARLA spawns these
    (r"Asphalt", "Road_Road"),
    (r"White|Yellow|Arrow|CrossWalk|Crosswalk", "Road_Marking"),
    (r"SideWalk|Sidewalk", "Road_Sidewalk"),
    (r"Curb", "Road_Curb"),
    (r"Grass|Background", "Road_Grass"),
    (r"Building", "Building_Shalun"),
]


def class_of(materials):
    for name in materials:
        for pat, cls in CLASSES:
            if re.search(pat, name):
                return cls if cls else "skip"
    return "Prop_Shalun"


def fix_controllers(root):
    """SVL's controllers name signal ids that do not exist ("174" for "signal_5"), and
    CARLA's map builder dereferences them unchecked (a segfault in
    MapBuilder::SolveControllerAndJuntionReferences). Rebuilt: each junction gets one
    controller per traffic light (type 1000001) at its ends of the roads that meet it, so
    CARLA groups the junction's lights; the world server sets their states anyway."""
    for c in root.findall("controller"):
        root.remove(c)
    for j in root.iter("junction"):
        for c in j.findall("controller"):
            j.remove(c)
    junctions = {j.get("id"): j for j in root.iter("junction")}
    cid = 1000
    added = 0
    for rd in root.iter("road"):
        link = rd.find("link")
        ends = {}
        for e in (link if link is not None else []):
            if e.get("elementType") == "junction":
                ends[e.tag] = e.get("elementId")
        length = float(rd.get("length"))
        for sig in rd.iter("signal"):
            if sig.get("type") != "1000001":
                continue
            s_ = float(sig.get("s"))
            jid = ends.get("successor") if s_ > length / 2 else ends.get("predecessor")
            if jid not in junctions:
                continue
            ctl = ET.SubElement(root, "controller", {"id": str(cid), "name": f"ctrl_{sig.get('id')}"})
            ET.SubElement(ctl, "control", {"signalId": sig.get("id"), "type": "0"})
            ET.SubElement(junctions[jid], "controller", {"id": str(cid), "type": "0", "sequence": "0"})
            cid += 1
            added += 1
    return added


def rotate_xodr(src, dst):
    """map (x, y) = (y, -x) of SVL's file; headings less 90 degrees."""
    tree = ET.parse(src)
    root = tree.getroot()
    n = 0
    for g in root.iter("geometry"):
        x, y, hdg = float(g.get("x")), float(g.get("y")), float(g.get("hdg"))
        g.set("x", f"{y:.17g}")
        g.set("y", f"{-x:.17g}")
        h = hdg - math.pi / 2
        g.set("hdg", f"{math.atan2(math.sin(h), math.cos(h)):.17g}")
        n += 1
    fix_controllers(root)
    header = root.find("header")
    if header is not None:
        header.set("vendor", "LGSVL; turned into the map frame by scripts/svl_import/svl_to_carla.py")
    tree.write(dst, encoding="utf-8", xml_declaration=True)
    return n


def build_fbx(dump, out_fbx, tex_dir):
    import bpy

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = json.load(open(os.path.join(dump, "scene.json")))
    mats = {m["name"]: m for m in json.load(open(os.path.join(dump, "materials.json")))}
    T = unity_to_map(scene.get("map_origin_yaw_deg", -90.0))
    os.makedirs(tex_dir, exist_ok=True)

    bmats = {}

    def material(name):
        if name in bmats:
            return bmats[name]
        m = mats.get(name, {"base_color": [0.8, 0.8, 0.8, 1]})
        bm = bpy.data.materials.new(re.sub(r"[^A-Za-z0-9_]", "_", m.get("unity_name", name)))
        bm.use_nodes = True
        bsdf = bm.node_tree.nodes["Principled BSDF"]
        c = m.get("base_color", [1, 1, 1, 1])
        bsdf.inputs["Base Color"].default_value = (c[0], c[1], c[2], 1)
        if m.get("base_map"):
            src = os.path.join(dump, "textures", m["base_map"])
            shutil.copyfile(src, os.path.join(tex_dir, m["base_map"]))
            img = bpy.data.images.load(os.path.join(tex_dir, m["base_map"]))
            t = bm.node_tree.nodes.new("ShaderNodeTexImage")
            t.image = img
            bm.node_tree.links.new(t.outputs["Color"], bsdf.inputs["Base Color"])
        bmats[name] = bm
        return bm

    groups = {}
    for inst in scene["instances"]:
        cls = class_of(inst["materials"])
        if cls == "skip":
            continue
        z = np.load(os.path.join(dump, "meshes", f"{inst['mesh']}.npz"))
        W = T @ np.array(inst["world"])
        V = (W[:3, :3] @ z["positions"].astype(np.float64).T).T + W[:3, 3]
        subs = sorted((k for k in z.files if k.startswith("sub")), key=lambda s: int(s[3:]))
        uv = z["uv"] if "uv" in z.files else None
        g = groups.setdefault(cls, {"V": [], "F": [], "M": [], "UV": [], "n": 0})
        for i, s in enumerate(subs):
            F = z[s][:, ::-1]  # the mirror reverses winding
            if not len(F):
                continue
            mname = inst["materials"][min(i, len(inst["materials"]) - 1)] if inst["materials"] else None
            g["F"].append(F + g["n"])
            g["M"].append(np.full(len(F), -1 if mname is None else id(material(mname)), np.int64))
            g.setdefault("mat_ids", {})
            if mname is not None:
                g["mat_ids"][id(material(mname))] = material(mname)
        g["V"].append(V)
        g["UV"].append(uv if uv is not None else np.zeros((len(V), 2), np.float32))
        g["n"] += len(V)

    for cls, g in groups.items():
        V = np.concatenate(g["V"])
        F = np.concatenate(g["F"])
        Mi = np.concatenate(g["M"])
        UV = np.concatenate(g["UV"])
        me = bpy.data.meshes.new(cls)
        me.from_pydata(V.tolist(), [], F.tolist())
        slots = list(g.get("mat_ids", {}).items())
        index = {k: j for j, (k, _) in enumerate(slots)}
        for _, bm in slots:
            me.materials.append(bm)
        me.polygons.foreach_set("material_index", [index.get(int(k), 0) for k in Mi])
        uvl = me.uv_layers.new(name="UVMap")
        loop_uv = UV[np.array([v for p in me.polygons for v in p.vertices])]
        uvl.data.foreach_set("uv", loop_uv.astype(np.float32).ravel())
        me.validate()
        me.update()
        ob = bpy.data.objects.new(cls, me)
        bpy.context.scene.collection.objects.link(ob)
        print(f"{cls}: {len(V):,} vertices, {len(F):,} triangles, {len(slots)} materials")
    bpy.ops.export_scene.fbx(filepath=out_fbx, path_mode="RELATIVE", embed_textures=False,
                             mesh_smooth_type="FACE", use_mesh_modifiers=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump")
    ap.add_argument("xodr")
    ap.add_argument("out")
    ap.add_argument("--name", default="Shalun")
    a = ap.parse_args()
    d = os.path.join(a.out, a.name)
    os.makedirs(d, exist_ok=True)
    n = rotate_xodr(a.xodr, os.path.join(d, f"{a.name}.xodr"))
    print(f"xodr: {n} geometries turned into the map frame")
    build_fbx(a.dump, os.path.join(d, f"{a.name}.fbx"), d)
    with open(os.path.join(a.out, f"{a.name}.json"), "w") as f:
        json.dump({"maps": [{"name": a.name, "source": f"./{a.name}/{a.name}.fbx",
                             "use_carla_materials": True, "xodr": f"./{a.name}/{a.name}.xodr"}],
                   "props": []}, f, indent=2)


if __name__ == "__main__":
    main()
