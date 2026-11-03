#!/usr/bin/env python3
"""A CARLA import package of the NTU campus (roadmap 072 C7): FBX, OpenDRIVE, textures and the
package JSON that CARLA's `make import` reads.

    carla_package.py MESHES_DIR ORTHO_DIR XODR OUT_DIR [--name NTU]

Run with a Python that has Blender's `bpy` (pip `bpy` 4.2, Python 3.11). MESHES_DIR is
`carla_meshes.py`'s output, ORTHO_DIR the ground textures (059's `ortho_refill.py` output,
rgb_<ix>_<iy>.jpg). Writes OUT_DIR/<name>.json and OUT_DIR/<name>/<name>.fbx, <name>.xodr and
the textures; copy OUT_DIR into a CARLA source build's `Import/` and run `make import`.

Objects, one per tile and class, named so CARLA's import tags them (SSTags.h): Road_Road_<t>
(Road), Road_Grass_<t> (Terrain), Prop_<t> and Tree_<t> (Terrain: the import has no other
folder). The ground's two take the tile's orthophoto (material M_Ortho_<t>); the props and
trees share M_ScanColour, a stand-in that the editor script `ue_materials.py` replaces after
the import with a master material reading the vertex colour (RGB: the scan's de-lit colour,
linear; A: the trees' cut-outs). `use_carla_materials` is false, so CARLA keeps ours.

Frame: as 064's Shalun. The vertices are in metres, the map frame less MAP_ORIGIN, z up, as the
OpenDRIVE; Blender's default FBX export, which UE imports in centimetres with y mirrored,
CARLA's convention.

Shading: every mesh is smooth-shaded (Blender's default is flat, and the FBX's smoothing
groups then make UE light each triangle as its own plane: the scan's coarse parked cars,
scooters and hedges came out as grey slabs, 072). Buildings keep edges sharper than
SHARP_ANGLE, so their corners stay corners.
"""
import argparse
import json
import os
import re
import shutil
import sys

import numpy as np


def material_ortho(bpy, name, image_path):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes["Principled BSDF"]
    t = m.node_tree.nodes.new("ShaderNodeTexImage")
    t.image = bpy.data.images.load(image_path)
    m.node_tree.links.new(t.outputs["Color"], bsdf.inputs["Base Color"])
    bsdf.inputs["Roughness"].default_value = 0.9
    return m


SHARP_ANGLE = 60.0  # deg, buildings only


def add_object(bpy, name, V, F, material, uv=None, colours=None, sharp_angle=None):
    me = bpy.data.meshes.new(name)
    me.vertices.add(len(V))
    me.vertices.foreach_set("co", V.astype(np.float32).ravel())
    me.loops.add(F.size)
    me.loops.foreach_set("vertex_index", F.astype(np.int32).ravel())
    me.polygons.add(len(F))
    me.polygons.foreach_set("loop_start", np.arange(0, F.size, 3, dtype=np.int32))
    me.polygons.foreach_set("loop_total", np.full(len(F), 3, np.int32))
    if uv is not None:
        layer = me.uv_layers.new(name="UVMap")
        layer.data.foreach_set("uv", uv[F.ravel()].astype(np.float32).ravel())
    if colours is not None:
        ca = me.color_attributes.new(name="Col", type="FLOAT_COLOR", domain="POINT")
        ca.data.foreach_set("color", colours.astype(np.float32).ravel())
    me.materials.append(material)
    me.validate()
    me.shade_smooth()
    if sharp_angle is not None:
        me.set_sharp_from_angle(angle=np.radians(sharp_angle))
    me.update()
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    return len(F)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("meshes")
    ap.add_argument("ortho")
    ap.add_argument("xodr")
    ap.add_argument("out")
    ap.add_argument("--name", default="NTU")
    a = ap.parse_args(argv)
    import bpy

    pkg = os.path.join(a.out, a.name)
    os.makedirs(pkg, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scan = bpy.data.materials.new("M_ScanColour")
    scan.use_nodes = True
    totals = {}
    tiles = sorted(f for f in os.listdir(a.meshes) if re.match(r"tile_-?\d+_-?\d+\.npz$", f))
    for f in tiles:
        t = f[5:-4]
        z = np.load(os.path.join(a.meshes, f))
        ortho = None
        for cls, obj in (("road", f"Road_Road_{t}"), ("terrain", f"Road_Grass_{t}")):
            if f"{cls}_V" not in z.files:
                continue
            if ortho is None:
                src = os.path.join(a.ortho, f"rgb_{t}.jpg")
                shutil.copyfile(src, os.path.join(pkg, f"T_Ortho_{t}.jpg"))
                ortho = material_ortho(bpy, f"M_Ortho_{t}", os.path.join(pkg, f"T_Ortho_{t}.jpg"))
            totals[cls] = totals.get(cls, 0) + add_object(bpy, obj, z[f"{cls}_V"], z[f"{cls}_F"], ortho,
                                                          uv=z[f"{cls}_UV"])
        for cls, obj in (("prop", f"Prop_{t}"), ("tree", f"Tree_{t}"), ("building", f"Building_{t}")):
            if f"{cls}_V" not in z.files:
                continue
            totals[cls] = totals.get(cls, 0) + add_object(bpy, obj, z[f"{cls}_V"], z[f"{cls}_F"], scan,
                                                          colours=z[f"{cls}_C"],
                                                          sharp_angle=SHARP_ANGLE if cls == "building" else None)
    fbx = os.path.join(pkg, f"{a.name}.fbx")
    bpy.ops.export_scene.fbx(filepath=fbx, path_mode="RELATIVE", embed_textures=False,
                             mesh_smooth_type="FACE", use_mesh_modifiers=False, colors_type="LINEAR")
    shutil.copyfile(a.xodr, os.path.join(pkg, f"{a.name}.xodr"))
    with open(os.path.join(a.out, f"{a.name}.json"), "w") as fh:
        json.dump({"maps": [{"name": a.name, "source": f"./{a.name}/{a.name}.fbx",
                             "use_carla_materials": False, "xodr": f"./{a.name}/{a.name}.xodr"}],
                   "props": []}, fh, indent=2)
    print(f"{len(tiles)} tiles; triangles {totals}; {fbx} {os.path.getsize(fbx) / 1e6:.0f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
