#!/usr/bin/env python3
"""Dump an SVL (LGSVL) environment's Unity asset bundles into plain files.

    unity_dump.py BUNDLE_DIR OUT_DIR

BUNDLE_DIR is an unzipped SVL map asset: `manifest.json`,
`<guid>_environment_main_linux` and `<guid>_environment_textures` (Unity 2020.3 bundles,
read with UnityPy). OUT_DIR gets:

  meshes/<id>.npz   per Unity mesh: positions, normals, uv (Unity's local frame) and one
                    index array per submesh (`sub0`, `sub1`, ...)
  textures/<name>.png
  materials.json    name, shader, base colour, the base colour, normal and mask maps, the
                    base map's tiling and offset
  scene.json        SVL's MapOrigin (UTM origin, zone, its yaw about y); every MeshRenderer: its GameObject's path, mesh id, materials,
                    whether it has a MeshCollider, and its world matrix (Unity: left-handed,
                    y up, metres); and the lights

Converting the frame is left to the builders (svl_to_usd.py): Unity (x, y, z) is map
(east, north, up) = (x, z, y), which also flips triangle winding.
"""
import argparse
import glob
import json
import os

import numpy as np


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def local_matrix(t):
    m = np.eye(4)
    s = np.diag([t.m_LocalScale.x, t.m_LocalScale.y, t.m_LocalScale.z])
    m[:3, :3] = quat_matrix(t.m_LocalRotation) @ s
    m[:3, 3] = [t.m_LocalPosition.x, t.m_LocalPosition.y, t.m_LocalPosition.z]
    return m


# Unity's built-in meshes ("unity default resources", not in the bundle), by path id
BUILTIN = {10202: "Cube", 10206: "Cylinder", 10207: "Sphere", 10208: "Capsule",
           10209: "Plane", 10210: "Quad"}


def builtin_mesh(name):
    """A stand-in for a Unity built-in mesh, in Unity's conventions (left-handed, y up,
    clockwise front faces seen from outside)."""
    if name == "Quad":
        P = np.array([[-.5, -.5, 0], [.5, -.5, 0], [.5, .5, 0], [-.5, .5, 0]], np.float32)
        return P, np.array([[0, 2, 1], [0, 3, 2]]), np.array([[0, 0], [1, 0], [1, 1], [0, 1]], np.float32)
    if name == "Plane":  # 10 x 10 m in x-z, facing +y
        P = np.array([[-5, 0, -5], [5, 0, -5], [5, 0, 5], [-5, 0, 5]], np.float32)
        return P, np.array([[0, 2, 1], [0, 3, 2]]), np.array([[1, 1], [0, 1], [0, 0], [1, 0]], np.float32)
    if name in ("Cylinder", "Capsule", "Sphere"):
        n = 24
        th = np.linspace(0, 2 * np.pi, n, endpoint=False)
        r, h = (0.5, 1.0) if name != "Sphere" else (0.5, 0.5)
        ring = np.stack([r * np.cos(th), np.zeros(n), r * np.sin(th)], 1)
        P = np.concatenate([ring + [0, -h, 0], ring + [0, h, 0], [[0, -h, 0], [0, h, 0]]]).astype(np.float32)
        F = []
        for i in range(n):
            j = (i + 1) % n
            F += [[i, n + i, j], [j, n + i, n + j], [2 * n, i, j], [2 * n + 1, n + j, n + i]]
        return P, np.array(F), None
    # Cube
    P, F, UV = [], [], []
    for axis in range(3):
        for sgn in (-1, 1):
            u, v = [(axis + 1) % 3, (axis + 2) % 3]
            base = len(P)
            for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                q = [0, 0, 0]
                q[axis], q[u], q[v] = sgn * .5, a * .5, b * .5
                P.append(q)
                UV.append([(a + 1) / 2, (b + 1) / 2])
            F += ([[base, base + 1, base + 2], [base, base + 2, base + 3]] if sgn < 0 else
                  [[base, base + 2, base + 1], [base, base + 3, base + 2]])
    return np.array(P, np.float32), np.array(F), np.array(UV, np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bundle_dir")
    ap.add_argument("out")
    a = ap.parse_args()
    import UnityPy
    from UnityPy.helpers.MeshHelper import MeshHandler

    files = sorted(glob.glob(os.path.join(a.bundle_dir, "*_environment_main_linux")) +
                   glob.glob(os.path.join(a.bundle_dir, "*_environment_textures")))
    env = UnityPy.load(*files)
    for d in ("meshes", "textures"):
        os.makedirs(os.path.join(a.out, d), exist_ok=True)

    by_type = {}
    for o in env.objects:
        by_type.setdefault(o.type.name, []).append(o)

    # transforms: world matrices through the parent chain
    tr = {}
    for o in by_type.get("Transform", []):
        tr[o.path_id] = o.read()
    world = {}

    def world_of(pid):
        if pid in world:
            return world[pid]
        t = tr[pid]
        m = local_matrix(t)
        father = t.m_Father.path_id
        if father and father in tr:
            m = world_of(father) @ m
        world[pid] = m
        return m

    go_path = {}

    def path_of(t_pid):
        if t_pid in go_path:
            return go_path[t_pid]
        t = tr[t_pid]
        name = t.m_GameObject.read().m_Name
        father = t.m_Father.path_id
        p = (path_of(father) + "/" if father and father in tr else "") + name
        go_path[t_pid] = p
        return p

    # GameObject -> its components
    comps = {}
    for o in by_type.get("GameObject", []):
        g = o.read()
        comps[o.path_id] = [c.component if hasattr(c, "component") else c[1] for c in g.m_Component]

    textures = {}

    def texture(pptr):
        if pptr is None or pptr.path_id == 0:
            return None
        try:
            obj = pptr.deref() if hasattr(pptr, "deref") else None
            tex = pptr.read()
        except Exception:
            return None
        name = tex.m_Name or f"tex_{pptr.path_id}"
        if name not in textures:
            try:
                tex.image.save(os.path.join(a.out, "textures", name + ".png"))
                textures[name] = name + ".png"
            except Exception as e:  # unsupported format
                print("texture", name, "failed:", e)
                textures[name] = None
        return textures[name]

    materials = {}

    def material(pptr):
        key = pptr.path_id
        if key in materials:
            return materials[key]["name"]
        try:
            mat = pptr.read()
        except FileNotFoundError:  # Unity's Default-Material
            materials[key] = {"name": f"Default_{key}", "unity_name": "Default-Material",
                              "shader": None, "base_color": [0.8, 0.8, 0.8, 1]}
            return materials[key]["name"]
        props = mat.m_SavedProperties
        tex = {k: v for k, v in props.m_TexEnvs}
        cols = {k: v for k, v in props.m_Colors}
        floats = {k: v for k, v in props.m_Floats}
        entry = {"name": f"{mat.m_Name}_{key}", "unity_name": mat.m_Name}
        try:
            entry["shader"] = mat.m_Shader.read().m_ParsedForm.m_Name
        except Exception:
            entry["shader"] = None
        c = cols.get("_BaseColor") or cols.get("_Color")
        entry["base_color"] = [c.r, c.g, c.b, c.a] if c else [1, 1, 1, 1]
        for slot, names in (("base_map", ("_BaseColorMap", "_MainTex", "_BaseMap")),
                            ("normal_map", ("_NormalMap", "_BumpMap")),
                            ("mask_map", ("_MaskMap",)),
                            ("emissive_map", ("_EmissiveColorMap",))):
            for n in names:
                if n in tex:
                    f = texture(tex[n].m_Texture)
                    if f:
                        entry[slot] = f
                        if slot == "base_map":
                            s, o = tex[n].m_Scale, tex[n].m_Offset
                            entry["tiling"] = [s.x, s.y]
                            entry["offset"] = [o.x, o.y]
                        break
        # the shader-graph materials keep their textures under generated names
        if "base_map" not in entry:
            for n, t in tex.items():
                f = texture(t.m_Texture)
                if f and "Normal" not in n and "Mask" not in n:
                    entry["base_map"] = f
                    s, o = t.m_Scale, t.m_Offset
                    entry["tiling"] = [s.x, s.y]
                    entry["offset"] = [o.x, o.y]
                    entry["base_map_slot"] = n
                    break
        for k in ("_Metallic", "_Smoothness"):
            if k in floats:
                entry[k.strip("_").lower()] = floats[k]
        materials[key] = entry
        return entry["name"]

    meshes = set()

    def dump_mesh(pptr):
        key = pptr.path_id
        if key in meshes:
            return key
        try:
            m = pptr.read()
        except FileNotFoundError:
            name = BUILTIN.get(key)
            if name is None:
                print("mesh", key, "is in another file; skipped")
                return None
            P, F, UV = builtin_mesh(name)
            out = {"positions": P, "sub0": F.astype(np.int32)}
            if UV is not None:
                out["uv"] = UV
            np.savez_compressed(os.path.join(a.out, "meshes", f"{key}.npz"), **out)
            meshes.add(key)
            return key
        h = MeshHandler(m)
        h.process()
        n = h.m_VertexCount
        # channels come with 2, 3 or 4 components per vertex, whatever the mesh was saved with
        out = {"positions": np.asarray(h.m_Vertices, np.float32).reshape(n, -1)[:, :3]}
        if h.m_Normals:
            out["normals"] = np.asarray(h.m_Normals, np.float32).reshape(n, -1)[:, :3]
        if h.m_UV0:
            out["uv"] = np.asarray(h.m_UV0, np.float32).reshape(n, -1)[:, :2]
        for i, (tris, sm) in enumerate(zip(h.get_triangles(), m.m_SubMeshes)):
            arr = np.asarray(tris, np.int64).reshape(-1, 3)
            base = getattr(sm, "baseVertex", 0) or 0
            out[f"sub{i}"] = (arr + base).astype(np.int32)
        np.savez_compressed(os.path.join(a.out, "meshes", f"{key}.npz"), **out)
        meshes.add(key)
        return key

    instances = []
    for go_id, cs in comps.items():
        objs = {}
        for c in cs:
            try:
                objs[c.type.name] = c
            except Exception:
                pass
        if "MeshFilter" not in objs or "MeshRenderer" not in objs:
            continue
        mf = objs["MeshFilter"].read()
        mr = objs["MeshRenderer"].read()
        if mf.m_Mesh.path_id == 0:
            continue
        t_id = objs["Transform"].path_id
        mesh_id = dump_mesh(mf.m_Mesh)
        if mesh_id is None:
            continue
        inst = {
            "path": path_of(t_id),
            "mesh": mesh_id,
            "builtin": BUILTIN.get(mesh_id),
            "materials": [material(p) for p in mr.m_Materials if p.path_id],
            "collider": "MeshCollider" in objs,
            "enabled": bool(getattr(mr, "m_Enabled", 1)),
            "world": world_of(t_id).tolist(),
        }
        instances.append(inst)

    lights = []
    for o in by_type.get("Light", []):
        l = o.read()
        t_id = None
        for c in comps.get(l.m_GameObject.path_id, []):
            if c.type.name == "Transform":
                t_id = c.path_id
        lights.append({"type": int(l.m_Type), "intensity": float(l.m_Intensity),
                       "color": [l.m_Color.r, l.m_Color.g, l.m_Color.b],
                       "range": float(getattr(l, "m_Range", 0)),
                       "path": path_of(t_id) if t_id else None,
                       "world": world_of(t_id).tolist() if t_id else None})

    # SVL's MapOrigin: the UTM origin and the GameObject's turn about y, which together
    # place the scene on the earth
    origin = {}
    for o in by_type.get("MonoBehaviour", []):
        try:
            tree = o.read_typetree()
        except Exception:
            continue
        if "OriginEasting" not in tree:
            continue
        origin = {k: tree[k] for k in ("OriginEasting", "OriginNorthing", "UTMZoneId", "AltitudeOffset")
                  if k in tree}
        for c in comps.get(tree["m_GameObject"]["m_PathID"], []):
            if c.type.name == "Transform":
                q = tr[c.path_id].m_LocalRotation
                origin["yaw_deg"] = float(np.degrees(2 * np.arctan2(q.y, q.w)))
                origin["position"] = world_of(c.path_id)[:3, 3].tolist()
    manifest = json.load(open(os.path.join(a.bundle_dir, "manifest.json")))
    with open(os.path.join(a.out, "scene.json"), "w") as f:
        json.dump({"manifest": manifest, "map_origin": origin, "map_origin_yaw_deg": origin.get("yaw_deg"),
                   "instances": instances, "lights": lights}, f, indent=1)
    with open(os.path.join(a.out, "materials.json"), "w") as f:
        json.dump(list(materials.values()), f, indent=1)
    print(f"{len(instances)} instances, {len(meshes)} meshes, {len(materials)} materials, "
          f"{sum(1 for v in textures.values() if v)} textures, {len(lights)} lights")


if __name__ == "__main__":
    main()
