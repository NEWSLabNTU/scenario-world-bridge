"""Roadmap 091 W9: walls and cones in Isaac. Authored physics materials (restitution, static and
dynamic friction) against what PhysX does with them, by drop, slide, tilt and push tests.

    python contact_check.py --out DIR --hz 240 [--only test,...]

No car: rigid bodies on a static floor (a box collider whose top is z = 0), each test in its own
lane. Every material is authored on a UsdShade material bound with the physics purpose, as an
authored stage would carry it.

- `drop_<e>`:    a 0.1 m cube (0.5 kg) dropped from 0.5 m; the cube's restitution e, the floor's
                 e too (combine `average`); the first rebound's apex over the drop height, and
                 the restitution it implies (sqrt of that ratio).
- `slide_<mu>`:  the cube sent at 2 m/s along the floor, cube and floor at dynamic friction mu;
                 its deceleration over g is the friction PhysX applied.
- `combine_<mode>`: the same slide with the cube at 0.4 and the floor at 0.8, the pair combined
                 by `mode` (average, min, max, multiply), set on both materials, and with
                 no mode authored (`combine_unset`).
- `tilt_<mu>`:   the cube at rest, gravity tilted 0.1 deg every 0.05 s until it slides; tan of
                 that angle is the static friction PhysX applied (the design note's tilt test).

The cone's tip-or-carry is measured with the car: smallcar.py's `cone` with `--floor-mu` and
`--cone-mu`.

Run in Isaac's environment (aib's scripts/isaac_env.sh); it needs nothing of aib's.
"""
import argparse
import json
import math
import os
import time
from pathlib import Path

G = 9.81
DROP_E = (0.0, 0.3, 0.6, 0.9)
SLIDE_MU = (0.2, 0.5, 0.8)
COMBINE = ("average", "min", "max", "multiply", None)  # None: no mode authored
TILT_MU = (0.3, 0.6, 0.9)
TIP_MU = ()  # the cone's tip-or-carry is measured with the car (smallcar.py --cone-mu)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--hz", type=float, default=240.0)
    ap.add_argument("--only", default="")
    args = ap.parse_args()
    out = Path(args.out) / f"{args.hz:.0f}hz"
    out.mkdir(parents=True, exist_ok=True)
    os.environ.pop("DISPLAY", None)
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "extra_args": ["--/crashreporter/url="]})
    import omni.physx
    import omni.usd
    from isaacsim.core.api import World
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

    dt = 1.0 / args.hz
    world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt)
    stage = omni.usd.get_context().get_stage()
    physx = omni.physx.get_physx_interface()
    only = [t for t in args.only.split(",") if t]

    def safe(path):
        return path.replace(".", "p")

    def material(path, static, dynamic, restitution, combine=None):
        m = UsdShade.Material.Define(stage, safe(path))
        api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(restitution)
        px = PhysxSchema.PhysxMaterialAPI.Apply(m.GetPrim())
        if combine:
            px.CreateFrictionCombineModeAttr(combine)
            px.CreateRestitutionCombineModeAttr(combine)
        return m

    def bind(prim, mat):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    def box(path, center, size, mass=None, kinematic=False):
        c = UsdGeom.Cube.Define(stage, safe(path))
        c.CreateSizeAttr(1.0)
        xf = UsdGeom.XformCommonAPI(c.GetPrim())
        xf.SetTranslate(Gf.Vec3d(*center))
        xf.SetScale(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        if mass is not None or kinematic:
            rb = UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
            # never asleep: a sleeping body does not wake when the gravity turns (tilt)
            PhysxSchema.PhysxRigidBodyAPI.Apply(c.GetPrim()).CreateSleepThresholdAttr(0.0)
            if kinematic:
                rb.CreateKinematicEnabledAttr(True)
            else:
                UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(mass)
        return c.GetPrim()

    def cylinder(path, center, r, h, mass):
        c = UsdGeom.Cylinder.Define(stage, safe(path))
        c.CreateRadiusAttr(r)
        c.CreateHeightAttr(h)
        c.CreateAxisAttr("Z")
        UsdGeom.XformCommonAPI(c.GetPrim()).SetTranslate(Gf.Vec3d(*center))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
        UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(mass)
        return c.GetPrim()

    # Each test: its own floor tile (its own material) and its bodies, 3 m apart along y.
    tests = []
    y = [0.0]

    def lane():
        y[0] += 3.0
        return y[0]

    def floor(name, ly, mat):
        f = box(f"/World/floor/{name}", (5.0, ly, -0.05), (14.0, 2.8, 0.1))
        bind(f, mat)

    for e in DROP_E:
        n = f"drop_{e:g}"
        ly = lane()
        mat = material(f"/World/mat/{n}", 0.5, 0.5, e, "average")
        floor(n, ly, mat)
        cube = box(f"/World/body/{n}", (0.0, ly, 0.5 + 0.05), (0.1, 0.1, 0.1), mass=0.5)
        bind(cube, mat)
        tests.append((n, "drop", {"e": e, "body": cube.GetPath().pathString, "h0": 0.5}))
    for mu in SLIDE_MU:
        n = f"slide_{mu:g}"
        ly = lane()
        mat = material(f"/World/mat/{n}", mu, mu, 0.0, "average")
        floor(n, ly, mat)
        cube = box(f"/World/body/{n}", (0.0, ly, 0.05), (0.1, 0.1, 0.1), mass=0.5)
        bind(cube, mat)
        tests.append((n, "slide", {"mu": mu, "body": cube.GetPath().pathString}))
    for mode in COMBINE:
        n = f"combine_{mode or 'unset'}"
        ly = lane()
        fm = material(f"/World/mat/{n}_floor", 0.8, 0.8, 0.0, mode)
        cm = material(f"/World/mat/{n}_cube", 0.4, 0.4, 0.0, mode)
        floor(n, ly, fm)
        cube = box(f"/World/body/{n}", (0.0, ly, 0.05), (0.1, 0.1, 0.1), mass=0.5)
        bind(cube, cm)
        expect = {"average": 0.6, "min": 0.4, "max": 0.8, "multiply": 0.32, None: 0.6}[mode]
        tests.append((n, "slide", {"mu": expect, "body": cube.GetPath().pathString}))
    for mu in TILT_MU:
        n = f"tilt_{mu:g}"
        ly = lane()
        mat = material(f"/World/mat/{n}", mu, mu, 0.0, "average")
        floor(n, ly, mat)
        cube = box(f"/World/body/{n}", (0.0, ly, 0.05), (0.1, 0.1, 0.1), mass=0.5)
        bind(cube, mat)
        tests.append((n, "tilt", {"mu": mu, "body": cube.GetPath().pathString}))
    for mu in TIP_MU:
        n = f"tip_{mu:g}"
        ly = lane()
        mat = material(f"/World/mat/{n}", mu, mu, 0.0, "average")
        floor(n, ly, mat)
        cyl = cylinder(f"/World/body/{n}", (1.0, ly, 0.075), 0.05, 0.15, 0.1)
        bind(cyl, mat)
        plate = box(f"/World/plate/{n}", (0.5, ly, 0.13), (0.02, 0.3, 0.2), kinematic=True)
        bind(plate, mat)
        tests.append((n, "tip", {"mu": mu, "body": cyl.GetPath().pathString,
                                 "plate": plate}))
    scene = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
    gravity = UsdPhysics.Scene(scene).CreateGravityDirectionAttr()
    world.reset()
    world.play()

    def pos(path):
        return tuple(float(c) for c in physx.get_rigidbody_transformation(path)["position"])

    def rot(path):
        x, y_, z, w = (float(c) for c in physx.get_rigidbody_transformation(path)["rotation"])
        return w, x, y_, z

    def step(n=1):
        for _ in range(n):
            world.step(render=False)

    def set_velocity(path, v):
        prim = stage.GetPrimAtPath(path)
        UsdPhysics.RigidBodyAPI(prim).GetVelocityAttr().Set(Gf.Vec3f(*v))

    results = {"hz": args.hz, "tests": {}}
    wall0 = time.monotonic()
    # every body settles at once (the drops are already falling: run the drops first)
    drops = [t for t in tests if t[1] == "drop" and (not only or t[0] in only)]
    if drops:
        traces = {n: [] for n, _, _ in drops}
        for i in range(int(3.0 * args.hz)):
            step()
            for n, _, p in drops:
                traces[n].append(pos(p["body"])[2])
        for n, _, p in drops:
            z = traces[n]
            # the first impact: the first minimum; the apex after it: the max until the next min
            i0 = next(i for i in range(1, len(z) - 1) if z[i] <= z[i - 1] and z[i] < z[i + 1] - 1e-6) \
                if any(z[i] < z[i + 1] - 1e-6 for i in range(len(z) - 1)) else None
            if i0 is None:
                res = {"apex_ratio": 0.0, "e_measured": 0.0}
            else:
                apex = max(z[i0:i0 + int(args.hz)]) - 0.05
                ratio = max(apex, 0.0) / p["h0"]
                res = {"apex_ratio": round(ratio, 4), "e_measured": round(math.sqrt(ratio), 4)}
            res["e_authored"] = p["e"]
            results["tests"][n] = res
            print(n, json.dumps(res), flush=True)
    else:
        step(int(1.0 * args.hz))
    # slides: give each cube 2 m/s, track x until it stops
    slides = [t for t in tests if t[1] == "slide" and (not only or t[0] in only)]
    if slides:
        for n, _, p in slides:
            set_velocity(p["body"], (2.0, 0.0, 0.0))
        step(1)
        x0 = {n: pos(p["body"])[0] for n, _, p in slides}
        traces = {n: [] for n, _, _ in slides}
        for i in range(int(2.0 * args.hz)):
            step()
            for n, _, p in slides:
                traces[n].append(pos(p["body"])[0])
        for n, _, p in slides:
            x = [x0[n]] + traces[n]
            v = [(x[i + 1] - x[i]) * args.hz for i in range(len(x) - 1)]
            moving = [i for i in range(len(v)) if v[i] > 0.2]
            # deceleration over the middle of the slide (past the start, before the stop)
            a, b = (moving[len(moving) // 5], moving[4 * len(moving) // 5]) if moving else (0, 0)
            decel = (v[a] - v[b]) / ((b - a) / args.hz) if b > a else float("nan")
            dist = x[-1] - x[0]
            res = {"mu_expected": p["mu"], "mu_measured": round(decel / G, 4),
                   "mu_from_distance": round(4.0 / (2 * G * dist), 4) if dist > 0 else None,
                   "distance": round(dist, 4)}
            results["tests"][n] = res
            print(n, json.dumps(res), flush=True)
    # tips: each plate driven at 2 m/s for 1.5 s
    tips = [t for t in tests if t[1] == "tip" and (not only or t[0] in only)]
    if tips:
        ops = {}
        for n, _, p in tips:
            plate = p["plate"]
            ops[n] = (UsdGeom.XformCommonAPI(plate), Gf.Vec3d(*plate.GetAttribute("xformOp:translate").Get()))
        z_max_tilt = {n: 0.0 for n, _, _ in tips}
        for i in range(int(1.5 * args.hz)):
            t = (i + 1) / args.hz
            for n, _, p in tips:
                xf, c0 = ops[n]
                xf.SetTranslate(Gf.Vec3d(c0[0] + 2.0 * t, c0[1], c0[2]))
            step()
            for n, _, p in tips:
                w, x, y_, z = rot(p["body"])
                tilt = math.degrees(2 * math.acos(min(1.0, abs(w) / math.sqrt(w * w + x * x + y_ * y_ + z * z) if (w or x or y_ or z) else 1.0)))
                z_max_tilt[n] = max(z_max_tilt[n], tilt)
        for n, _, p in tips:
            b = pos(p["body"])
            w, x, y_, z = rot(p["body"])
            up = 1 - 2 * (x * x + y_ * y_)  # the body's z axis's z component
            res = {"mu": p["mu"], "tilt_max_deg": round(z_max_tilt[n], 2),
                   "upright_end": up > 0.9, "moved": round(b[0] - 1.0, 3), "z_end": round(b[2], 4)}
            results["tests"][n] = res
            print(n, json.dumps(res), flush=True)
    # tilts: gravity's direction turned about y by 0.1 deg a step (0.05 s), all at once
    tilts = [t for t in tests if t[1] == "tilt" and (not only or t[0] in only)]
    if tilts:
        start = {n: pos(p["body"]) for n, _, p in tilts}
        slid = {}
        ang = 0.0
        while ang < 60.0 and len(slid) < len(tilts):
            ang += 0.1
            th = math.radians(ang)
            gravity.Set(Gf.Vec3f(math.sin(th), 0.0, -math.cos(th)))
            step(max(1, int(0.05 * args.hz)))
            for n, _, p in tilts:
                if n not in slid and pos(p["body"])[0] - start[n][0] > 0.005:
                    slid[n] = ang
        gravity.Set(Gf.Vec3f(0.0, 0.0, -1.0))
        for n, _, p in tilts:
            a = slid.get(n)
            res = {"mu_authored": p["mu"], "slide_angle_deg": a,
                   "mu_static_measured": round(math.tan(math.radians(a)), 4) if a else None}
            results["tests"][n] = res
            print(n, json.dumps(res), flush=True)
        step(int(0.5 * args.hz))
    results["wall_s"] = round(time.monotonic() - wall0, 2)
    (out / "summary.json").write_text(json.dumps(results, indent=2))
    app.close()


if __name__ == "__main__":
    main()
