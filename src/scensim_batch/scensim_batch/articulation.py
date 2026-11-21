"""Backend B of the batch (roadmap 092 K2.2b, batch-executor.md section 5): the car as an
articulation of rigid bodies and joints, all slots in one Isaac Lab `Articulation`, so the
actuation and the read-back are one tensor write and one read per step (`vectorised = True`).
Backend A (physx_vehicle) is the seats car itself, one car at a time; this one is the fast one,
and what makes it a different car is named, not hidden (the differences below and the body
file's `choices:`).

The car comes from sim_vehicle_spec's neutral body file, `generated/<name>/articulation[_<body>]
.yaml` (sim_vehicle_spec.articulation, from `effective(spec, body)`): links, joints, drives,
masses, contact materials, spec_hash and physics_hash. Here it becomes

- a USD stage (`write_usd`, pxr only: rigid bodies, the joints, their limits, collision shapes
  and physics materials), and
- an Isaac Lab `ArticulationCfg` over it (`articulation_cfg`): the joints' drives as actuators,
  self-collisions off (the chassis box encloses the wheels).

The actuation (`ArticulationModel`, torch only, testable without Isaac) is the component's,
the same `drive`, `brake` [N, 4] and `tire_angle` [N] as backend A takes:

- each wheel's spin joint gets the drive plus its brake, the brake Coulomb friction with
  stiction: full torque against the spin while the wheel turns, never reversing it, holding a
  wheel at rest (what turns with the wheel estimated from the step before);
- the bicycle tire angle becomes the two front steer joints' position targets, as Ackermann
  angles when the body file says so, clipped to the steer limit;
- each suspension joint's position target is its file's (minus the static sag).

A spec whose spin drive is a velocity drive (an RC car's, as WheeledLab's MuSHR) takes joint
targets instead (`apply_joints`): a component's torques are refused for it rather than turned
into a velocity by a rule of ours. That is also the control component bypassed (094 P2 note 3).

The named differences from backend A (and seats): the tyre is a rigid cylinder in Coulomb
contact, no slip curve, no load sensitivity; the brake is an explicit torque, its stiction
from a one-step estimate of what turns with the wheel (no PhysX brake for a joint); the powertrain is the component's torques on the spin joints, no PhysX drivetrain.
"""
import math

import torch

CORNERS = ("front_left", "front_right", "rear_left", "rear_right")   # the component's FL FR RL RR
GRIP_LOAD = 0.1      # of a corner's static load: below it, the tyre carries nothing
GRIP_SLIP = 0.5      # m/s between the tread and the hub: above it, the tyre slides
JOINT_KINDS = ("suspension", "steer", "spin")


def load_body(path):
    """The body file as a dict, its required keys checked."""
    import yaml
    with open(path) as f:
        doc = yaml.safe_load(f)
    check_body(doc, str(path))
    return doc


def check_body(doc, where="body file"):
    for k in ("name", "spec_hash", "physics_hash", "links", "joints", "actuation"):
        if k not in doc:
            raise ValueError(f"{where}: no {k!r} (not a sim_vehicle_spec articulation file)")
    for c in CORNERS:
        for kind in ("suspension", "spin") + (("steer",) if c.startswith("front") else ()):
            if f"{c}_{kind}" not in doc["joints"]:
                raise ValueError(f"{where}: no joint {c}_{kind}")


def joint_order(doc):
    """The joints in a fixed order: the four suspensions, the two steers, the four spins."""
    return ([f"{c}_suspension" for c in CORNERS] + [f"{c}_steer" for c in CORNERS[:2]]
            + [f"{c}_spin" for c in CORNERS])


# --- the actuation (torch, no Isaac) ----------------------------------------------------------

class ArticulationModel:
    """The body file's actuation for N cars: what the joints get from the component's
    actuation, and what the cars' state is from the articulation's read-back."""

    def __init__(self, doc, n, device="cpu"):
        check_body(doc)
        self.doc, self.n, self.device = doc, n, device
        a, j, links = doc["actuation"], doc["joints"], doc["links"]
        self.L, self.T = float(a["wheel_base"]), float(a["wheel_tread"])
        self.r = float(a["wheel_radius"])
        self.max_steer = float(a["max_steer_angle"])
        self.ackermann = "Ackermann" in a.get("steer", "")
        f64 = dict(dtype=torch.float64, device=device)
        self.spin_type = j["front_left_spin"]["drive"]["type"]
        types = {j[f"{c}_spin"]["drive"]["type"] for c in CORNERS}
        if len(types) != 1:
            raise ValueError(f"{doc['name']}: the spin joints' drives differ in type {types}")
        self.spin_I = torch.tensor([links[f"{c}_wheel"]["inertia"][1] for c in CORNERS], **f64)
        self.spin_limit = torch.tensor(
            [j[f"{c}_spin"]["drive"].get("effort_limit", math.inf) for c in CORNERS], **f64)
        self.suspension_target = torch.tensor(
            [j[f"{c}_suspension"]["drive"]["target"] for c in CORNERS], **f64)
        self.driven = torch.tensor([bool(j[f"{c}_spin"].get("driven")) for c in CORNERS],
                                   device=device)
        self._omega = self._tau = self._dt = None     # the step before (spin_efforts)
        self._I = None                                 # each wheel's effective inertia
        self.suspension_k = torch.tensor(
            [j[f"{c}_suspension"]["drive"]["stiffness"] for c in CORNERS], **f64)
        self.static_load = self.suspension_k * -self.suspension_target
        self.hub_y = torch.tensor([j[f"{c}_spin"]["origin"][1] for c in CORNERS], **f64)
        self.car_share = sum(float(l["mass"]) for l in links.values()) / 4
        self._known = torch.zeros(n, 1, dtype=torch.bool, device=device)

    def steer_targets(self, tire_angle):
        """[N, 2] (front left, front right) for the bicycle angle [N] (+ left)."""
        d = torch.as_tensor(tire_angle, dtype=torch.float64, device=self.device)
        d = d.clamp(-self.max_steer, self.max_steer)
        if not self.ackermann:
            return torch.stack([d, d], -1)
        t = torch.tan(d)
        # the inner wheel steers more: tan(d_side) = L tan(d) / (L -+ T/2 tan(d))
        left = torch.atan2(self.L * t, self.L - self.T / 2 * t)
        right = torch.atan2(self.L * t, self.L + self.T / 2 * t)
        # atan2 with a positive denominator keeps the sign of d; clip again (the inner wheel)
        return torch.stack([left, right], -1).clamp(-self.max_steer, self.max_steer)

    def grip(self, suspension_q, omega, hub_speed):
        """[N, 4] bool: whether each tyre carries the car, so its share of the car turns with
        the wheel: loaded (the suspension's spring force, k (q - target), above GRIP_LOAD of
        its static load; no contact sensor needed) and rolling, not sliding (|omega r -
        hub_speed| under GRIP_SLIP m/s). `hub_speed` [N, 4] is each hub's speed along the car's
        x (243's review of K2.2b: a wheel lifting or locking under the brake)."""
        f64 = dict(dtype=torch.float64, device=self.device)
        q = torch.as_tensor(suspension_q, **f64)
        load = self.suspension_k * (q - self.suspension_target)
        slip = (torch.as_tensor(omega, **f64) * self.r - torch.as_tensor(hub_speed, **f64)).abs()
        return (load > GRIP_LOAD * self.static_load) & (slip < GRIP_SLIP)

    def spin_efforts(self, drive, brake, omega, dt, grip=None):
        """[N, 4] spin joint torques for one physics step: the drive plus each wheel's brake.
        The brake is Coulomb friction with stiction: the torque that would bring the wheel to
        rest within `dt`, clipped to the brake's own torque. So it opposes the spin at full
        torque while the wheel turns, never reverses it, and holds a wheel at rest against
        the drive. What brings a wheel to rest depends on what turns with it: the wheel alone
        when it is off the ground or sliding, its share of the car when the tyre grips. That
        effective inertia is estimated from the step before (the torque applied then over the
        wheel's angular acceleration since), between the wheel's own and the wheel's plus a
        quarter of the car's reflected through its radius; before any estimate, the wheel's
        own (so a lifted wheel is never reversed). `grip` [N, 4] (self.grip): a wheel whose
        tyre does not carry the car is braked on its own inertia, whatever the estimate, so a
        wheel that lifts or slides mid-brake is not reversed by the step before's. Call once
        per physics step; `reset` forgets a slot's step before."""
        if self.spin_type != "effort":
            raise ValueError(f"{self.doc['name']}: the spin joints are {self.spin_type} drives: "
                             "give joint targets (apply_joints), not the component's torques")
        f64 = dict(dtype=torch.float64, device=self.device)
        drive = torch.as_tensor(drive, **f64)
        brake = torch.as_tensor(brake, **f64).abs()
        omega = torch.as_tensor(omega, **f64)
        lo, hi = self.spin_I, self.spin_I + self.car_share * self.r * self.r
        if self._I is None:
            self._I = lo.expand_as(omega).clone()
        if self._omega is not None and self._dt is not None:
            alpha = (omega - self._omega) / self._dt
            # an estimate only from a torque that moved the wheel the way it pushed
            ok = self._known & (self._tau.abs() > 1.0) & (alpha * self._tau > 0)
            est = (self._tau / torch.where(ok, alpha, torch.ones_like(alpha)))
            self._I = torch.where(ok, est.clamp(lo, hi), self._I)
        I_now = self._I if grip is None else torch.where(
            torch.as_tensor(grip, dtype=torch.bool, device=self.device), self._I, lo.expand_as(
                self._I))
        hold = -(I_now * omega / dt + drive)            # what brings the wheel to rest
        tau = (drive + torch.maximum(torch.minimum(hold, brake), -brake)).clamp(
            -self.spin_limit, self.spin_limit)
        self._omega, self._tau, self._dt = omega.clone(), tau.clone(), dt
        self._known = torch.ones(self.n, 1, dtype=torch.bool, device=self.device)
        return tau

    def reset(self, mask):
        """Slots `mask` [N] start again: no step before to estimate the road's torque from."""
        if self._omega is not None:
            m = torch.as_tensor(mask, dtype=torch.bool, device=self.device)[:, None]
            self._known = self._known & ~m
            self._I = torch.where(m, self.spin_I.expand_as(self._I), self._I)

    def suspension_targets(self):
        return self.suspension_target.expand(self.n, 4)

    def state(self, pos, quat_xyzw, lin_vel_w, ang_vel_w, spin_vel, prev_vel_b, dt, origins):
        """The batch's state (core.BatchRun's: x, y, yaw, v, yaw_rate, accel_x, accel_y [N],
        wheel_speeds [N, 4]) from the root's world pose and velocity, the spin joints' speeds
        and the body-frame velocity of the step before (`prev_vel_b` [N, 2], None: no
        acceleration yet). Returns (state, this step's body-frame velocity)."""
        f64 = dict(dtype=torch.float64, device=self.device)
        pos, q = torch.as_tensor(pos, **f64), torch.as_tensor(quat_xyzw, **f64)
        lin, ang = torch.as_tensor(lin_vel_w, **f64), torch.as_tensor(ang_vel_w, **f64)
        qx, qy, qz, qw = q.unbind(-1)
        yaw = torch.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        c, s = torch.cos(yaw), torch.sin(yaw)
        vb = torch.stack([c * lin[:, 0] + s * lin[:, 1], -s * lin[:, 0] + c * lin[:, 1]], -1)
        acc = (torch.zeros_like(vb) if prev_vel_b is None or dt is None
               else (vb - prev_vel_b) / dt)
        origins = torch.as_tensor(origins, **f64)
        st = {"x": pos[:, 0] - origins[:, 0], "y": pos[:, 1] - origins[:, 1], "yaw": yaw,
              "v": vb[:, 0], "yaw_rate": ang[:, 2], "accel_x": acc[:, 0], "accel_y": acc[:, 1],
              "wheel_speeds": torch.as_tensor(spin_vel, **f64)}
        return st, vb


# --- the stage (pxr, no Isaac Lab) -------------------------------------------------------------

def _sub(a, b):
    return [float(x) - float(y) for x, y in zip(a, b)]


def _link_position(doc, name):
    return doc["links"][name].get("position", [0.0, 0.0, 0.0])


def write_usd(doc, path, root="/Car"):
    """The body file as a USD layer at `path`: `root` an Xform with the articulation root, a
    rigid body per link (mass, centre of mass, diagonal inertia), its collision shape and
    physics material, and the joints with their axes and limits. The drives' gains are the
    ArticulationCfg's (articulation_cfg), so they are written here only as the drives' kind.
    Returns `root`."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    top = UsdGeom.Xform.Define(stage, root)
    stage.SetDefaultPrim(top.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(top.GetPrim())
    materials = {}

    def material(contact):
        key = tuple(sorted(contact.items()))
        if key in materials:
            return materials[key]
        mp = f"{root}/materials/m{len(materials)}"
        m = UsdShade.Material.Define(stage, mp)
        api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
        api.CreateStaticFrictionAttr(float(contact.get("static_friction", 0.5)))
        api.CreateDynamicFrictionAttr(float(contact.get("dynamic_friction", 0.5)))
        api.CreateRestitutionAttr(float(contact.get("restitution", 0.0)))
        if "combine" in contact:   # PhysX's own schema, written by name (no PhysxSchema here)
            p = m.GetPrim()
            p.AddAppliedSchema("PhysxMaterialAPI")
            for a in ("frictionCombineMode", "restitutionCombineMode"):
                p.CreateAttribute(f"physxMaterial:{a}", Sdf.ValueTypeNames.Token).Set(
                    contact["combine"])
        materials[key] = m
        return m

    for name, link in doc["links"].items():
        pos = _link_position(doc, name)
        lp = f"{root}/{name}"
        x = UsdGeom.Xform.Define(stage, lp)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        prim = x.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.CreateMassAttr(float(link["mass"]))
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*_sub(link["com"], pos)))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, link["inertia"])))
        shape = link.get("shape")
        if not shape:
            continue
        if "box" in shape:
            g = UsdGeom.Cube.Define(stage, f"{lp}/collision")
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*_sub(shape.get("center", pos), pos)))
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, shape["box"])))
        else:
            cyl = shape["cylinder"]
            g = UsdGeom.Cylinder.Define(stage, f"{lp}/collision")
            g.CreateRadiusAttr(float(cyl["radius"]))
            g.CreateHeightAttr(float(cyl["width"]))
            g.CreateAxisAttr(cyl.get("axis", "y").upper())
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        if link.get("contact"):
            UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(
                material(link["contact"]), UsdShade.Tokens.weakerThanDescendants, "physics")

    for name, jt in doc["joints"].items():
        jp = f"{root}/joints/{name}"
        axis = "XYZ"[[abs(v) for v in jt["axis"]].index(1)]
        if jt["type"] == "prismatic":
            j = UsdPhysics.PrismaticJoint.Define(stage, jp)
            lo, hi = jt["limits"]
            drive_kind = "linear"
        else:
            j = UsdPhysics.RevoluteJoint.Define(stage, jp)
            lim = jt.get("limits")
            lo, hi = (math.degrees(lim[0]), math.degrees(lim[1])) if lim else (None, None)
            drive_kind = "angular"
        j.CreateAxisAttr(axis)
        if lo is not None:
            j.CreateLowerLimitAttr(float(lo))
            j.CreateUpperLimitAttr(float(hi))
        j.CreateBody0Rel().SetTargets([f"{root}/{jt['parent']}"])
        j.CreateBody1Rel().SetTargets([f"{root}/{jt['child']}"])
        j.CreateLocalPos0Attr(Gf.Vec3f(*_sub(jt["origin"], _link_position(doc, jt["parent"]))))
        j.CreateLocalPos1Attr(Gf.Vec3f(*_sub(jt["origin"], _link_position(doc, jt["child"]))))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        UsdPhysics.DriveAPI.Apply(j.GetPrim(), drive_kind)
    stage.GetRootLayer().Save()
    return root


# --- Isaac Lab -------------------------------------------------------------------------------

def actuator_groups(doc):
    """{group: (joint names, kind, {stiffness, damping, effort_limit, velocity_limit} per
    joint)}: the drives grouped by kind, as articulation_cfg makes them actuators. Pure data,
    so a test reads what Isaac Lab will get. `kind`: "implicit" (PhysX's drive: the position
    drives, a velocity drive) or "explicit" (an effort drive: our torque, clipped)."""
    groups = {}
    for kind in JOINT_KINDS:
        names = [n for n in joint_order(doc) if n.endswith(f"_{kind}")]
        drives = {n: doc["joints"][n]["drive"] for n in names}
        types = {d["type"] for d in drives.values()}
        if len(types) != 1:
            raise ValueError(f"{doc['name']}: the {kind} joints' drives differ in type {types}")
        t = types.pop()
        gains = {n: {"stiffness": float(d.get("stiffness", 0.0)),
                     "damping": float(d.get("damping", 0.0)),
                     "effort_limit": d.get("effort_limit"),
                     "velocity_limit": d.get("velocity_limit")} for n, d in drives.items()}
        groups[kind] = (names, "explicit" if t == "effort" else "implicit", gains)
    return groups


def articulation_cfg(doc, usd_path, prim_path="/World/envs/env_.*/ego", z=0.0,
                     position_iterations=None, velocity_iterations=None):
    """Isaac Lab's ArticulationCfg for the body file's USD (write_usd). Import after Isaac
    Sim's app."""
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import IdealPDActuatorCfg, ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg

    def per(gains, k):
        vals = {n: g[k] for n, g in gains.items() if g[k] is not None}
        return vals or None

    actuators = {}
    for kind, (names, how, gains) in actuator_groups(doc).items():
        cls = IdealPDActuatorCfg if how == "explicit" else ImplicitActuatorCfg
        actuators[kind] = cls(joint_names_expr=names, stiffness=per(gains, "stiffness"),
                              damping=per(gains, "damping"),
                              effort_limit=per(gains, "effort_limit"),
                              velocity_limit=per(gains, "velocity_limit"))
    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(usd_path),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=position_iterations,
                solver_velocity_iteration_count=velocity_iterations)),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, z), joint_pos={n: 0.0 for n in joint_order(doc)},
            joint_vel={n: 0.0 for n in joint_order(doc)}),
        actuators=actuators)


class ArticulationBackend:
    """core.BatchRun's backend on an Isaac Lab `Articulation` of N cars (articulation_cfg's,
    spawned by the host's scene): reset(mask, x, y, yaw, speed), apply(actuation), advance(dt),
    state(). `sim`: what advance steps (None for a host that owns the physics loop: it calls
    actuate, its own step, then sample); `origins` [N, 2 or 3]: each slot's place."""
    vectorised = True

    def __init__(self, asset, doc, origins, sim=None, z=0.0):
        self.asset, self.doc, self.sim, self.z = asset, doc, sim, z
        self.n = asset.num_instances
        self.device = asset.device
        self.model = ArticulationModel(doc, self.n, "cpu")
        self.origins = torch.as_tensor(origins, dtype=torch.float64)[:, :2].cpu()
        ids, names = asset.find_joints(joint_order(doc), preserve_order=True)
        self.ids = dict(zip(names, ids))
        self.sus_ids = [self.ids[f"{c}_suspension"] for c in CORNERS]
        self.steer_ids = [self.ids[f"{c}_steer"] for c in CORNERS[:2]]
        self.spin_ids = [self.ids[f"{c}_spin"] for c in CORNERS]
        self.act = None
        self.joint_targets = None
        self.prev_vb = None
        self.dt = None
        self._state = None
        self.timing = {"actuate": 0.0, "physics": 0.0, "sample": 0.0, "steps": 0}

    def _dev(self, t):
        return torch.as_tensor(t, dtype=torch.float32).to(self.device)

    def reset(self, mask, x, y, yaw, speed):
        ids = torch.nonzero(torch.as_tensor(mask, dtype=torch.bool)).flatten()
        if len(ids) == 0:
            return
        f = lambda a: torch.as_tensor(a, dtype=torch.float64)[ids]   # noqa: E731
        h, v = f(yaw), f(speed)
        ox, oy = self.origins[ids, 0], self.origins[ids, 1]
        zeros = torch.zeros_like(h)
        pose = torch.stack([ox + f(x), oy + f(y), torch.full_like(h, self.z), zeros, zeros,
                            torch.sin(h / 2), torch.cos(h / 2)], -1)      # x y z, qx qy qz qw
        vel = torch.stack([v * torch.cos(h), v * torch.sin(h), zeros, zeros, zeros, zeros], -1)
        env_ids = ids.to(self.device)
        a = self.asset
        a.write_root_pose_to_sim_index(root_pose=self._dev(pose), env_ids=env_ids)
        a.write_root_velocity_to_sim_index(root_velocity=self._dev(vel), env_ids=env_ids)
        nj = a.num_joints
        jpos = torch.zeros(len(ids), nj)
        jvel = torch.zeros(len(ids), nj)
        jvel[:, self.spin_ids] = (v / self.model.r)[:, None].float()   # the wheels rolling
        a.write_joint_position_to_sim_index(position=self._dev(jpos), env_ids=env_ids)
        a.write_joint_velocity_to_sim_index(velocity=self._dev(jvel), env_ids=env_ids)
        self.model.reset(torch.as_tensor(mask, dtype=torch.bool))
        if self.prev_vb is not None:     # a reset slot starts with no acceleration
            self.prev_vb[ids] = torch.stack([v, zeros], -1)
        self._state = None

    def apply(self, act, velocity=None, gear=None):
        """The coming step's actuation (the component's: drive, brake [N, 4], tire_angle [N]).
        `velocity` and `gear` are backend A's sticky-tire release and mean nothing here."""
        self.act, self.joint_targets = act, None

    def apply_joints(self, steer, spin):
        """The control component bypassed: the front steer joints' targets [N, 2] (rad) and the
        spin joints' targets [N, 4] (rad/s for a velocity drive, N m for an effort drive)."""
        self.act, self.joint_targets = None, (steer, spin)

    def actuate(self, dt):
        a, m = self.asset, self.model
        nj = a.num_joints
        pos_t = torch.zeros(self.n, nj, dtype=torch.float64)
        pos_t[:, self.sus_ids] = m.suspension_targets()
        if self.joint_targets is not None:
            steer, spin = (torch.as_tensor(t, dtype=torch.float64) for t in self.joint_targets)
        else:
            steer = m.steer_targets(self.act["tire_angle"])
            omega = a.data.joint_vel.torch[:, self.spin_ids].detach().cpu().double()
            q = a.data.joint_pos.torch[:, self.sus_ids].detach().cpu().double()
            st = self.state()
            # each hub's speed along the car's x: v - yaw_rate y (the steer's cosine ignored:
            # a steered wheel reads as sliding sooner, which only brakes it on its own inertia)
            hub = st["v"][:, None] - st["yaw_rate"][:, None] * m.hub_y
            spin = m.spin_efforts(self.act["drive"], self.act["brake"], omega, dt,
                                  grip=m.grip(q, omega, hub))
        pos_t[:, self.steer_ids] = steer
        a.set_joint_position_target_index(target=self._dev(pos_t))
        if m.spin_type == "velocity":
            a.set_joint_velocity_target_index(target=self._dev(spin), joint_ids=self.spin_ids)
        else:
            a.set_joint_effort_target_index(target=self._dev(spin), joint_ids=self.spin_ids)
        a.write_data_to_sim()

    def advance(self, dt):
        import time
        t0 = time.perf_counter()
        self.actuate(dt)
        t1 = time.perf_counter()
        self.sim.step(render=False)
        t2 = time.perf_counter()
        self.sample(dt)
        t3 = time.perf_counter()
        tm = self.timing
        tm["actuate"] += t1 - t0
        tm["physics"] += t2 - t1
        tm["sample"] += t3 - t2
        tm["steps"] += 1

    def sample(self, dt):
        self.asset.update(dt)
        self.dt = dt
        self._state = None

    def state(self):
        """The cars as the last sample (or reset) left them; read once per physics step, so
        the acceleration is that step's however often a step asks."""
        if self._state is not None:
            return self._state
        d = self.asset.data
        t = lambda p: p.torch.detach().cpu()   # noqa: E731
        pose, vel = t(d.root_link_pose_w), t(d.root_link_vel_w)
        st, vb = self.model.state(pose[:, :3], pose[:, 3:7], vel[:, :3], vel[:, 3:6],
                                  t(d.joint_vel)[:, self.spin_ids], self.prev_vb, self.dt,
                                  self.origins)
        self.prev_vb, self.dt, self._state = vb, None, st
        return st
