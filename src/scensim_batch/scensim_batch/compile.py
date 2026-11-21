"""A scenario and N sample records as one batch's tensors (roadmap 092 K2.2; batch-executor.md
section 4): the entity table, each slot's parameters, and each entity's initial pose and speed.

    c = compile_scenario("cs2.xosc", records, world)

The scenario must pass the compilable subset (`scensim_sampler.subset.check`), and the
environment must meet every condition its verdict lists: `provides` names those it meets (for
example "SSv2's default controller", which scensim_batch.npc provides). The map is the world
adapter's (`world.lane_pose(lane_id, s, offset)` -> (x, y, yaw) per slot, `world.route(x, y,
yaw, length)` -> centreline points), so this module reads no map format itself.

Compiled today: Init TeleportActions to WorldPosition, LanePosition, RelativeWorldPosition and
RelativeObjectPosition (relative to an entity already placed); Init SpeedActions to an absolute
target; inline BoundingBoxes; parameters as `$name` (each slot's record value, else the
declared default) or literals. Anything else in Init, an expression `${...}`, and an entity
from a catalog are refused here by name (K2.3 and later).
"""
import math
from dataclasses import dataclass, field

import torch

from scensim_sampler import subset

PROVIDED = ("SSv2's default controller",)   # the conditions scensim_batch itself meets


class CompileError(ValueError):
    pass


@dataclass
class Entity:
    name: str
    role: str          # "ego", "default" (SSv2's default behaviour), "prop"
    kind: str          # "vehicle", "pedestrian", "misc"
    length: float
    width: float
    height: float
    cx: float = 0.0    # the box's centre in the entity's frame (BoundingBox/Center)
    cy: float = 0.0


@dataclass
class Compiled:
    scenario: str
    n: int
    entities: list
    params: dict                     # name -> [N] tensor (numeric) or list of N (string)
    x: torch.Tensor                  # [N, E] initial pose and speed
    y: torch.Tensor
    yaw: torch.Tensor
    speed: torch.Tensor
    placed: torch.Tensor             # [N, E] bool: Init placed it
    speed_set: torch.Tensor = None   # [N, E] bool: an Init SpeedAction set its speed
    verdict: object = None
    seeds: list = field(default_factory=list)
    storyboard: object = None        # storyboard.Storyboard (K2.3)
    goal_waypoints: list = None      # the ego's Init routing: [(x, y, yaw) [N]], the route goal
    records: list = field(default_factory=list)

    def index(self, name):
        return [e.name for e in self.entities].index(name)

    def of_role(self, role):
        return [i for i, e in enumerate(self.entities) if e.role == role]


def _number(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


class _Params:
    def __init__(self, declared, records):
        self.declared, self.records = declared, records
        self.n = len(records)
        self.cache = {}

    def value(self, text, where):
        """An attribute: a literal, or `$name` per slot. Returns a list of N."""
        if text is None:
            raise CompileError(f"{where}: missing")
        text = text.strip()
        if text.startswith("${"):
            raise CompileError(f"{where}: expression {text!r} (not compiled yet)")
        if text.startswith("-$"):          # a negated parameter, as SSv2 reads it (CS2's -$decel)
            vals = self.value(text[1:], where)
            nums = [_number(v) for v in vals]
            if any(v is None for v in nums):
                raise CompileError(f"{where}: {text!r} negates a parameter that is not a number")
            return [repr(-v) for v in nums]
        if text.startswith("$"):
            name = text[1:]
            if name in self.cache:
                return self.cache[name]
            if name not in self.declared:
                raise CompileError(f"{where}: parameter {name!r} is not declared")
            ptype, default = self.declared[name]
            out = []
            for r in self.records:
                v = (r.get("values") or {}).get(name, default)
                out.append(v)
            self.cache[name] = out
            return out
        return [text] * self.n

    def floats(self, text, where):
        vals = self.value(text, where)
        out = [_number(v) for v in vals]
        if any(v is None for v in out):
            raise CompileError(f"{where}: {text!r} is not a number in every slot")
        return torch.tensor(out, dtype=torch.float64)


def _box(obj, where):
    dims = obj.find(".//BoundingBox/Dimensions")
    if dims is None:
        if obj.find("CatalogReference") is not None:
            raise CompileError(f"{where}: an entity from a catalog (not compiled yet)")
        raise CompileError(f"{where}: no BoundingBox")
    centre = obj.find(".//BoundingBox/Center")
    cx, cy = ((float(centre.get("x", 0)), float(centre.get("y", 0))) if centre is not None
              else (0.0, 0.0))
    return tuple(float(dims.get(k)) for k in ("length", "width", "height")) + (cx, cy)


def _kind(obj):
    for tag, kind in (("Vehicle", "vehicle"), ("Pedestrian", "pedestrian"),
                      ("MiscObject", "misc")):
        if obj.find(tag) is not None:
            return kind
    return None


def compile_scenario(path, records, world, provides=(), device="cpu"):
    """The batch for `records` (N sample records, slot i = records[i]) of scenario `path`."""
    if not records:
        raise CompileError(f"{path}: no records (a batch of no slots)")
    v = subset.check(path)
    if not v.compiles:
        raise CompileError(f"{path}: {v.line()}")
    met = set(PROVIDED) | set(provides)
    unmet = [c for c in v.conditions if c.split(":")[0] not in met]
    if unmet:
        raise CompileError(f"{path}: compiles only if: {'; '.join(unmet)}; this environment "
                           f"provides {', '.join(sorted(met))}")
    root, spath = subset.read(path)
    from scensim_sampler.xosc import Scenario
    declared = Scenario(spath).parameters
    P = _Params(declared, records)
    n = len(records)

    entities = []
    for obj in root.findall("Entities/ScenarioObject"):
        name = obj.get("name")
        where = f"Entities/ScenarioObject[@name='{name}']"
        kind = _kind(obj)
        if kind is None:
            raise CompileError(f"{where}: neither Vehicle, Pedestrian nor MiscObject")
        length, width, height, cx, cy = _box(obj, where)
        if name in v.egos:
            role = "ego"
        elif kind == "misc":
            role = "prop"
        elif kind == "vehicle" and not list(obj.iter("Controller")):
            role = "default"
        else:
            raise CompileError(f"{where}: a {kind} with a controller or behaviour model "
                               "(not compiled yet)")
        entities.append(Entity(name, role, kind, length, width, height, cx, cy))
    names = [e.name for e in entities]
    e = len(entities)
    z = torch.zeros(n, e, dtype=torch.float64)
    x, y, yaw, speed = z.clone(), z.clone(), z.clone(), z.clone()
    placed = torch.zeros(n, e, dtype=torch.bool)
    speed_set = torch.zeros(n, e, dtype=torch.bool)
    goal = None

    for pr in root.findall("Storyboard/Init/Actions/Private"):
        ref = pr.get("entityRef")
        if ref not in names:
            raise CompileError(f"Init/Private[@entityRef='{ref}']: no such entity")
        i = names.index(ref)
        for k, pa in enumerate(pr.findall("PrivateAction")):
            where = f"Storyboard/Init/Actions/Private[@entityRef='{ref}']/PrivateAction[{k + 1}]"
            pos = pa.find("TeleportAction/Position")
            sp = pa.find("LongitudinalAction/SpeedAction/SpeedActionTarget/AbsoluteTargetSpeed")
            if pos is not None:
                px, py, pyaw = _position(pos, where, P, world, names, x, y, yaw, placed)
                x[:, i], y[:, i], yaw[:, i] = px, py, pyaw
                placed[:, i] = True
            elif sp is not None:
                speed[:, i] = P.floats(sp.get("value"), where + "/.../AbsoluteTargetSpeed")
                speed_set[:, i] = True
            elif pa.find("ControllerAction") is not None:
                continue   # the ego's pilot: the batch's policy (subset.check said so)
            elif pa.find("RoutingAction") is not None:
                from . import storyboard as sbmod
                ra = next(iter(pa.find("RoutingAction")))
                if ra.tag not in sbmod.EGO_GOAL or entities[i].role != "ego":
                    raise CompileError(f"{where}: RoutingAction/{ra.tag} on {ref!r} (batch "
                                       "executor pending)")
                # the policy's task: the slot's route goal (Init positions may be relative)
                try:
                    goal = sbmod.routing_waypoints(
                        ra, f"{where}/RoutingAction/{ra.tag}",
                        lambda p_, w_: _position(p_, w_, P, world, names, x, y, yaw, placed))
                except sbmod.StoryboardError as err:
                    raise CompileError(str(err)) from None
            else:
                tag = next(iter(pa), None)
                raise CompileError(f"{where}: {getattr(tag, 'tag', 'empty')} in Init "
                                   "(not compiled yet)")
    params = {}
    for name, (ptype, _) in declared.items():
        vals = P.value("$" + name, f"ParameterDeclaration {name}")
        nums = [_number(t) for t in vals]
        params[name] = (torch.tensor(nums, dtype=torch.float64)
                        if ptype in ("double", "int", "unsignedInt", "unsignedShort")
                        and all(t is not None for t in nums) else list(vals))
    seeds = [(r.get("seeds") or {}).get("scenario") for r in records]
    from . import storyboard as sbmod
    info = _StoryInfo(n, P, entities, world, names, x, y, yaw, placed)
    sb_el = root.find("Storyboard")
    try:
        story = sbmod.compile_storyboard(sb_el, info) if sb_el is not None else None
    except sbmod.StoryboardError as e:
        raise CompileError(str(e)) from None
    to = (lambda t: t.to(device))
    return Compiled(str(spath), n, entities, params, to(x), to(y), to(yaw), to(speed),
                    placed.to(device), speed_set.to(device), v, seeds, storyboard=story,
                    records=list(records), goal_waypoints=goal)


class _StoryInfo:
    """What the storyboard's compile reads: the slots, parameters, entities, map."""

    def __init__(self, n, P, entities, world, names, x, y, yaw, placed):
        self.n, self.P, self.entities, self.world = n, P, entities, world
        self.names, self.x, self.y, self.yaw, self.placed = names, x, y, yaw, placed
        self.defaults = [i for i, e in enumerate(entities) if e.role == "default"]

    def index(self, name, where):
        if name not in self.names:
            raise CompileError(f"{where}: no entity {name!r}")
        return self.names.index(name)

    def position(self, pos, where):
        if pos is None:
            raise CompileError(f"{where}: missing")
        return _position(pos, where, self.P, self.world, self.names, self.x, self.y, self.yaw,
                         self.placed)


def _position(pos, where, P, world, names, x, y, yaw, placed):
    w = pos.find("WorldPosition")
    if w is not None:
        h = w.get("h")
        return (P.floats(w.get("x"), where + "/WorldPosition@x"),
                P.floats(w.get("y"), where + "/WorldPosition@y"),
                P.floats(h, where + "/WorldPosition@h") if h is not None
                else torch.zeros(P.n, dtype=torch.float64))
    lp = pos.find("LanePosition")
    if lp is not None:
        lanes = P.value(lp.get("laneId"), where + "/LanePosition@laneId")
        s = P.floats(lp.get("s"), where + "/LanePosition@s")
        off = P.floats(lp.get("offset") or "0", where + "/LanePosition@offset")
        px, py, pyaw = world.lane_pose(lanes, s, off)
        return (torch.as_tensor(px, dtype=torch.float64), torch.as_tensor(py, dtype=torch.float64),
                torch.as_tensor(pyaw, dtype=torch.float64))
    for tag in ("RelativeWorldPosition", "RelativeObjectPosition"):
        rp = pos.find(tag)
        if rp is not None:
            ref = rp.get("entityRef")
            if ref not in names or not bool(placed[:, names.index(ref)].all()):
                raise CompileError(f"{where}/{tag}: entity {ref!r} is not placed before it")
            j = names.index(ref)
            dx = P.floats(rp.get("dx"), f"{where}/{tag}@dx")
            dy = P.floats(rp.get("dy"), f"{where}/{tag}@dy")
            if tag == "RelativeObjectPosition":   # in the reference's own frame
                c, s_ = torch.cos(yaw[:, j]), torch.sin(yaw[:, j])
                dx, dy = c * dx - s_ * dy, s_ * dx + c * dy
            return x[:, j] + dx, y[:, j] + dy, yaw[:, j].clone()
    tag = next(iter(pos), None)
    raise CompileError(f"{where}: {getattr(tag, 'tag', 'empty')} (not compiled yet)")


__all__ = ["CompileError", "Compiled", "Entity", "PROVIDED", "compile_scenario"]
_ = math
