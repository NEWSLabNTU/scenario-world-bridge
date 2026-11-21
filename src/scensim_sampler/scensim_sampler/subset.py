"""The compilable subset of OpenSCENARIO: which constructs a batch executor (an Isaac Lab
environment, roadmap 092 K2) can run per slot, and which only run on seats (SSv2 with a
simulator). The spec is docs/design/compilable-subset.md; this module is its table, and the
checker behind `scensim-sample compilable`.

Every element in a scenario's Storyboard and every entity's controller is classified:

- batch: compiles per slot;
- derived: compiles once rewritten (named), e.g. TimeToCollisionCondition as a distance on a
  derived parameter;
- conditional: compiles when the slot provides something (a camera and a renderer for weather,
  a batched model for a behaviour), which the verdict lists;
- seats: never batches, with the reason.

An element the table does not list is refused as "not in the subset": the table grows by
decision, not by default. "batch" is a claim until K2's adapter runs each construct and
compares it against seats (K2's acceptance).

Reads .xosc, and SSv2's YAML scenario format (read as the same XML tree). A
ParameterValueDistribution is read through to its ScenarioFile: a distribution changes values,
not constructs.
"""
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

BATCH, DERIVED, CONDITIONAL, SEATS = "batch", "derived", "conditional", "seats"

# containers and data: they hold the constructs, and compile when their contents do
STRUCTURE = set("""
Storyboard Init Actions Private PrivateAction Story Act ManeuverGroup Actors EntityRef Maneuver
Event Action StartTrigger StopTrigger ConditionGroup Condition ByEntityCondition
TriggeringEntities EntityCondition ByValueCondition GlobalAction UserDefinedAction
LongitudinalAction LateralAction RoutingAction Position Orientation Properties Property
ParameterDeclarations ParameterDeclaration ParameterAssignments ParameterAssignment
CatalogReference Route Waypoint SpeedActionDynamics SpeedActionTarget LaneChangeActionDynamics
LaneChangeTarget Trajectory Shape Polyline Vertex TimeReference Timing TrajectoryFollowingMode
TrajectoryRef DynamicConstraints SpeedProfileEntry InfrastructureAction TrafficSignalAction
TimeToCollisionConditionTarget RelativeLaneRange ByType EnvironmentAction Environment Weather
EntityAction
""".split())

PER_SLOT = "per slot, vectorised"
TABLE = {
    # positions and placing
    "TeleportAction": (BATCH, "a per-slot reset pose"),
    "WorldPosition": (BATCH, PER_SLOT),
    "LanePosition": (BATCH, "resolved on the world's map once per slot"),
    "RelativeWorldPosition": (BATCH, PER_SLOT),
    "RelativeObjectPosition": (BATCH, PER_SLOT),
    "RelativeLanePosition": (BATCH, "resolved on the world's map once per slot"),
    # motion
    "SpeedAction": (BATCH, "a speed profile per slot (step, linear, cubic)"),
    "AbsoluteTargetSpeed": (BATCH, PER_SLOT),
    "RelativeTargetSpeed": (BATCH, PER_SLOT),
    "SpeedProfileAction": (BATCH, "a speed profile per slot"),
    "LaneChangeAction": (BATCH, "a lateral profile per slot"),
    "AbsoluteTargetLane": (BATCH, PER_SLOT),
    "RelativeTargetLane": (BATCH, PER_SLOT),
    "LaneOffsetAction": (BATCH, "a lateral profile per slot"),
    "FollowTrajectoryAction": (BATCH, "precomputed per slot"),
    "AssignRouteAction": (BATCH, "a route precomputed per slot on the world's map"),
    "AcquirePositionAction": (BATCH, "a route precomputed per slot on the world's map"),
    "RoadCondition": (BATCH, "friction as a per-slot material"),
    # conditions
    "DistanceCondition": (BATCH, PER_SLOT),
    "RelativeDistanceCondition": (BATCH, PER_SLOT),
    "RelativeClearanceCondition": (BATCH, PER_SLOT),
    "ReachPositionCondition": (BATCH, PER_SLOT),
    "SimulationTimeCondition": (BATCH, PER_SLOT),
    "TimeHeadwayCondition": (BATCH, PER_SLOT),
    "SpeedCondition": (BATCH, PER_SLOT),
    "RelativeSpeedCondition": (BATCH, PER_SLOT),
    "AccelerationCondition": (BATCH, PER_SLOT),
    "StandStillCondition": (BATCH, PER_SLOT),
    "TraveledDistanceCondition": (BATCH, PER_SLOT),
    "CollisionCondition": (BATCH, "contact per slot"),
    "StoryboardElementStateCondition": (BATCH, "the storyboard's state machine per slot"),
    "ParameterCondition": (BATCH, "a parameter fixed for the slot (no mid-run set)"),
    "TimeToCollisionCondition": (DERIVED, "SSv2 implements it only partly",
                                 "write it as a DistanceCondition on a parameter the sampler "
                                 "derives (speed x time to collision; scenario-description.md CS1)"),
    # signals
    "TrafficSignalStateAction": (BATCH, "a per-slot signal state"),
    "TrafficSignalControllerAction": (BATCH, "a per-slot signal phase"),
    # the environment: what the slot's sensors see
    "TimeOfDay": (CONDITIONAL, "a slot camera (illumination), and its render time"),
    "Fog": (CONDITIONAL, "a slot camera, and a renderer with fog per slot"),
    "Precipitation": (CONDITIONAL, "a slot camera or lidar model with precipitation per slot"),
    "Sun": (CONDITIONAL, "a slot camera (sun position), and its render time"),
    "Wind": (CONDITIONAL, "a wind force model per slot"),
    "DomeFile": (CONDITIONAL, "a slot camera (a sky dome), and its render time"),
    "DomeImage": (CONDITIONAL, "a slot camera (a sky dome), and its render time"),
    # seats only
    "AddEntityAction": (SEATS, "a slot has a fixed set of entities"),
    "DeleteEntityAction": (SEATS, "a slot has a fixed set of entities"),
    "ParameterAction": (SEATS, "sets a parameter mid-run; the batch's parameters are fixed per slot"),
    "SetAction": (SEATS, "sets a parameter mid-run; the batch's parameters are fixed per slot"),
    "ModifyAction": (SEATS, "modifies a parameter mid-run"),
    "AddValue": (SEATS, "modifies a parameter mid-run"),
    "Rule": (SEATS, "modifies a parameter mid-run"),
    "ParameterSetAction": (SEATS, "sets a parameter mid-run"),
    "ParameterModifyAction": (SEATS, "modifies a parameter mid-run"),
    "ControllerAction": (SEATS, "swaps or overrides a pilot mid-run"),
    "AssignControllerAction": (SEATS, "assigns a pilot mid-run"),
    "OverrideControllerValueAction": (SEATS, "overrides an external pilot's pedals or steering"),
    "ActivateControllerAction": (SEATS, "switches a pilot's lateral or longitudinal control"),
    "VisibilityAction": (SEATS, "changes what an external pilot's sensors see"),
}
for _t in ("Brake", "Throttle", "Clutch", "Gear", "ParkingBrake", "SteeringWheel"):
    TABLE[_t] = (SEATS, "overrides an external pilot's controls")

# CustomCommandAction types
COMMANDS = {
    "exitSuccess": (BATCH, "the slot's end, success"),
    "exitFailure": (BATCH, "the slot's end, failure"),
    "scensim.fault": (BATCH, "a fault as a per-slot tensor mask (scenario-description.md CS4)"),
    ":": (BATCH, "does nothing"),
}

# world knobs (ParameterDeclarations named world.<knob>, bound by the world interface)
WORLD_KNOBS = {
    "surface": (BATCH, "friction as a per-slot material"),
    "zone_layout": (BATCH, "a prop set, instanced per slot"),
    "illumination": (CONDITIONAL, "a slot camera (illumination), and its render time"),
    "weather": (CONDITIONAL, "a slot camera, and a renderer that supports it per slot"),
    "fog": (CONDITIONAL, "a slot camera, and a renderer with fog per slot"),
}
UNKNOWN_KNOB = (CONDITIONAL, "the world supports this knob per slot")
PILOT_PARAM = "pilot.* parameters go to the batch's policy as observation inputs"

EGO_POLICY = ("compiles with the ego replaced by the batch's policy; the scenario's own "
              "autopilot does not run in a batch")


class SubsetError(ValueError):
    pass


# --- reading --------------------------------------------------------------------------------

def _element(tag, value):
    """SSv2's YAML scenario format as the XML tree: mappings are elements, scalars attributes,
    lists repeated elements."""
    el = ET.Element(tag)
    if isinstance(value, dict):
        for k, v in value.items():
            if isinstance(v, dict):
                el.append(_element(k, v))
            elif isinstance(v, list):
                for item in v:
                    el.append(_element(k, item if isinstance(item, dict) else {}))
            elif v is not None:
                el.set(k, str(v))
    return el


def read(path):
    """`(root, scenario path)`: the scenario's OpenSCENARIO tree, a distribution read through."""
    path = Path(path)
    try:
        if path.suffix in (".yaml", ".yml"):
            data = yaml.safe_load(path.read_text())
            if not isinstance(data, dict) or "OpenSCENARIO" not in data:
                raise SubsetError(f"{path}: no OpenSCENARIO: at the top")
            root = _element("OpenSCENARIO", data["OpenSCENARIO"])
        else:
            root = ET.parse(path).getroot()
    except (OSError, yaml.YAMLError, ET.ParseError) as e:
        raise SubsetError(f"{path}: {e}") from None
    pvd = root.find("ParameterValueDistribution")
    if pvd is not None:
        sf = pvd.find("ScenarioFile")
        if sf is None or not sf.get("filepath"):
            raise SubsetError(f"{path}: a distribution without a ScenarioFile")
        return read(path.parent / sf.get("filepath"))
    return root, path


# --- checking -------------------------------------------------------------------------------

class Verdict:
    def __init__(self, path):
        self.path = str(path)
        self.refusals, self.conditions, self.notes, self.counts = [], [], [], {}

    def add(self, cls, where, what, reason, fix=None):
        self.counts[cls] = self.counts.get(cls, 0) + 1
        if cls in (SEATS, DERIVED, "unlisted"):
            self.refusals.append({"class": cls, "construct": what, "at": where,
                                  "reason": reason, "fix": fix})
        elif cls == CONDITIONAL:
            c = f"{what}: {reason}"
            if c not in self.conditions:
                self.conditions.append(c)

    @property
    def compiles(self):
        return not self.refusals

    def line(self):
        if self.refusals:
            r = self.refusals[0]
            fix = f"; fix: {r['fix']}" if r["fix"] else ""
            more = (f" (and {len(self.refusals) - 1} more)" if len(self.refusals) > 1 else "")
            return f"refused: {r['construct']} at {r['at']} ({r['reason']}{fix}){more}"
        ego = EGO_POLICY in self.notes
        s = EGO_POLICY if ego else "compiles"
        rest = [n for n in self.notes if n != EGO_POLICY]
        if rest:
            s += "; " + "; ".join(rest)
        if self.conditions:
            s += "; compiles only if: " + "; ".join(self.conditions)
        return s

    def to_dict(self):
        return {"scenario": self.path, "compiles": self.compiles, "verdict": self.line(),
                "refusals": self.refusals, "conditions": self.conditions, "notes": self.notes,
                "counts": self.counts}


def _step(el):
    name = el.get("name")
    return f"{el.tag}[@name='{name}']" if name else el.tag


def _is_ego(obj):
    for c in obj.iter("Controller"):
        if (c.get("name") or "").lower() == "autoware":
            return True
        for p in c.iter("Property"):
            if p.get("name") == "isEgo" and str(p.get("value")).lower() == "true":
                return True
    return False


def check(path, ego="policy"):
    """The Verdict for one scenario file. `ego`: "policy" (the ego is the batch's policy) or
    "seats" (an ego driven by an autopilot is refused)."""
    if ego not in ("policy", "seats"):
        raise SubsetError(f"ego is policy or seats, not {ego!r}")
    root, spath = read(path)
    v = Verdict(spath)
    egos = {o.get("name") for o in root.findall("Entities/ScenarioObject") if _is_ego(o)}
    # an Init controller assignment (Autoware to an entity) makes it an ego too
    for pr in root.findall("Storyboard/Init/Actions/Private"):
        for c in pr.iter("Controller"):
            if (c.get("name") or "").lower() == "autoware" or any(
                    q.get("name") == "isEgo" and str(q.get("value")).lower() == "true"
                    for q in c.iter("Property")):
                egos.add(pr.get("entityRef"))
    v.egos, v.ego_mode = egos, ego
    if len(egos) > 1 and ego == "policy":
        v.add(SEATS, "/OpenSCENARIO/Entities", f"{len(egos)} egos ({', '.join(sorted(egos))})",
              "a batch has one policy per slot; a scenario with two autopilots runs on seats")
    for pd in (root.findall("ParameterDeclarations/ParameterDeclaration")):
        name = pd.get("name") or ""
        where = f"/OpenSCENARIO/ParameterDeclarations/ParameterDeclaration[@name='{name}']"
        if name.startswith("world."):
            cls, reason = WORLD_KNOBS.get(name[6:], UNKNOWN_KNOB)
            v.add(cls, where, f"world knob {name[6:]}={pd.get('value')}", reason)
        elif name.startswith("pilot.") and PILOT_PARAM not in v.notes:
            v.notes.append(PILOT_PARAM)
    for obj in root.findall("Entities/ScenarioObject"):
        where = f"/OpenSCENARIO/Entities/{_step(obj)}"
        if obj.get("name") in egos:
            if ego == "seats":
                v.add(SEATS, where + "/ObjectController", "the ego's autopilot",
                      "the scenario's own autopilot (Autoware) does not run in a batch")
            elif EGO_POLICY not in v.notes:
                v.notes.append(EGO_POLICY)
            continue
        for c in obj.iter("Controller"):
            props = {p.get("name"): p.get("value") for p in c.iter("Property")}
            name = (c.get("name") or "")
            if props.get("model", "").startswith("scensim.behaviour:"):
                v.add(CONDITIONAL, where, f"behaviour model {props['model']}",
                      "a batched form of the model")
            elif name.lower() in ("autoware", "agent") or props.get("pilot"):
                v.add(SEATS, where + "/ObjectController", f"controller {name or 'agent'}",
                      "an external pilot on a non-ego entity does not run in a batch")
            else:
                v.add(CONDITIONAL, where, "SSv2's default controller",
                      "a batched model of SSv2's default traffic behaviour (lane following, "
                      "stopping for obstacles)")
        if not list(obj.iter("Controller")):
            if obj.find("MiscObject") is not None:
                v.add(BATCH, where, "MiscObject", "a static prop, instanced per slot")
            else:   # a vehicle or pedestrian with no controller named: SSv2's default
                v.add(CONDITIONAL, where, "SSv2's default controller",
                      "a batched model of SSv2's default traffic behaviour (lane following, "
                      "stopping for obstacles)")
    sb = root.find("Storyboard")
    if sb is not None:
        _walk(sb, "/OpenSCENARIO/Storyboard", v, in_init=False, entity=None)
    return v


def _init_controller(el, here, v, entity):
    """A ControllerAction in Init: the entity's pilot, set before the run (static)."""
    if entity in v.egos:
        if v.ego_mode == "seats":
            v.add(SEATS, here, "the ego's autopilot", "the scenario's own autopilot "
                                                       "(Autoware) does not run in a batch")
        elif EGO_POLICY not in v.notes:
            v.notes.append(EGO_POLICY)
        return
    names = {(c.get("name") or "").lower() for c in el.iter("Controller")}
    if names & {"autoware", "agent"}:
        v.add(SEATS, here, "ControllerAction", "an external pilot on a non-ego entity does not "
                                               "run in a batch")
    else:
        v.add(CONDITIONAL, here, "SSv2's default controller",
              "a batched model of SSv2's default traffic behaviour (lane following, stopping "
              "for obstacles)")


def _walk(el, where, v, in_init, entity):
    here = where if el.tag == "Storyboard" else f"{where}/{_step(el)}"
    init = in_init or el.tag == "Init"
    entity = el.get("entityRef") if el.tag == "Private" else entity
    tag = el.tag
    if init and tag == "ControllerAction":
        _init_controller(el, here, v, entity)
        return
    if tag == "CustomCommandAction":
        t = (el.get("type") or "").strip()
        key = t.split("(")[0].strip()
        cls, reason = COMMANDS.get(key, (SEATS, "a command run as a process on seats; only "
                                                "exitSuccess, exitFailure and scensim.fault "
                                                "compile"))
        v.add(cls, here, f"CustomCommandAction {t or '(empty)'}", reason)
        return
    if tag == "UserDefinedValueCondition":
        name = (el.get("name") or "").strip()
        if re.match(r"^RelativeHeadingCondition\s*\(", name):
            v.add(BATCH, here, f"UserDefinedValueCondition {name}",
                  "SSv2 computes it from the entity's pose against its lane: per slot")
        elif name.startswith("/"):
            v.add(SEATS, here, f"UserDefinedValueCondition {name}",
                  "reads a ROS topic, outside the scenario")
        else:
            v.add(SEATS, here, f"UserDefinedValueCondition {name or '(empty)'}",
                  "reads an autopilot's state (e.g. <entity>.currentState), outside the scenario")
        return
    if tag in STRUCTURE:
        pass
    elif tag in TABLE:
        entry = TABLE[tag]
        cls, reason = entry[0], entry[1]
        fix = entry[2] if len(entry) > 2 else None
        if init and tag in ("ParameterAction", "SetAction", "ParameterSetAction"):
            cls, reason = BATCH, "set before the run: fixed for the slot"
        v.add(cls, here, tag, reason, fix)
        if cls in (SEATS, DERIVED):
            return   # its contents are its own
    else:
        v.add("unlisted", here, tag, "not in the compilable subset (docs/design/"
                                     "compilable-subset.md lists what is)")
        return
    for child in el:
        _walk(child, here, v, init, entity)


def summary(verdicts):
    """`{scenarios, compile, compile_unconditionally, conditional, refused, first_refusals}`."""
    first = {}
    for x in verdicts:
        if x.refusals:
            k = x.refusals[0]["construct"]
            first[k] = first.get(k, 0) + 1
    return {"scenarios": len(verdicts),
            "compile": sum(x.compiles for x in verdicts),
            "compile_unconditionally": sum(x.compiles and not x.conditions for x in verdicts),
            "conditional": sum(x.compiles and bool(x.conditions) for x in verdicts),
            "refused": sum(not x.compiles for x in verdicts),
            "first_refusals": dict(sorted(first.items(), key=lambda kv: -kv[1]))}
