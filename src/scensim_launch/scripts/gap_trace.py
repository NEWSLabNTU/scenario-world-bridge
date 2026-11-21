"""Reader for the bridge's gap trace, the run's truth file (roadmap 024; schema 2 in
docs/design/truth-stream.md, schema 1 in docs/design/gap-campaign.md, appendix "Gap trace
format").

Library module, not an executable: `gap_oracle` and the tests import it from this
directory. Pure Python, stdlib only.

    trace = load_trace("20261013T101500Z_3.jsonl")
    for frame in trace.frames:
        ego = frame.entities["ego"]

`iter_records` streams typed records and validates them as it goes; `load_trace` collects
them into a `Trace`. Any malformed line raises `TraceError`, which names the source, the
line number and the field. `to_json` / `dumps` write records back in the same format.

Each file is read by its schema (the `episode` line's): schema 1 named the simulator's values
after CARLA (`carla`, `carla_frame`, `actor_id`, `blueprint`, `blueprint_box`), schema 2 names
them neutrally (`sim_pose`, `sim_frame`, `sim_id`, `asset`, `asset_box`). The records here use
schema 2's names for both; schema 2's additions are None in a schema 1 trace (an entity's
`index` is its header's position).
"""

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

SCHEMA = 2
SCHEMAS = (1, 2)
# The file's key for each value, per schema.
KEYS = {
    1: {"sim_pose": "carla", "sim_frame": "carla_frame", "sim_id": "actor_id",
        "asset": "blueprint", "asset_box": "blueprint_box"},
    2: {"sim_pose": "sim_pose", "sim_frame": "sim_frame", "sim_id": "sim_id",
        "asset": "asset", "asset_box": "asset_box"},
}
ROLES = ("ego", "scenario", "autopilot", "agent")
VELOCITY_SOURCES = ("measured", "derived")

ENTITY_KINDS = ("ego", "vehicle", "pedestrian", "misc_object")
CONTACT_KINDS = ("ego", "vehicle", "pedestrian", "misc_object", "static", "unknown")
NPC_MOTIONS = ("tracked", "kinematic")
# `kinematic` added under schema 1 (roadmap 032): a reader of the two-flag schema rejects it,
# but every trace written before it still reads here.
FLAGS = ("spawn", "contact_hold", "kinematic")


class TraceError(ValueError):
    """A malformed trace line. `line` is 1-based; `field` is a dotted path or None."""

    def __init__(self, source, line, field, message):
        self.source, self.line, self.field = source, line, field
        where = f"{source} line {line}"
        super().__init__(f"{where}: {field}: {message}" if field else f"{where}: {message}")


# --- records ------------------------------------------------------------------------------


@dataclass(frozen=True)
class Episode:
    schema: int
    episode: int
    start_unix: float
    npc_motion: str
    map: object = None  # schema 2: the map's name
    step_time: object = None  # schema 2: s


@dataclass(frozen=True)
class DeclaredBox:
    center: tuple  # (x, y, z) m, entity frame
    dimensions: tuple  # (length, width, height) m


@dataclass(frozen=True)
class AssetBox:
    location: tuple  # (x, y, z) m, body frame (ROS-handed)
    extent: tuple  # half sizes (x, y, z) m, body frame


@dataclass(frozen=True)
class OriginOffset:
    x: float
    y: float
    yaw: float  # rad


@dataclass(frozen=True)
class EntityHeader:
    frame: int
    name: str
    kind: str
    declared: DeclaredBox
    origin_offset: OriginOffset
    sim_id: int
    asset: str
    asset_box: object  # AssetBox, or None when the simulator's box could not be recovered
    index: int = 0  # the entity's slot in the episode (schema 1: the header's position)
    role: object = None  # schema 2: one of ROLES
    subtype: object = None  # schema 2: SSv2's subtype


@dataclass(frozen=True)
class Pose6:
    """ROS map frame: m and rad."""

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class EntityState:
    ssv2: Pose6
    sim_pose: object  # Pose6, or None when the body was not in the simulator's snapshot
    velocity: object  # (vx, vy, vz) m/s, or None with `sim_pose`
    flags: tuple = ()
    angular_velocity: object = None  # schema 2: (wx, wy, wz) rad/s, or None with `sim_pose`
    velocity_source: object = None  # schema 2: one of VELOCITY_SOURCES, or None

    @property
    def spawn(self):
        return "spawn" in self.flags

    @property
    def contact_hold(self):
        return "contact_hold" in self.flags

    @property
    def kinematic(self):
        """A tracked NPC handed to kinematic after a blocked snap (roadmap 032)."""
        return "kinematic" in self.flags


@dataclass(frozen=True)
class Frame:
    frame: int
    sim_time: float
    entities: dict  # name -> EntityState
    # Simulator frame of the snapshot the simulator's half was read from (the frame's last
    # tick; roadmap 032 N2): absent in older traces.
    sim_frame: object = None
    sim_time_ns: object = None  # schema 2, when known


@dataclass(frozen=True)
class Contact:
    frame: int
    sim_frame: int
    sim_time: float
    ego_id: int
    other_id: object  # int, or None for the static world
    name: object  # SSv2 name of the other entity, or None
    other_type: str
    other_kind: str
    impulse: float  # N*s
    # SSv2 name of the tracked NPC whose own collision sensor saw this contact; None for the
    # ego's sensor. Added under schema 1 (roadmap 032 step 5): absent in older traces.
    subject: object = None

    @property
    def ego_side(self):
        """Seen by the ego's sensor (the contacts 021's report and G4 are about)."""
        return self.subject is None


@dataclass(frozen=True)
class Signal:
    """Schema 2: a signal's colour changed (the bridge set it). Events, not repeated per
    frame: a reader keeps the last colour per signal."""

    frame: int
    sim_time_ns: int
    opendrive_id: str
    colour: str


@dataclass(frozen=True)
class End:
    frame: int
    sim_time: float
    frames: int
    dropped_frames: int
    complete: bool


@dataclass
class Trace:
    episode: Episode
    entities: dict = field(default_factory=dict)  # name -> latest EntityHeader
    frames: list = field(default_factory=list)
    contacts: list = field(default_factory=list)
    end: object = None  # End, or None for a trace cut short (require_end=False)
    headers: list = field(default_factory=list)  # every EntityHeader, in order
    signals: list = field(default_factory=list)  # Signal, in order (schema 2)

    def records(self):
        """All records in an order that reads back to the same trace."""
        yield self.episode
        headers = list(self.headers)
        contacts = list(self.contacts)
        signals = list(self.signals)
        for frame in self.frames:
            while headers and headers[0].frame <= frame.frame:
                yield headers.pop(0)
            while contacts and contacts[0].frame < frame.frame:
                yield contacts.pop(0)
            while signals and signals[0].frame < frame.frame:
                yield signals.pop(0)
            yield frame
        yield from headers
        yield from contacts
        yield from signals
        if self.end is not None:
            yield self.end


# --- field checks -------------------------------------------------------------------------


class _Line:
    """One parsed line, with checks that raise TraceError naming it."""

    def __init__(self, source, number, obj, schema=SCHEMA):
        self.source, self.number, self.obj = source, number, obj
        self.keys = KEYS[schema]
        self.schema = schema

    def error(self, field, message):
        return TraceError(self.source, self.number, field, message)

    def get(self, obj, key, path):
        if not isinstance(obj, dict) or key not in obj:
            raise self.error(path, "missing")
        return obj[key]

    def number_(self, value, path, minimum=None):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise self.error(path, f"expected a number, got {value!r}")
        if not math.isfinite(value):
            raise self.error(path, f"not finite: {value!r}")
        if minimum is not None and value < minimum:
            raise self.error(path, f"must be >= {minimum}, got {value!r}")
        return float(value)

    def integer(self, value, path, minimum=None):
        if isinstance(value, bool) or not isinstance(value, int):
            raise self.error(path, f"expected an integer, got {value!r}")
        if minimum is not None and value < minimum:
            raise self.error(path, f"must be >= {minimum}, got {value!r}")
        return value

    def string(self, value, path, choices=None):
        if not isinstance(value, str):
            raise self.error(path, f"expected a string, got {value!r}")
        if choices is not None and value not in choices:
            raise self.error(path, f"{value!r} is not one of {', '.join(choices)}")
        return value

    def boolean(self, value, path):
        if not isinstance(value, bool):
            raise self.error(path, f"expected true or false, got {value!r}")
        return value

    def vector(self, value, path, length, minimum=None):
        if not isinstance(value, list) or len(value) != length:
            raise self.error(path, f"expected {length} numbers, got {value!r}")
        return tuple(self.number_(v, f"{path}[{i}]", minimum) for i, v in enumerate(value))

    def num(self, obj, key, path=None, minimum=None):
        path = path or key
        return self.number_(self.get(obj, key, path), path, minimum)

    def int_(self, obj, key, path=None, minimum=None):
        path = path or key
        return self.integer(self.get(obj, key, path), path, minimum)

    def str_(self, obj, key, path=None, choices=None):
        path = path or key
        return self.string(self.get(obj, key, path), path, choices)

    def vec(self, obj, key, path, length, minimum=None):
        return self.vector(self.get(obj, key, path), path, length, minimum)


# --- parsing ------------------------------------------------------------------------------


def _episode(ln):
    o = ln.obj
    schema = ln.int_(o, "schema")
    if schema not in SCHEMAS:
        raise ln.error("schema", f"unsupported schema {schema} (this reader reads "
                                 f"{', '.join(map(str, SCHEMAS))})")
    two = schema >= 2
    return Episode(
        schema=schema,
        episode=ln.int_(o, "episode", minimum=0),
        start_unix=ln.num(o, "start_unix"),
        npc_motion=ln.str_(o, "npc_motion", choices=NPC_MOTIONS),
        map=ln.str_(o, "map") if two else None,
        step_time=ln.num(o, "step_time", minimum=0.0) if two else None,
    )


def _entity(ln, position):
    o, k = ln.obj, ln.keys
    two = ln.schema >= 2
    declared = ln.get(o, "declared", "declared")
    box_key = k["asset_box"]
    bp = ln.get(o, box_key, box_key)
    if bp is not None:
        bp = AssetBox(
            location=ln.vec(bp, "location", f"{box_key}.location", 3),
            extent=ln.vec(bp, "extent", f"{box_key}.extent", 3, minimum=0.0),
        )
    offset = ln.vec(o, "origin_offset", "origin_offset", 3)
    name = ln.str_(o, "name")
    if not name:
        raise ln.error("name", "empty")
    return EntityHeader(
        frame=ln.int_(o, "frame", minimum=0),
        name=name,
        kind=ln.str_(o, "kind", choices=ENTITY_KINDS),
        declared=DeclaredBox(
            center=ln.vec(declared, "center", "declared.center", 3),
            dimensions=ln.vec(declared, "dimensions", "declared.dimensions", 3, minimum=0.0),
        ),
        origin_offset=OriginOffset(*offset),
        sim_id=ln.int_(o, k["sim_id"], minimum=0),
        asset=ln.str_(o, k["asset"]),
        asset_box=bp,
        index=ln.int_(o, "index", minimum=0) if two else position,
        role=ln.str_(o, "role", choices=ROLES) if two else None,
        subtype=ln.str_(o, "subtype") if two else None,
    )


def _state(ln, name, s):
    path = f"entities.{name}"
    if not isinstance(s, dict):
        raise ln.error(path, f"expected an object, got {s!r}")
    ssv2 = Pose6(*ln.vec(s, "ssv2", f"{path}.ssv2", 6))
    pk = ln.keys["sim_pose"]
    sim_pose = ln.get(s, pk, f"{path}.{pk}")
    velocity = ln.get(s, "velocity", f"{path}.velocity")
    angular = source = None
    two = ln.schema >= 2
    if two:
        angular = ln.get(s, "angular_velocity", f"{path}.angular_velocity")
        source = ln.get(s, "velocity_source", f"{path}.velocity_source")
    if sim_pose is None:
        for key, v in (("velocity", velocity), ("angular_velocity", angular),
                       ("velocity_source", source)):
            if v is not None:
                raise ln.error(f"{path}.{key}", f"must be null when {pk} is null")
    else:
        sim_pose = Pose6(*ln.vector(sim_pose, f"{path}.{pk}", 6))
        if velocity is None:
            raise ln.error(f"{path}.velocity", f"missing while {pk} is set")
        velocity = ln.vector(velocity, f"{path}.velocity", 3)
        if two:
            angular = ln.vector(angular, f"{path}.angular_velocity", 3)
            source = ln.string(source, f"{path}.velocity_source", VELOCITY_SOURCES)
    flags = s.get("flags", [])
    if not isinstance(flags, list):
        raise ln.error(f"{path}.flags", f"expected a list, got {flags!r}")
    for f in flags:
        if f not in FLAGS:
            raise ln.error(f"{path}.flags", f"unknown flag {f!r} (known: {', '.join(FLAGS)})")
    return EntityState(ssv2=ssv2, sim_pose=sim_pose, velocity=velocity, flags=tuple(flags),
                       angular_velocity=angular, velocity_source=source)


def _frame(ln, known):
    o = ln.obj
    frame = ln.int_(o, "frame", minimum=0)
    sim_time = ln.num(o, "sim_time", minimum=0.0)
    entities = ln.get(o, "entities", "entities")
    if not isinstance(entities, dict):
        raise ln.error("entities", f"expected an object, got {entities!r}")
    states = {}
    for name, s in entities.items():
        if name not in known:
            raise ln.error(f"entities.{name}", "no entity record declares this name")
        states[name] = _state(ln, name, s)
    fk = ln.keys["sim_frame"]
    sim_frame = o.get(fk)
    if sim_frame is not None:
        sim_frame = ln.int_(o, fk, minimum=0)
    sim_time_ns = o.get("sim_time_ns") if ln.schema >= 2 else None
    if sim_time_ns is not None:
        sim_time_ns = ln.int_(o, "sim_time_ns", minimum=0)
    return Frame(frame=frame, sim_time=sim_time, entities=states, sim_frame=sim_frame,
                 sim_time_ns=sim_time_ns)


def _contact(ln):
    o = ln.obj
    kind = ln.str_(o, "other_kind", choices=CONTACT_KINDS)
    other_id = ln.get(o, "other_id", "other_id")
    if kind == "static":
        if other_id is not None:
            raise ln.error("other_id", f"must be null for the static world, got {other_id!r}")
    else:
        other_id = ln.integer(other_id, "other_id", minimum=0)
    name = ln.get(o, "name", "name")
    if name is not None:
        name = ln.string(name, "name")
    subject = o.get("subject")
    if subject is not None:
        subject = ln.string(subject, "subject")
        if not subject:
            raise ln.error("subject", "empty")
    return Contact(
        frame=ln.int_(o, "frame", minimum=0),
        sim_frame=ln.int_(o, ln.keys["sim_frame"], minimum=0),
        sim_time=ln.num(o, "sim_time", minimum=0.0),
        ego_id=ln.int_(o, "ego_id", minimum=0),
        other_id=other_id,
        name=name,
        other_type=ln.str_(o, "other_type"),
        other_kind=kind,
        impulse=ln.num(o, "impulse", minimum=0.0),
        subject=subject,
    )


def _signal(ln):
    o = ln.obj
    return Signal(
        frame=ln.int_(o, "frame", minimum=0),
        sim_time_ns=ln.int_(o, "sim_time_ns", minimum=0),
        opendrive_id=ln.str_(o, "opendrive_id"),
        colour=ln.str_(o, "colour"),
    )


def _end(ln):
    o = ln.obj
    return End(
        frame=ln.int_(o, "frame", minimum=0),
        sim_time=ln.num(o, "sim_time", minimum=0.0),
        frames=ln.int_(o, "frames", minimum=0),
        dropped_frames=ln.int_(o, "dropped_frames", minimum=0),
        complete=ln.boolean(ln.get(o, "complete", "complete"), "complete"),
    )


def _lines(source):
    """(name, iterable of lines) for a path or an iterable of strings."""
    if isinstance(source, (str, Path)):
        path = Path(source)
        return path.name, path.read_text().splitlines()
    return "<trace>", source


def iter_records(source, require_end=True):
    """Typed records of a trace, validated in order. `source` is a path or lines of text."""
    name, text = _lines(source)
    known = set()
    schema = SCHEMA
    headers = 0
    last_frame = None
    seen_episode = seen_end = False
    number = 0
    for number, raw in enumerate(text, start=1):
        if not raw.strip():
            continue
        try:
            obj = json.loads(raw)  # NaN/Infinity parse; the field checks reject them
        except ValueError as e:
            raise TraceError(name, number, None, f"not JSON: {e}") from None
        if not isinstance(obj, dict):
            raise TraceError(name, number, None, f"expected a JSON object, got {raw[:40]!r}")
        ln = _Line(name, number, obj, schema)
        kind = ln.str_(obj, "type")
        if seen_end:
            raise ln.error("type", f"{kind!r} record after the end record")
        if not seen_episode and kind != "episode":
            raise ln.error("type", f"the first record must be 'episode', got {kind!r}")
        if kind == "episode":
            if seen_episode:
                raise ln.error("type", "a second episode record (one episode per file)")
            seen_episode = True
            episode = _episode(ln)
            schema = episode.schema
            yield episode
        elif kind == "entity":
            header = _entity(ln, headers)
            headers += 1
            known.add(header.name)
            yield header
        elif kind == "frame":
            frame = _frame(ln, known)
            if last_frame is not None and frame.frame <= last_frame:
                raise ln.error("frame", f"{frame.frame} does not follow frame {last_frame}")
            last_frame = frame.frame
            yield frame
        elif kind == "contact":
            yield _contact(ln)
        elif kind == "signal" and schema >= 2:
            yield _signal(ln)
        elif kind == "end":
            seen_end = True
            yield _end(ln)
        else:
            raise ln.error(
                "type", f"unknown record type {kind!r} (episode, entity, frame, contact, "
                        f"{'signal, ' if schema >= 2 else ''}end)"
            )
    if not seen_episode:
        raise TraceError(name, number + 1, "type", "no episode record: the trace is empty")
    if require_end and not seen_end:
        raise TraceError(name, number + 1, "type", "no end record: the trace was cut short")


def load_trace(source, require_end=True):
    """The whole trace as a `Trace`. See `iter_records`."""
    trace = None
    for r in iter_records(source, require_end=require_end):
        if isinstance(r, Episode):
            trace = Trace(episode=r)
        elif isinstance(r, EntityHeader):
            trace.entities[r.name] = r
            trace.headers.append(r)
        elif isinstance(r, Frame):
            trace.frames.append(r)
        elif isinstance(r, Contact):
            trace.contacts.append(r)
        elif isinstance(r, Signal):
            trace.signals.append(r)
        elif isinstance(r, End):
            trace.end = r
    return trace


# --- writing ------------------------------------------------------------------------------


def _pose(p):
    return None if p is None else [p.x, p.y, p.z, p.roll, p.pitch, p.yaw]


def to_json(record, schema=SCHEMA):
    """The JSON object of one record, as the bridge writes it in `schema`."""
    k = KEYS[schema]
    two = schema >= 2
    if isinstance(record, Episode):
        out = {
            "type": "episode",
            "schema": record.schema,
            "episode": record.episode,
            "start_unix": record.start_unix,
            "npc_motion": record.npc_motion,
        }
        if two:
            out |= {"map": record.map, "step_time": record.step_time}
        return out
    if isinstance(record, EntityHeader):
        bp = record.asset_box
        return {
            "type": "entity",
            "frame": record.frame,
            "name": record.name,
        } | ({"index": record.index, "role": record.role} if two else {}) | {
            "kind": record.kind,
        } | ({"subtype": record.subtype} if two else {}) | {
            "declared": {
                "center": list(record.declared.center),
                "dimensions": list(record.declared.dimensions),
            },
            "origin_offset": [
                record.origin_offset.x,
                record.origin_offset.y,
                record.origin_offset.yaw,
            ],
            k["sim_id"]: record.sim_id,
            k["asset"]: record.asset,
            k["asset_box"]: None
            if bp is None
            else {"location": list(bp.location), "extent": list(bp.extent)},
        }
    if isinstance(record, Frame):
        entities = {}
        for name, s in record.entities.items():
            e = {
                "ssv2": _pose(s.ssv2),
                k["sim_pose"]: _pose(s.sim_pose),
                "velocity": None if s.velocity is None else list(s.velocity),
            }
            if two:
                e["angular_velocity"] = (None if s.angular_velocity is None
                                         else list(s.angular_velocity))
                e["velocity_source"] = s.velocity_source
            if s.flags:
                e["flags"] = list(s.flags)
            entities[name] = e
        return {
            "type": "frame",
            "frame": record.frame,
            "sim_time": record.sim_time,
        } | ({"sim_time_ns": record.sim_time_ns}
             if two and record.sim_time_ns is not None else {}) | (
            {} if record.sim_frame is None else {k["sim_frame"]: record.sim_frame}) | {
            "entities": entities,
        }
    if isinstance(record, Contact):
        return {
            "type": "contact",
            "frame": record.frame,
            k["sim_frame"]: record.sim_frame,
            "sim_time": record.sim_time,
            "ego_id": record.ego_id,
            "other_id": record.other_id,
            "name": record.name,
            "other_type": record.other_type,
            "other_kind": record.other_kind,
            "impulse": record.impulse,
        } | ({} if record.subject is None else {"subject": record.subject})
    if isinstance(record, Signal):
        return {
            "type": "signal",
            "frame": record.frame,
            "sim_time_ns": record.sim_time_ns,
            "opendrive_id": record.opendrive_id,
            "colour": record.colour,
        }
    if isinstance(record, End):
        return {
            "type": "end",
            "frame": record.frame,
            "sim_time": record.sim_time,
            "frames": record.frames,
            "dropped_frames": record.dropped_frames,
            "complete": record.complete,
        }
    raise TypeError(f"not a trace record: {record!r}")


def dumps(trace):
    """A whole trace as JSON Lines text."""
    schema = trace.episode.schema
    return "".join(json.dumps(to_json(r, schema)) + "\n" for r in trace.records())
