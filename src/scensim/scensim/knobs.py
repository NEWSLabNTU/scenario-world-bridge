"""World knobs per run (093 D7 gap 1; scenario-description.md section 3): a scenario's bound
knobs (FileHeader `scensim.knob.<knob> = $<parameter>`) take their run's parameter values,
which go to the world before the scenario starts; the world's answer is read back and
`result.json` records it under `knobs`. A run whose applied values differ from its record's is
an error run, excluded from rates.

What a knob's value means is the world's: `knobs:` under the world in testbench.yaml (a subset
of section 3's world.yaml, which may carry it later)::

    world:
      knobs:
        illumination:
          values:
            day:   {weather: {sun_altitude_angle: 60}}
            night: {weather: {sun_altitude_angle: -30, street_lights: 100}}
          default: day
        surface:
          values:
            dry:      {surface_map: dry, mu: 1.0}
            wet:      {surface_map: wet, mu: 0.7}
            wet_look: {weather: {wetness: 80}, physics: none}   # visual only

A value maps to weather parameters (the world server's names, through the bridge's
set_weather) and/or a named surface map (Isaac: the scenario's RoadCondition
`scensim.surface_map`, linked by the project's ground). Every run sends the testbench's
weather baseline merged with its knobs' parameters, so no run inherits another's weather.

On Isaac there is no weather service: a value with weather parameters is refused there by name
(its scenario's EnvironmentAction sets the sky). On CARLA a surface map cannot be applied, and weather (wetness) is visual only: a value that
names a surface map or a friction (`mu`) is refused there by name, unless it says
`physics: none`, and then the record says the friction was not applied.
"""
from pathlib import Path

KINDS = ("weather", "surface_map", "mu", "physics")
TOLERANCE = 1e-3   # a read-back may round (the world server stores floats)


class KnobError(ValueError):
    pass


def bound(scenario: Path, parameters: dict) -> dict:
    """{knob: value}: each bound knob's parameter, the run's value or its declared default."""
    from scensim_sampler.xosc import Scenario
    sc = Scenario(scenario)
    out = {}
    for knob, param in sc.knobs.items():
        v = (parameters or {}).get(param, sc.parameters[param][1])
        out[knob] = str(v)
    return out


WEATHER_ELEMENTS = ("Weather", "TimeOfDay", "Sun", "Fog", "Precipitation", "Wind")


def environment_weather(scenario: Path) -> list:
    """The weather a scenario's EnvironmentActions set during the run (the bridge forwards
    Weather and TimeOfDay to the world): the element names found, empty when none."""
    from scensim_sampler.subset import read
    root, _ = read(scenario)
    found = []
    for env in root.iter("EnvironmentAction"):
        for tag in WEATHER_ELEMENTS:
            if env.find(f".//{tag}") is not None and tag not in found:
                found.append(tag)
    return found


def problems(world_knobs: dict, where="world.knobs") -> list:
    """The testbench's knob declarations, checked: (where, message) pairs."""
    out = []
    if world_knobs is None:
        return out
    if not isinstance(world_knobs, dict):
        return [(where, "is {knob: {values: {value: {weather?, surface_map?, mu?, physics?}}, "
                        "default?}}")]
    for k, d in world_knobs.items():
        w = f"{where}.{k}"
        if not isinstance(d, dict) or not isinstance(d.get("values"), dict) or not d["values"]:
            out.append((w, "needs values: {value: {...}}"))
            continue
        for v, m in d["values"].items():
            if m is None:
                continue
            if not isinstance(m, dict) or set(m) - set(KINDS):
                out.append((f"{w}.values.{v}", f"is a mapping of {', '.join(KINDS)}"))
                continue
            wp = m.get("weather")
            if wp is not None and (not isinstance(wp, dict) or not all(
                    isinstance(x, (int, float)) and not isinstance(x, bool) for x in wp.values())):
                out.append((f"{w}.values.{v}.weather", "is {parameter: number}"))
            if m.get("physics") not in (None, "none"):
                out.append((f"{w}.values.{v}.physics", "is `none` (visual only) or absent"))
        if d.get("default") is not None and str(d["default"]) not in map(str, d["values"]):
            out.append((f"{w}.default", f"{d['default']!r} is not one of its values"))
    return out


def plan(world_knobs: dict, knobs: dict, simulator: str, baseline: dict = None) -> dict:
    """What to send for a run's knob values: {weather: {param: value}, surface_maps: {knob:
    name}, notes: {knob: text}}. KnobError names a knob the world lacks, a value it does not
    declare, or one this simulator cannot honour."""
    weather = dict(baseline or {})
    maps, notes = {}, {}
    for knob, value in knobs.items():
        decl = (world_knobs or {}).get(knob)
        if decl is None:
            raise KnobError(f"knob {knob!r}: the world declares no such knob (testbench.yaml "
                            f"world.knobs: {', '.join(sorted(world_knobs or {})) or 'none'})")
        values = {str(k): v for k, v in decl["values"].items()}
        if value not in values:
            raise KnobError(f"knob {knob}={value!r}: not a value the world declares "
                            f"({', '.join(values)})")
        m = values[value] or {}
        visual = m.get("physics") == "none"
        if simulator == "carla":
            if m.get("surface_map"):
                raise KnobError(f"knob {knob}={value}: a surface map, which CARLA cannot apply "
                                "per run; declare a visual-only value (physics: none) for CARLA")
            if m.get("mu") is not None and not visual:
                raise KnobError(f"knob {knob}={value}: a friction (mu {m['mu']}), which CARLA's "
                                "weather does not change (wetness is visual); declare it "
                                "physics: none to run it as a look only")
        if simulator == "isaac" and m.get("weather"):
            raise KnobError(f"knob {knob}={value}: weather parameters, but Isaac has no weather "
                            "service (a scenario's EnvironmentAction sets its sky, fog and rain); "
                            "declare this value without weather for Isaac")
        if visual and m.get("mu") is None:
            notes[knob] = "visual only: the friction was not applied"
        weather.update({p: float(x) for p, x in (m.get("weather") or {}).items()})
        if m.get("surface_map"):
            maps[knob] = str(m["surface_map"])
    return {"weather": weather, "surface_maps": maps, "notes": notes}


def compare(knobs: dict, world_knobs: dict, sent: dict, read: dict, maps_read=None) -> dict:
    """result.json's `knobs`: per knob {value, applied: {param: read-back}, ok}, and `ok` over
    all. A weather parameter whose read-back differs from what was sent (beyond TOLERANCE) is
    not ok; a surface map with no read-back is `unverified`."""
    out, all_ok = {}, True
    for knob, value in knobs.items():
        m = ((world_knobs or {}).get(knob) or {}).get("values", {})
        m = {str(k): v for k, v in m.items()}.get(value) or {}
        applied, ok = {}, True
        for p in (m.get("weather") or {}):
            got = (read or {}).get(p)
            applied[p] = got
            if got is None or abs(float(got) - float(sent[p])) > TOLERANCE:
                ok = False
        entry = {"value": value, "applied": applied}
        if m.get("surface_map"):
            got = (maps_read or {}).get(knob)
            entry["surface_map"] = {"sent": m["surface_map"],
                                    "read": got if got is not None else "unverified"}
            if got is not None and got != m["surface_map"]:
                ok = False
        if m.get("physics") == "none":
            entry["physics"] = "none: the friction was not applied"
        entry["ok"] = ok
        all_ok = all_ok and ok
        out[knob] = entry
    return {"ok": all_ok, "knobs": out}


GET_WEATHER = "/scenario_world_bridge/get_weather"


def get_weather_argv() -> list:
    return ["ros2", "service", "call", GET_WEATHER, "scensim_interfaces/srv/GetWeather"]


def weather_read(text: str):
    """{name: value} from get_weather's printed response, or None when it did not answer."""
    import re
    if not re.search(r"success=True", text or ""):
        return None
    names = re.search(r"names=\[(.*?)\]", text or "", re.S)
    values = re.search(r"values=\[(.*?)\]", text or "", re.S)
    if not names or not values:
        return {}
    ns = [n.strip().strip("'\"") for n in names.group(1).split(",") if n.strip()]
    vs = [float(v) for v in values.group(1).split(",") if v.strip()]
    return dict(zip(ns, vs))
