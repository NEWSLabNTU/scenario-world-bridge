"""World knobs per run (093 D7 gap 1): a scenario's bound knobs with the run's values, the
world's declarations, what a run sends (baseline merged, absolute), CARLA's refusals of
friction and surface maps, the read-back compared, and get_weather's answer parsed."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scensim_sampler"))

from scensim import knobs as K  # noqa: E402

WORLD = {
    "illumination": {"values": {"day": {"weather": {"sun_altitude_angle": 60}},
                                "night": {"weather": {"sun_altitude_angle": -30,
                                                      "street_lights": 100}}},
                     "default": "day"},
    "surface": {"values": {"dry": {"surface_map": "dry", "mu": 1.0},
                           "wet": {"surface_map": "wet", "mu": 0.7},
                           "wet_look": {"weather": {"wetness": 80}, "physics": "none"}}},
}
SCEN = """<?xml version="1.0"?><OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="3" date="2026-11-21T00:00:00" author="t" description="t">
    <Properties>
      <Property name="scensim.knob.illumination" value="$light"/>
      <Property name="scensim.knob.surface" value="$road"/>
    </Properties></FileHeader>
  <ParameterDeclarations>
    <ParameterDeclaration name="light" parameterType="string" value="day"/>
    <ParameterDeclaration name="road" parameterType="string" value="dry"/>
  </ParameterDeclarations><Entities/><Storyboard/></OpenSCENARIO>"""


def test_the_bound_knobs_take_the_runs_values_else_the_defaults(tmp_path):
    p = tmp_path / "s.xosc"
    p.write_text(SCEN)
    assert K.bound(p, {"light": "night"}) == {"illumination": "night", "surface": "dry"}


def test_the_declarations_are_checked():
    assert K.problems(WORLD) == []
    bad = {"illumination": {"values": {"night": {"weather": {"sun": "low"}}}, "default": "dusk"},
           "fog": {"values": {}}}
    msgs = [m for _, m in K.problems(bad)]
    assert any("parameter: number" in m for m in msgs)
    assert any("'dusk' is not one of its values" in m for m in msgs)
    assert any("needs values" in m for m in msgs)


def test_a_run_sends_the_baseline_merged_with_its_knobs():
    p = K.plan(WORLD, {"illumination": "night", "surface": "wet_look"}, "carla",
               baseline={"cloudiness": 10.0, "sun_altitude_angle": 45.0})
    assert p["weather"] == {"cloudiness": 10.0, "sun_altitude_angle": -30.0,
                            "street_lights": 100.0, "wetness": 80.0}


def test_isaac_takes_surface_maps_and_refuses_weather():
    p = K.plan(WORLD, {"surface": "wet"}, "isaac")
    assert p["surface_maps"] == {"surface": "wet"} and p["weather"] == {}
    with pytest.raises(K.KnobError, match="Isaac has no weather service"):
        K.plan(WORLD, {"illumination": "night"}, "isaac")


def test_carla_refuses_friction_and_surface_maps_but_runs_a_look():
    with pytest.raises(K.KnobError, match="a surface map, which CARLA cannot apply"):
        K.plan(WORLD, {"surface": "wet"}, "carla")
    world = {"surface": {"values": {"wet": {"mu": 0.7, "weather": {"wetness": 80}}}}}
    with pytest.raises(K.KnobError, match="weather does not change"):
        K.plan(world, {"surface": "wet"}, "carla")
    p = K.plan(WORLD, {"surface": "wet_look"}, "carla")
    assert p["weather"] == {"wetness": 80.0}
    assert p["notes"] == {"surface": "visual only: the friction was not applied"}


def test_a_knob_or_value_the_world_lacks_is_refused():
    with pytest.raises(K.KnobError, match="declares no such knob"):
        K.plan(WORLD, {"fog": "thick"}, "carla")
    with pytest.raises(K.KnobError, match="not a value the world declares"):
        K.plan(WORLD, {"illumination": "dusk"}, "carla")


def test_the_read_back_decides():
    knobs = {"illumination": "night", "surface": "wet"}
    p = {"weather": {"sun_altitude_angle": -30.0, "street_lights": 100.0}}   # as on a world
                                                                              # with both
    good = K.compare(knobs, WORLD, p["weather"],
                     {"sun_altitude_angle": -30.0004, "street_lights": 100.0})
    assert good["ok"] and good["knobs"]["illumination"]["ok"]
    assert good["knobs"]["surface"]["surface_map"] == {"sent": "wet", "read": "unverified"}
    bad = K.compare(knobs, WORLD, p["weather"], {"sun_altitude_angle": 60.0,
                                                 "street_lights": 100.0})
    assert not bad["ok"] and bad["knobs"]["illumination"]["applied"]["sun_altitude_angle"] == 60.0
    missing = K.compare({"illumination": "night"}, WORLD, p["weather"], None)
    assert not missing["ok"]


def test_get_weathers_answer_is_parsed():
    text = ("response:\nscensim_interfaces.srv.GetWeather_Response(success=True, message='', "
            "preset='', names=['cloudiness', 'sun_altitude_angle'], values=[10.0, -30.0])")
    assert K.weather_read(text) == {"cloudiness": 10.0, "sun_altitude_angle": -30.0}
    assert K.weather_read("waiting for service") is None


# --- through the runner ---------------------------------------------------------------------

import io  # noqa: E402
import json  # noqa: E402
import signal  # noqa: E402

import yaml  # noqa: E402

from scensim import results, run  # noqa: E402
from test_run import fake, start_stack, where  # noqa: E402,F401 - the fixture
from test_sampled import SEEDED, runs_dir, sampled_project  # noqa: E402

KNOBBED = SEEDED + """world:
  knobs:
    illumination:
      values:
        day: {weather: {sun_altitude_angle: 60}}
        night: {weather: {sun_altitude_angle: -30, street_lights: 100}}
      default: day
    surface:
      values:
        dry: {weather: {wetness: 0}, physics: none}
        wet_look: {weather: {wetness: 80}, physics: none}
"""
LIGHTS = """
      <StochasticDistribution parameterName="light">
        <ProbabilityDistributionSet><Element value="day" weight="1"/><Element value="night" weight="1"/></ProbabilityDistributionSet>
      </StochasticDistribution>
      <StochasticDistribution parameterName="road">
        <ProbabilityDistributionSet><Element value="dry" weight="1"/><Element value="wet_look" weight="1"/></ProbabilityDistributionSet>
      </StochasticDistribution>"""


def knobbed_project(where):
    d = sampled_project(where, testbench=KNOBBED)
    s = (d / "scenarios" / "sampled.xosc").read_text()
    s = s.replace('<Property name="scensim.cell_attributes" value="surface"/>'
                  '<Property name="scensim.cell.surface" value="dry"/>',
                  '<Property name="scensim.cell_attributes" value="illumination,surface"/>'
                  '<Property name="scensim.knob.illumination" value="$light"/>'
                  '<Property name="scensim.knob.surface" value="$road"/>', 1)
    s = s.replace('<ParameterDeclaration name="gap"',
                  '<ParameterDeclaration name="light" parameterType="string" value="day"/>'
                  '<ParameterDeclaration name="road" parameterType="string" value="dry"/>'
                  '<ParameterDeclaration name="gap"', 1)
    assert "scensim.knob.surface" in s
    (d / "scenarios" / "sampled.xosc").write_text(s)
    dist = (d / "scenarios" / "sampled_dist.xosc").read_text()
    (d / "scenarios" / "sampled_dist.xosc").write_text(
        dist.replace("</Stochastic>", LIGHTS + "\n    </Stochastic>", 1))
    return d


class Bridge:
    """The bridge's set_weather and get_weather: it keeps what it is sent, and `drop` names a
    parameter it then reports wrong (a world that did not apply it)."""

    def __init__(self, drop=None):
        self.weather, self.sent, self.drop = {}, [], drop

    def __call__(self, rec, argv):
        if "set_weather" in argv[3]:
            req = argv[-1]
            names = [n.strip() for n in req.split("names: [")[1].split("]")[0].split(",")]
            values = [float(v) for v in req.split("values: [")[1].split("]")[0].split(",")]
            self.sent.append(dict(zip(names, values)))
            self.weather.update(zip(names, values))
            return "response:\nSetWeather_Response(success=True, message='')", ""
        w = dict(self.weather)
        if self.drop in w:
            w[self.drop] = 12.5
        return ("response:\nGetWeather_Response(success=True, message='', preset='', names=["
                + ", ".join(f"'{k}'" for k in w) + "], values=["
                + ", ".join(repr(v) for v in w.values()) + "])"), ""


@pytest.fixture
def knob_stack(where, maps_hub, monkeypatch):
    d = knobbed_project(where)
    monkeypatch.setattr(run, "scenario_argv", fake(where))
    monkeypatch.setattr(run, "pilot_argv", lambda *a: [sys.executable, "-c", "pass"])
    p, proc = start_stack(d)
    yield d, monkeypatch
    proc.send_signal(signal.SIGTERM)
    proc.wait(timeout=10)


def _use(monkeypatch, bridge):
    real = run.apply_knobs
    monkeypatch.setattr(run, "apply_knobs",
                        lambda rec, spec, ps, call=None: real(rec, spec, ps, call=bridge))


def _suite(d, n):
    (d / "suites" / "k.yaml").write_text(yaml.safe_dump({
        "sample": {"distribution": "../scenarios/sampled_dist.xosc", "n": n, "seed": 5},
        "logging": {"keep": {}}}))
    return d / "suites" / "k.yaml"


def test_sampled_night_and_wet_reach_the_world_and_the_record(knob_stack, where):
    d, mp = knob_stack
    bridge = Bridge()
    _use(mp, bridge)
    out = io.StringIO()
    assert run.main([str(_suite(d, 6)), "--project", str(d)], out=out) == 0, out.getvalue()
    seen = set()
    for rd in sorted(runs_dir(where).iterdir()):
        r = results.RunResult.read(rd / "result.json")
        rec = json.loads((rd / "sample.json").read_text())
        light, road = rec["values"]["light"], rec["values"]["road"]
        assert rec["cell"] == {"illumination": light, "surface": road}
        assert r.knobs["ok"] and r.knobs["knobs"]["illumination"]["value"] == light
        want = -30.0 if light == "night" else 60.0
        assert r.knobs["knobs"]["illumination"]["applied"]["sun_altitude_angle"] == want
        assert r.knobs["sent"]["wetness"] == (80.0 if road == "wet_look" else 0.0)
        assert r.knobs["knobs"]["surface"]["physics"].startswith("none")
        seen.add((light, road))
    assert ("night", "wet_look") in seen or len(seen) >= 2
    # absolute: every run sent every knob's parameters, so none inherits another's weather
    assert all({"sun_altitude_angle", "wetness"} <= set(s) for s in bridge.sent)
    # K7 groups the runs by the knobs the world applied
    from scensim_sampler import coverage
    rep = coverage.report(coverage.outcomes_from_runs([runs_dir(where)]))
    cells = {tuple(sorted(c["cell"].items())) for c in rep["layer3"]}
    assert cells == {(("illumination", l), ("surface", r)) for l, r in seen}


def test_a_world_that_did_not_apply_a_knob_gives_an_error_run(knob_stack, where):
    d, mp = knob_stack
    _use(mp, Bridge(drop="sun_altitude_angle"))
    out = io.StringIO()
    run.main([str(_suite(d, 1)), "--project", str(d)], out=out)
    rd = sorted(runs_dir(where).iterdir())[-1]
    r = results.RunResult.read(rd / "result.json")
    assert r.verdict == "error", (r.verdict, r.reason)
    assert "knobs: the world did not apply illumination" in r.reason
    assert r.knobs["ok"] is False
    assert r.knobs["knobs"]["illumination"]["applied"]["sun_altitude_angle"] == 12.5
