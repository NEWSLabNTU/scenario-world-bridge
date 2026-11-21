"""Sampled runs (roadmap 092 K3): a suite's sample: expanded into one run per record, each run's
record bound to its world and car, --replay of a record, and replay fidelity. The stack is the
test_run harness's (a real keeper, a stub SSv2)."""
import io
import json
import signal
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scensim_sampler"))

from scensim import results, run, runs, sampled  # noqa: E402
from test_run import fake, make, start_stack, where  # noqa: E402,F401 - the fixture

import scensim_sampler  # noqa: E402

SEEDED = """simulator: carla
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01]
pilots:
  ego: {kind: autoware, vehicle: tesla_model3}
  lead: {kind: carla_basic}
  follower: {kind: command, lifetime: run,
             run: [python3, pilots/follower.py, --seed, "{seed}"]}
"""

DIST = """<?xml version="1.0"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="2" date="2026-11-21T00:00:00" author="t" description="t"/>
  <ParameterValueDistribution>
    <ScenarioFile filepath="sampled.xosc"/>
    <Stochastic numberOfTestRuns="3" randomSeed="41">
      <StochasticDistribution parameterName="gap">
        <UniformDistribution><Range lowerLimit="5" upperLimit="15"/></UniformDistribution>
      </StochasticDistribution>
      <StochasticDistribution parameterName="outcome">
        <ProbabilityDistributionSet><Element value="{outcome}" weight="1"/></ProbabilityDistributionSet>
      </StochasticDistribution>
    </Stochastic>
  </ParameterValueDistribution>
</OpenSCENARIO>
"""


def sampled_project(where, outcome="passed", testbench=None):
    d = make(where, testbench)
    src = (d / "scenarios" / "three_pilots.xosc").read_text()
    src = src.replace(
        'revMajor="1" revMinor="2" />',
        'revMajor="1" revMinor="2"><Properties>'
        '<Property name="scensim.category" value="R1.car_following.lead_brakes"/>'
        '<Property name="scensim.cell_attributes" value="surface"/>'
        '<Property name="scensim.cell.surface" value="dry"/></Properties></FileHeader>', 1)
    src = src.replace(
        "<ParameterDeclarations />",
        '<ParameterDeclarations>'
        '<ParameterDeclaration name="gap" parameterType="double" value="10"/>'
        '<ParameterDeclaration name="outcome" parameterType="string" value="passed"/>'
        '<ParameterDeclaration name="scensim_seed" parameterType="unsignedInt" value="0"/>'
        '</ParameterDeclarations>', 1)
    assert "scensim.category" in src and "scensim_seed" in src
    (d / "scenarios" / "sampled.xosc").write_text(src)
    (d / "scenarios" / "sampled_dist.xosc").write_text(DIST.replace("{outcome}", outcome))
    (d / "suites").mkdir(exist_ok=True)
    return d


def suite(d, **sample):
    (d / "suites" / "s.yaml").write_text(yaml.safe_dump({
        "sample": {"distribution": "../scenarios/sampled_dist.xosc", **sample},
        "logging": {"keep": {}}}))
    return d / "suites" / "s.yaml"


# --- expansion ------------------------------------------------------------------------------

def test_a_sample_key_expands_to_one_run_per_record(where, maps_hub):
    d = sampled_project(where)
    specs, _ = run.expand(suite(d, n=4, seed=7), d)
    assert [s.key for s in specs] == [f"sampled_s{i}_r0" for i in range(4)]
    for i, s in enumerate(specs):
        assert s.sample["sampler"]["index"] == i and s.sample["sampler"]["run_seed"] == 7
        assert 5 <= s.parameters["gap"] <= 15 and s.parameters["outcome"] == "passed"
        # the scenario declares scensim_seed: SSv2 gets the record's scenario seed
        assert s.parameters["scensim_seed"] == s.sample["seeds"]["scenario"]
        assert s.scenario == (d / "scenarios" / "sampled.xosc").resolve()
    # drawn again, the same records
    again, _ = run.expand(suite(d, n=4, seed=7), d)
    assert [s.sample["id"] for s in again] == [s.sample["id"] for s in specs]


def test_sample_sits_beside_the_old_keys_and_they_are_unchanged(where, maps_hub):
    d = sampled_project(where)
    (d / "suites" / "both.yaml").write_text(yaml.safe_dump({
        "scenarios": ["../scenarios/three_pilots.xosc"],
        "parameters": {"outcome": ["passed", "failed"]}, "repetitions": 2, "seed": 1000,
        "sample": [{"distribution": "../scenarios/sampled_dist.xosc", "n": 2, "seed": 1},
                   {"distribution": "../scenarios/sampled_dist.xosc", "n": 1, "seed": 2}]}))
    specs, _ = run.expand(d / "suites" / "both.yaml", d)
    old = [s for s in specs if s.sample is None]
    assert [(s.key, s.seed) for s in old] == [
        ("three_pilots_outcome-passed_r0", 1000), ("three_pilots_outcome-passed_r1", 1001),
        ("three_pilots_outcome-failed_r0", 1000), ("three_pilots_outcome-failed_r1", 1001)]
    new = [s for s in specs if s.sample is not None]
    assert [(s.key, s.seed) for s in new] == [("sampled_s0_r0", 1), ("sampled_s1_r0", 1),
                                              ("sampled_s0_r0", 2)]


@pytest.mark.parametrize("sample, words", [
    ({"distribution": "../scenarios/sampled_dist.xosc", "n": 0}, "n: an integer >= 1"),
    ({"distribution": "../scenarios/sampled_dist.xosc", "count": 3}, "unknown key count"),
    ({"distribution": "../scenarios/nope.xosc"}, "distribution ../scenarios/nope.xosc: no such"),
    ("x.xosc", "a mapping {distribution, n, seed"),
])
def test_a_bad_sample_is_refused_by_name(where, maps_hub, sample, words):
    d = sampled_project(where)
    (d / "suites" / "s.yaml").write_text(yaml.safe_dump({"sample": sample}))
    with pytest.raises(run.Refused, match=words):
        run.expand(d / "suites" / "s.yaml", d)


def test_a_scenario_without_a_category_is_refused_by_the_sampler(where, maps_hub):
    d = sampled_project(where)
    p = d / "scenarios" / "sampled.xosc"
    p.write_text(p.read_text().replace("scensim.category", "other.category"))
    with pytest.raises(run.Refused, match="no scensim.category property"):
        run.expand(suite(d, n=1, seed=1), d)


def test_pilot_seed_is_the_records_for_its_entity():
    rec = {"seeds": {"pilots": {"follower": 77}}}
    assert sampled.pilot_seed(rec, "follower", 5) == 77
    assert sampled.pilot_seed(rec, "lead", 5) == 5
    assert sampled.pilot_seed(None, "follower", 5) == 5


def test_world_of_a_carla_town_hashes_its_lanelet2_map(tmp_path):
    (tmp_path / "lanelet2_map.osm").write_text("<osm/>")
    w = sampled.world_of({"simulator": "carla", "map": "Town01", "map_path": str(tmp_path)},
                         {"carla": {"version": "0.9.16"}})
    assert w["name"] == "Town01"
    assert w["hash"].startswith("carla-0.9.16:Town01:lanelet2:sha256:")
    w = sampled.world_of({"simulator": "isaac", "map": "haruna"},
                         {"stage": {"root": "haruna", "sha256": "ab" * 32}})
    assert w == {"name": "haruna", "hash": "stage:sha256:" + "ab" * 32}
    assert sampled.world_of({"simulator": "carla", "map": "Town01"}, {})["hash"].endswith(
        ":unhashed")


# --- runs, replay, fidelity -----------------------------------------------------------------

@pytest.fixture
def stack_up(where, maps_hub, monkeypatch):
    d = sampled_project(where, testbench=SEEDED)
    monkeypatch.setattr(run, "scenario_argv", fake(where))
    seen = where["tmp"] / "seeds.jsonl"

    real = run.pilot_argv

    def pilot_cmd(rec, pilot, entry, log_dir):
        argv = real(rec, pilot, entry, log_dir)
        words = dict(w.split(":=", 1) for w in argv if ":=" in w)
        return [sys.executable, "-c",
                f"import json; open({str(seen)!r}, 'a').write(json.dumps("
                f"{json.dumps(words.get('run', ''))}) + '\\n')"]
    monkeypatch.setattr(run, "pilot_argv", pilot_cmd)
    p, proc = start_stack(d)
    yield d, seen
    proc.send_signal(signal.SIGTERM)
    proc.wait(timeout=10)


def runs_dir(where):
    return where["tmp"] / "logs" / "proj" / "runs"


def test_sampled_runs_carry_their_bound_record_and_replay_exactly(stack_up, where):
    d, seen = stack_up
    out = io.StringIO()
    assert run.main([str(suite(d, n=2, seed=7)), "--project", str(d)], out=out) == 0, \
        out.getvalue()
    dirs = sorted(runs_dir(where).iterdir())
    assert len(dirs) == 2
    for rd in dirs:
        r = results.RunResult.read(rd / "result.json")
        rec = json.loads((rd / "sample.json").read_text())
        assert r.sample["id"] == rec["id"] and r.sample["drawn_id"] != rec["id"]
        assert rec["world"]["name"] == "Town01"
        assert rec["world"]["hash"].startswith("carla")
        # bound (world, and car when the stack names a spec_hash) and valid
        bound = scensim_sampler.check(rec, bound="car" in rec)
        assert bound == [], bound
        if "car" not in rec:
            assert any(n.startswith("car:") for n in r.sample["unbound"])
        assert r.scenario["parameters"]["gap"] == rec["values"]["gap"]
    # the follower (lifetime: run) got each record's seed for it
    pilot_runs = [json.loads(line) for line in seen.read_text().splitlines()]
    recs = [json.loads((rd / "sample.json").read_text()) for rd in dirs]
    want = {str(x["seeds"]["pilots"]["follower"]) for x in recs}
    assert {w.split("--seed ")[1] for w in pilot_runs} == want

    # replay the first run: same values and seeds, the target recorded
    first = dirs[0]
    out = io.StringIO()
    assert run.main(["--replay", str(first), "--project", str(d)], out=out) == 0, out.getvalue()
    replay = [x for x in sorted(runs_dir(where).iterdir()) if x.name.endswith("_replay")]
    assert len(replay) == 1
    rr = results.RunResult.read(replay[0] / "result.json")
    orig = results.RunResult.read(first / "result.json")
    assert rr.replay["of"] == orig.sample["drawn_id"] and rr.replay["of_run"] == first.name
    assert rr.replay["target"]["simulator"] == "carla"
    assert rr.scenario["parameters"] == orig.scenario["parameters"]
    assert json.loads((replay[0] / "sample.json").read_text())["seeds"] == \
        json.loads((first / "sample.json").read_text())["seeds"]
    assert json.loads(seen.read_text().splitlines()[-1]).split("--seed ")[1] == \
        str(recs[0]["seeds"]["pilots"]["follower"])

    # fidelity: the replay reached the original's outcome
    out = io.StringIO()
    assert runs.main(["fidelity", str(runs_dir(where)), "--json",
                      str(where["tmp"] / "f.json")], out=out) == 0
    assert "fidelity: 1 of 1 replays reached the original's outcome (1.00)" in out.getvalue()
    rep = json.loads((where["tmp"] / "f.json").read_text())
    assert rep["pairs"][0]["differs"] == []


def test_a_replay_of_a_changed_scenario_is_refused(stack_up, where, capsys):
    d, _ = stack_up
    assert run.main([str(suite(d, n=1, seed=3)), "--project", str(d)], out=io.StringIO()) == 0
    first = next(runs_dir(where).iterdir())
    p = d / "scenarios" / "sampled.xosc"
    p.write_text(p.read_text() + "\n<!-- edited -->\n")
    assert run.main(["--replay", str(first), "--project", str(d)], out=io.StringIO()) == 3
    assert "has changed since the record was drawn" in capsys.readouterr().err


def test_replay_takes_a_record_file_and_refuses_extra_arguments(where, maps_hub, capsys):
    d = sampled_project(where)
    b = scensim_sampler.Sampler.from_file(d / "scenarios" / "sampled_dist.xosc").draw(n=3)
    f = where["tmp"] / "r.jsonl"
    f.write_text("".join(json.dumps(r) + "\n" for r in b.records))
    with pytest.raises(run.Refused, match="3 records; give --index"):
        run.replay_specs(str(f), None, d)
    (spec,) = run.replay_specs(str(f), 2, d)
    assert spec.key == "sampled_s2_replay" and spec.replay_of == ""
    assert spec.sample["id"] == b.records[2]["id"]
    assert run.main(["--replay", str(f), "x.xosc", "--project", str(d)]) == 3
    assert "--replay runs the record as it is; drop a target" in capsys.readouterr().err
    assert run.main(["--project", str(d)]) == 3
    assert "a target (or --replay)" in capsys.readouterr().err


def test_fidelity_names_what_differs(tmp_path):
    def write(name, verdict, sample=None, replay=None, failures=0):
        rd = tmp_path / name
        rd.mkdir()
        r = results.RunResult(run_id=name, verdict=verdict,
                              ssv2={"junit": None, "failures": failures, "errors": 0,
                                    "message": ""},
                              sample=sample, replay=replay)
        r.write(rd / "result.json")
    s = {"id": "b" * 64, "drawn_id": "a" * 64, "index": 0, "run_seed": 1, "category": "R1.a.b",
         "cell": {}, "base": None, "proposal": None, "weight": 1.0, "log_weight": 0.0,
         "file": "sample.json"}
    write("o", "passed", sample=s)
    write("r1", "passed", sample=s, replay={"of": "a" * 64, "of_run": "o", "target": {}})
    write("r2", "failed", sample=s, replay={"of": "a" * 64, "of_run": "o", "target": {}},
          failures=1)
    rep = sampled.fidelity(sorted(tmp_path.iterdir()))
    assert rep["total"] == 2 and rep["agree"] == 1 and rep["share"] == 0.5
    bad = next(p for p in rep["pairs"] if p["replay"] == "r2")
    assert bad["differs"] == ["verdict", "end"]
    assert bad["replay_outcome"]["end"] == "ssv2.failure"


def test_the_result_validator_takes_sample_and_replay():
    good = {"schema": "swb.result/1", "run_id": "x", "verdict": "passed",
            "sample": {"id": "a" * 64, "drawn_id": "b" * 64, "index": 0, "weight": 1.0,
                       "log_weight": 0.0},
            "replay": {"of": "b" * 64, "of_run": None, "target": {}}}
    assert results.problems(good) == []
    bad = json.loads(json.dumps(good))
    bad["sample"]["index"] = -1
    bad["replay"]["of"] = "short"
    assert results.problems(bad) == ["sample.index: expected an integer >= 0",
                                     "replay.of: expected the replayed record's id (64 hex "
                                     "digits)"]
    # a run without a sample has neither key
    assert "sample" not in results.RunResult("x", "passed").to_dict()
