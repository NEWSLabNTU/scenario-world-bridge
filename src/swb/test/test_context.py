"""swb.context (roadmap 077, G33): SSv2's storyboard snapshots as state changes, the log
incident_judge reads; the runner records it when a judge needs it."""
import importlib.util
import json
from pathlib import Path

from swb import context, judges, record


def snapshot(event_state, action_state="standbyState", story_state="runningState"):
    return {"OpenSCENARIO": {"Storyboard": {"Story": [{
        "name": "incident", "currentState": story_state,
        "Act": {"name": "act", "currentState": "runningState",
                "ManeuverGroup": [{"name": "g", "currentState": "runningState",
                                   "Maneuver": [{"name": "m", "currentState": "runningState",
                                                 "Event": [{"name": "i1_lead_brake",
                                                            "currentState": event_state,
                                                            "Action": [{"name": "brake",
                                                                        "currentState": action_state}]}]}]}]}}]}}}


def test_elements_walks_every_level_with_lists_or_single_objects():
    e = context.elements(snapshot("runningState", "completeState"))
    assert e["incident"] == "runningState"
    assert e["incident/act/g/m/i1_lead_brake"] == "runningState"
    assert e["incident/act/g/m/i1_lead_brake/brake"] == "completeState"
    assert context.elements(None) == {} and context.elements({"OpenSCENARIO": {}}) == {}


def test_only_changes_are_written_and_the_first_snapshot_starts_from_null():
    c = context.Changes()
    first = c.feed(snapshot("standbyState"), None)
    assert {"ev": "state", "sim": None, "element": "incident/act/g/m/i1_lead_brake",
            "from": None, "to": "standbyState"} in first
    assert c.feed(snapshot("standbyState"), 1.0) == []
    [ev, act] = c.feed(snapshot("completeState", "completeState"), 2.0)
    assert (ev["element"], ev["from"], ev["to"], ev["sim"]) == \
        ("incident/act/g/m/i1_lead_brake", "standbyState", "completeState", 2.0)
    assert act["element"].endswith("/brake")


def _incident_judge():
    p = Path(__file__).resolve().parents[2] / "swb_launch" / "scripts" / "incident_judge"
    from importlib.machinery import SourceFileLoader
    loader = SourceFileLoader("incident_judge", str(p))
    spec = importlib.util.spec_from_loader("incident_judge", loader)
    m = importlib.util.module_from_spec(spec)
    loader.exec_module(m)
    return m


def test_incident_judge_reads_the_log_as_written():
    c = context.Changes()
    lines = []
    for t, (e, a) in enumerate([("standbyState", "standbyState"),
                                ("runningState", "runningState"),
                                ("completeState", "completeState")]):
        lines += [json.dumps(x) for x in c.feed(snapshot(e, a), float(t))]
    done, changes = _incident_judge().events_from_context(lines)
    assert done["i1_lead_brake"] == [1.0] and done["brake"] == [1.0]
    assert any(n == "i1_lead_brake" and to == "completeState" for _, n, to in changes)


def test_a_judge_may_need_the_context_and_the_run_then_records_it():
    assert judges.problems([{"name": "i1", "run": ["x"], "needs": ["context"]}]) == []
    assert judges.needed([{"name": "i1", "run": ["x"], "needs": ["context", "gap_trace"]}]) \
        == {"context", "gap_trace"}
    p = record.plan(Path("/r"), "standard", [{"entity": "ego", "domain": 1}], 9, context=True)
    [(name, domain, argv)] = p
    assert (name, domain) == ("context", 9)
    assert argv[1:3] == ["-m", "swb.context"] and argv[-1] == "/r/context.jsonl"
    full = record.plan(Path("/r"), "full", [{"entity": "ego", "domain": 1}], 9, context=True)
    assert [n for n, _, _ in full] == ["context", "ego"]
    assert record.plan(Path("/r"), "standard", [{"entity": "ego", "domain": 1}], 9) == []


def test_main_needs_one_path(capsys):
    assert context.main([]) == 2 and "usage" in capsys.readouterr().err


def test_incident_auto_takes_the_number_from_the_scenario_name(tmp_path, capsys):
    from swb import gap
    (tmp_path / "resolved.yaml").write_text(
        "scenario: {file: ../../src/swb_examples/scenarios/demo/incidents/"
        "town01_i3_red_light.xosc}\n")
    assert gap.incident_number(tmp_path, "auto") == (3, None)
    assert gap.incident_number(tmp_path, "5") == (5, None)
    assert gap.incident_number(tmp_path, "7") == (7, None)
    assert gap.incident_number(tmp_path, "8") == (8, None)
    n, why = gap.incident_number(tmp_path, "9")
    assert n is None and "no criteria" in why
    (tmp_path / "resolved.yaml").write_text("scenario: {file: town01_ego_drive.xosc}\n")
    n, why = gap.incident_number(tmp_path, "auto")
    assert n is None and "not named" in why
    assert gap.main(["incident", "--incident", "auto", "--context", "context.jsonl",
                     str(tmp_path)]) == 2
    assert json.loads(capsys.readouterr().out)["verdict"] == "error"


def test_standard_keeps_the_storyboard_log():
    from swb import runs
    assert "context.jsonl" in runs.kept_names("standard")
    assert "context.jsonl" not in runs.kept_names("result")


def test_the_stop_line_comes_from_the_scenario(tmp_path):
    from swb import gap
    sc = tmp_path / "i3.xosc"
    sc.write_text('<OpenSCENARIO><ParameterDeclarations><ParameterDeclaration '
                  'name="csbIncidentStopLine" parameterType="string" value="325.73,-330.61,-0.0"/>'
                  '</ParameterDeclarations></OpenSCENARIO>')
    (tmp_path / "resolved.yaml").write_text(f"scenario: {{path: {sc}}}\n")
    assert gap.scenario_stop_line(tmp_path) == (325.73, -330.61, -0.0)
    sc.write_text("<OpenSCENARIO/>")
    assert gap.scenario_stop_line(tmp_path) is None


def test_a_tour_is_judged_on_every_incident_its_storyboard_names(tmp_path, monkeypatch, capsys):
    from swb import gap
    (tmp_path / "resolved.yaml").write_text("scenario: {file: town01_tour.xosc}\n")
    ctx = tmp_path / "context.jsonl"
    ctx.write_text("\n".join(json.dumps({"ev": "state", "element": e, "to": "completeState"})
                             for e in ("tour/act/i1/m/i1_lead_brake", "tour/act/i3/m/i3_red",
                                       "tour/act/x/m/ambient_go", "tour/act/i9/m/i9_x")))
    assert gap.context_incidents(ctx) == [1, 3]          # 9 has no row; ambient is not one
    calls = []

    def one(run_dir, n, c, sl, numbered=False):
        calls.append(n)
        return {"incident": n, "verdict": "failed" if n == 3 else "passed", "message": "x"}
    monkeypatch.setattr(gap, "_judge_one", one)
    assert gap.incident(tmp_path, "auto", "context.jsonl", None) == 1
    out = json.loads(capsys.readouterr().out)
    assert calls == [1, 3] and out["verdict"] == "failed"
    assert out["message"].startswith("1 of 2 incidents passed; i3: x")
