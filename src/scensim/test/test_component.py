"""A pilot's own control component (roadmap 091 W3): the testbench key, where its package is found, the
stack's ego config and the run's provenance."""
import hashlib
from pathlib import Path

import pytest
import yaml

from scensim import component as comp
from scensim import config, stack

from test_stack import make, where  # noqa: F401 (fixtures)

PILOT = {"kind": "launch", "launch": "pkg://x/y.launch.py", "vehicle": "tesla_model3",
         "overlays": ["{workspace:racer}/install/setup.bash"],
         "component": {"entry_point": "my_racer.sample", "params": {"erpm_gain": 4614.0},
                       "truth": {"speed": {"latency_s": 0.02, "noise_std": 0.01}}}}


def workspace(root: Path, isolated=True):
    """A colcon install with one pure package declaring my_racer.sample (what setup.py install writes)."""
    install = root / "install"
    site = (install / "my_racer_model" if isolated else install) / "lib" / "python3.10" / "site-packages"
    (site / "my_racer_model").mkdir(parents=True)
    (site / "my_racer_model" / "__init__.py").write_text("class Sample:\n    commands = 'autoware'\n")
    info = site / "my_racer_model-0.1.0-py3.10.egg-info"
    info.mkdir()
    (info / "PKG-INFO").write_text("Metadata-Version: 2.1\nName: my-racer-model\nVersion: 0.1.0\n")
    (info / "entry_points.txt").write_text(
        "[scensim.vehicle_components]\nmy_racer.sample = my_racer_model:Sample\n")
    (install / "setup.bash").write_text("# colcon\n")
    return site


def test_the_key_s_shape():
    assert comp.schema_problems("ego", PILOT, "isaac") == []
    assert comp.schema_problems("ego", {"kind": "simple"}, "carla") == []
    bad = {"component": {"params": [1], "truth": {"telepathy": {}, "speed": {"delay": 1}}, "colour": 1}}
    msgs = [f.what for f in comp.schema_problems("ego", bad, "carla")]
    assert any("unknown key colour" in m for m in msgs)
    assert any("entry_point: <name> is required" in m for m in msgs)
    assert any("params: is not a mapping" in m for m in msgs)
    assert any("telepathy is not a ground-truth input" in m for m in msgs)
    assert any("truth.speed: needs" in m for m in msgs)
    assert any("runs on Isaac only" in m for m in msgs)


def test_the_entry_point_is_found_in_the_workspace_without_importing_it(tmp_path):
    for isolated in (True, False):
        site = workspace(tmp_path / ("iso" if isolated else "merged"), isolated)
        assert comp.site_dirs(site.parents[2] if isolated else site.parents[2]) == [str(site)] or \
            comp.site_dirs(site.parents[3]) == [str(site)]
        found = comp.lookup("my_racer.sample", [str(site)])
        assert found["dist"] == "my-racer-model" and found["version"] == "0.1.0"
        assert found["value"] == "my_racer_model:Sample"
        init = site / "my_racer_model" / "__init__.py"
        assert found["file"] == str(init.resolve())
        assert found["sha256"] == hashlib.sha256(init.read_bytes()).hexdigest()
        assert comp.lookup("my_racer.nope", [str(site)]) is None


def test_its_workspace_resolves_and_a_missing_one_is_refused(tmp_path):
    ws = tmp_path / "racer"
    site = workspace(ws)
    host = {"workspaces": {"racer": str(ws)}}
    rec = comp.resolved(tmp_path, PILOT, host)
    assert rec["paths"] == [str(site)] and rec["found"]["dist"] == "my-racer-model"
    assert comp.ego_entry(rec) == {"entry_point": "my_racer.sample", "paths": [str(site)],
                                   "params": {"erpm_gain": 4614.0},
                                   "truth": {"speed": {"latency_s": 0.02, "noise_std": 0.01}}}
    settings = {"pilots": {"ego": PILOT}}
    assert comp.problems(tmp_path, settings, host) == []
    [f] = comp.problems(tmp_path, {"pilots": {"ego": {**PILOT, "component": {"entry_point": "x.y"}}}}, host)
    assert f.refused and "no package declares x.y" in f.what
    [f] = comp.problems(tmp_path, settings, {})
    assert f.refused and "no package declares" in f.what  # no workspace racer on this host: nothing found
    alone = {**PILOT, "component": {**PILOT["component"], "workspace": "{workspace:other}/install/setup.bash"}}
    [f] = comp.problems(tmp_path, {"pilots": {"ego": alone}}, host)
    assert "component.workspace" in f.what and "names no workspace other" in f.what
    # the shipped ones need no package
    assert comp.problems(tmp_path, {"pilots": {"ego": {"component": {"entry_point": "ideal"}}}}, {}) == []


def test_the_budget_note():
    over = {"ok": True, "mean_s": 2e-4, "worst_s": 5e-4, "budget_s": 1e-4}
    assert "0.200 ms per step" in comp.budget_note("ego", over) and "goes ahead" in comp.budget_note("ego", over)
    assert comp.budget_note("ego", {**over, "mean_s": 5e-5}) == ""
    assert comp.budget_note("ego", {"ok": False}) == ""


def test_the_stack_hands_the_world_server_its_component(where, maps_hub, tmp_path):  # noqa: F811
    """The ego config of an Isaac stack whose ego pilot names a component carries it, with the workspace's
    package directories; the stack record keeps what was found, for the run's provenance."""
    base = tmp_path / "aib" / "ego.yaml"
    base.parent.mkdir()
    base.write_text("rig: tesla_model3.rig.yaml\ndomain: 1\nnode_name: isaac_vehicle\n")
    ws = tmp_path / "racer"
    site = workspace(ws)
    hy = where["tmp"] / "user" / "host.yaml"
    h = yaml.safe_load(hy.read_text())
    h["workspaces"] = {"racer": str(ws)}
    hy.write_text(yaml.safe_dump(h))
    d = make(where, template="isaac")
    doc = yaml.safe_load((d / "testbench.yaml").read_text())
    doc["pilots"]["ego"]["component"] = {"entry_point": "my_racer.sample", "workspace":
                                         "{workspace:racer}/install/setup.bash"}
    (d / "testbench.yaml").write_text(yaml.safe_dump(doc, sort_keys=False))
    real = stack.aib_ego_config
    stack.aib_ego_config = lambda: base
    try:
        p = stack.plan(project_arg=str(d))
    finally:
        stack.aib_ego_config = real
    cfg = p.record["isaac_ego"]["config"]
    assert cfg["component"] == {"entry_point": "my_racer.sample", "paths": [str(site)]}
    assert cfg["rig"] == str(base.parent / "tesla_model3.rig.yaml") and "ground_truth" not in cfg
    assert f"ego:={p.stack_dir / stack.ISAAC_EGO}" in p.launch_args["sim_args"]
    [ego] = p.record["pilots"]
    assert ego["component"]["found"]["dist"] == "my-racer-model"
    stack.write_plan(p)
    assert yaml.safe_load((p.stack_dir / stack.ISAAC_EGO).read_text()) == cfg
    # on CARLA it is refused, by name
    with pytest.raises(stack.StackRefused, match="Isaac only"):
        doc["simulator"] = "carla"
        (d / "testbench.yaml").write_text(yaml.safe_dump(doc, sort_keys=False))
        stack.plan(project_arg=str(d), map_name="Town01")
    assert config.PROJECT_FILE == "testbench.yaml"
