from pathlib import Path

import pytest

from scensim import world


def test_an_ego_config_naming_a_package_is_handed_over_resolved(tmp_path, monkeypatch):
    """Issue 057: aib reads paths only, so a pkg:// rig is resolved into a copy."""
    import yaml
    pytest.importorskip("ament_index_python")
    prefix = tmp_path / "install"
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True)
    (index / "aib").write_text("")
    (prefix / "share" / "aib" / "config").mkdir(parents=True)
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    tb = tmp_path / "tb"
    (tb / "config").mkdir(parents=True)
    ego = tb / "config" / "ego.yaml"
    ego.write_text("rig: pkg://aib/config/car.rig.yaml\nbody: body/entry.json\nlockstep: {}\n")
    args = world.sim_args(tb, "isaac", {}, {"ego": "config/ego.yaml"}, ego_dir=tmp_path / "st")
    out = Path(args["ego"])
    assert out == tmp_path / "st" / world.RESOLVED_EGO
    doc = yaml.safe_load(out.read_text())
    assert doc == {"rig": str(prefix / "share" / "aib" / "config" / "car.rig.yaml"),
                   "body": str((tb / "config" / "body" / "entry.json").resolve()),
                   "lockstep": {}}
    # a config without pkg:// is handed over as it is
    ego.write_text("rig: car.rig.yaml\n")
    assert world.sim_args(tb, "isaac", {}, {"ego": "config/ego.yaml"})["ego"] == str(ego.resolve())
    ego.write_text("rig: pkg://nope/car.rig.yaml\n")
    from scensim import config
    with pytest.raises(config.ConfigError, match="package nope is not installed"):
        world.sim_args(tb, "isaac", {}, {"ego": "config/ego.yaml"})
