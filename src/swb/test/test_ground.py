import pytest

from swb import ground


def proj(tmp_path):
    (tmp_path / "ground").mkdir()
    (tmp_path / "ground" / "t1.yaml").write_text("surfaces: []\n")
    (tmp_path / "ground" / "rough.yaml").write_text("rocks: []\n")
    return tmp_path


def test_problems(tmp_path):
    p = proj(tmp_path)
    assert ground.problems(p, {"Town01": {"surfaces": "ground/t1.yaml"}}) == []
    msgs = [m for _, m in ground.problems(p, {"A": {"surfaces": "nope.yaml", "x": "y"}, "B": 1})]
    assert "nope.yaml is not a file in the project" in msgs
    assert any("unknown key x" in m for m in msgs) and any("is not a mapping" in m for m in msgs)
    # without a project, files are not looked at
    assert ground.problems(None, {"A": {"surfaces": "nope.yaml"}}) == []


def test_apply_links_and_unlinks(tmp_path):
    p = proj(tmp_path)
    made = ground.apply(p, {"Town01": {"surfaces": "ground/t1.yaml",
                                       "road_detail": "ground/rough.yaml"}})
    s = p / "state" / "Town01"
    assert sorted(x.name for x in made) == ["road_detail.yaml", "surfaces.yaml"]
    assert (s / "surfaces.yaml").read_text() == "surfaces: []\n"
    # dropped from the project: the link goes, other state stays
    (s / "ruts.npz").write_text("r")
    ground.apply(p, {"Town01": {"surfaces": "ground/t1.yaml"}})
    assert not (s / "road_detail.yaml").exists() and (s / "ruts.npz").exists()
    ground.apply(p, None)
    assert not (s / "surfaces.yaml").exists()


def test_apply_refuses_a_hand_placed_file(tmp_path):
    p = proj(tmp_path)
    (p / "state" / "Town01").mkdir(parents=True)
    (p / "state" / "Town01" / "surfaces.yaml").write_text("mine\n")
    with pytest.raises(ground.GroundError, match="not a link swb made"):
        ground.apply(p, {"Town01": {"surfaces": "ground/t1.yaml"}})
