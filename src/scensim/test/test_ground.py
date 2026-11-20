import pytest

from scensim import ground


def proj(tmp_path):
    tmp_path.mkdir(parents=True, exist_ok=True)
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
    with pytest.raises(ground.GroundError, match="not a link scensim made"):
        ground.apply(p, {"Town01": {"surfaces": "ground/t1.yaml"}})


def test_apply_again_with_a_file_in_a_linked_dataset(tmp_path):
    """Issue 043: a ground file under assets/, a link into a cache outside the project."""
    cache = tmp_path / "cache" / "ds"
    cache.mkdir(parents=True)
    (cache / "rough.yaml").write_text("rocks: []\n")
    p = proj(tmp_path / "p")
    (p / "assets").mkdir()
    (p / "assets" / "ds").symlink_to(cache)
    g = {"Town01": {"road_detail": "assets/ds/rough.yaml"}}
    dst = p / "state" / "Town01" / "road_detail.yaml"
    ground.apply(p, g)
    assert dst.read_text() == "rocks: []\n"
    ground.apply(p, g)  # the second start knows its own link
    # a link an older scensim made, to the resolved file in the cache, is taken too
    dst.unlink()
    dst.symlink_to(cache / "rough.yaml")
    ground.apply(p, g)
    assert str(dst.readlink()).startswith(str(p.resolve()))
    ground.apply(p, None)
    assert not dst.exists()


def test_a_ground_file_from_another_package(tmp_path, monkeypatch):
    """Issue 054: a testbench names a shared package's ground file as pkg://<package>/<path>;
    its link is scensim's, so it is replaced and dropped like a project file's."""
    pytest.importorskip("ament_index_python")
    prefix = tmp_path / "install"
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True)
    (index / "road_pkg").write_text("")
    (prefix / "share" / "road_pkg" / "ground").mkdir(parents=True)
    shared = prefix / "share" / "road_pkg" / "ground" / "town01.yaml"
    shared.write_text("surfaces: [mud]\n")
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    p = proj(tmp_path / "tb")
    g = {"Town01": {"surfaces": "pkg://road_pkg/ground/town01.yaml"}}
    assert ground.problems(p, g) == []
    ground.apply(p, g)
    link = p / "state" / "Town01" / "surfaces.yaml"
    assert link.read_text() == "surfaces: [mud]\n"
    ground.apply(p, g)                       # again: its own link, replaced
    ground.apply(p, None)                    # no longer named: dropped
    assert not link.exists() and not link.is_symlink()
    msgs = [m for _, m in ground.problems(p, {"Town01": {"surfaces": "pkg://nope/g.yaml"}})]
    assert any("package nope is not installed" in m for m in msgs)
    with pytest.raises(ground.GroundError, match="package nope is not installed"):
        ground.apply(p, {"Town01": {"surfaces": "pkg://nope/g.yaml"}})


def test_named_surface_maps_link_into_the_maps_surfaces_dir(tmp_path):
    """056, 086 C4: ground.<map>.surface_maps.<name> is <state>/<map>/surfaces/<name>.yaml."""
    (tmp_path / "ground").mkdir()
    (tmp_path / "ground" / "mud.yaml").write_text("zones: []\n")
    g = {"Town01": {"surface_maps": {"mud": "ground/mud.yaml"}}}
    assert ground.problems(tmp_path, g) == []
    made = ground.apply(tmp_path, g)
    dst = tmp_path / "state" / "Town01" / "surfaces" / "mud.yaml"
    assert made == [dst] and dst.resolve() == (tmp_path / "ground" / "mud.yaml").resolve()
    ground.apply(tmp_path, {"Town01": {"surfaces": "ground/mud.yaml"}})  # no longer named
    assert not dst.exists() and not dst.is_symlink()


def test_named_surface_maps_are_checked(tmp_path):
    got = dict(ground.problems(tmp_path, {
        "Town01": {"surface_maps": {"Bad Name": "ground/a.yaml", "ok": "ground/gone.yaml"}},
        "Town02": {"surface_maps": []}}))
    assert "[a-z0-9_-]+" in got["ground.Town01.surface_maps.Bad Name"]
    assert "not a file in the project" in got["ground.Town01.surface_maps.ok"]
    assert "is not a mapping" in got["ground.Town02.surface_maps"]
