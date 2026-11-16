import hashlib
import os
from pathlib import Path

import pytest
import yaml

from swb import assets

from conftest import TOWN

MAPS = "jerry73204/carla-town-autoware-maps"


def project(tmp, text):
    p = tmp / "proj"
    p.mkdir(exist_ok=True)
    (p / "swb.yaml").write_text(text)
    return p


def lock(p):
    return yaml.safe_load((p / "swb.lock").read_text())


MAP_PROJECT = """simulator: carla
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01]
pilots: {ego: {kind: autoware}}
"""


def test_fetch_takes_links_and_locks(host, maps_hub):
    p = project(host["tmp"], MAP_PROJECT)
    out = []
    assets.fetch(p, out=out.append)
    lk = lock(p)
    commit = maps_hub.repos[MAPS]["refs"]["tags"]["v1"]
    e = lk["datasets"]["carla-town-autoware-maps"]
    assert lk["version"] == 1 and e["repo"] == MAPS and e["commit"] == commit
    assert e["take"] == ["Town01"]
    assert set(e["files"]) == {"Town01/lanelet2_map.osm", "Town01/pointcloud_map.pcd",
                               "Town01/isaac/world.yaml"}
    want = "sha256:" + hashlib.sha256(TOWN["Town01/lanelet2_map.osm"].encode()).hexdigest()
    assert e["files"]["Town01/lanelet2_map.osm"] == want
    cdir = host["cache"] / "datasets" / MAPS / commit
    link = p / "assets" / "maps" / "Town01"
    assert link.is_symlink() and Path(os.readlink(link)) == cdir / "Town01"
    assert (link / "lanelet2_map.osm").read_text() == "<osm town01/>"
    assert not (p / "assets" / "carla-town-autoware-maps").exists()  # all of it is a map
    assert not (cdir / "Town05").exists()
    # read-only once complete, files and directories
    assert not os.access(cdir / "Town01" / "lanelet2_map.osm", os.W_OK)
    assert not os.access(cdir / "Town01", os.W_OK)
    assert (p / "state").is_dir()
    assert any("swb.lock written" in o for o in out)


def test_fetch_is_idempotent(host, maps_hub):
    p = project(host["tmp"], MAP_PROJECT)
    assets.fetch(p, out=lambda s: None)
    n = len(maps_hub.downloads)
    text = (p / "swb.lock").read_text()
    out = []
    assets.fetch(p, out=out.append)
    assert len(maps_hub.downloads) == n
    assert out == ["carla-town-autoware-maps: unchanged"]
    assert (p / "swb.lock").read_text() == text


def test_lock_wins_over_new_tag_until_update(host, maps_hub):
    p = project(host["tmp"], MAP_PROJECT)
    assets.fetch(p, out=lambda s: None)
    old = lock(p)["datasets"]["carla-town-autoware-maps"]["commit"]
    files = dict(TOWN, **{"Town01/lanelet2_map.osm": "<osm town01 v2/>"})
    new = maps_hub.commit(MAPS, files, tag="v2", when="2026-12-01T00:00:00.000Z")
    assets.fetch(p, out=lambda s: None)
    assert lock(p)["datasets"]["carla-town-autoware-maps"]["commit"] == old
    out = []
    assets.fetch(p, update=True, out=out.append)
    assert lock(p)["datasets"]["carla-town-autoware-maps"]["commit"] == new
    assert any("->" in o for o in out)
    assert (p / "assets/maps/Town01/lanelet2_map.osm").read_text() == "<osm town01 v2/>"
    # the old commit's copy is untouched
    assert (host["cache"] / "datasets" / MAPS / old / "Town01/lanelet2_map.osm").is_file()


def test_newest_tag_by_date_not_name(host, hub):
    a = hub.commit("jerry73204/d", {"x/a.txt": "a"}, tag="zz-old", when="2026-01-01T00:00:00Z")
    b = hub.commit("jerry73204/d", {"x/a.txt": "b"}, tag="aa-new", when="2026-06-01T00:00:00Z")
    assert a != b
    p = project(host["tmp"], "simulator: carla\npilots: {}\nassets: [{dataset: d}]\n")
    assets.fetch(p, out=lambda s: None)
    assert lock(p)["datasets"]["d"]["commit"] == b
    assert (p / "assets" / "d").is_symlink()


def test_no_tag_means_main(host, hub):
    sha = hub.commit("jerry73204/d", {"a.txt": "a"})
    p = project(host["tmp"], "simulator: carla\npilots: {}\nassets: [{dataset: d}]\n")
    assets.fetch(p, out=lambda s: None)
    assert lock(p)["datasets"]["d"]["commit"] == sha


def test_revision_and_repo(host, hub):
    first = hub.commit("someone/maps", {"M/lanelet2_map.osm": "1"}, tag="v1")
    hub.commit("someone/maps", {"M/lanelet2_map.osm": "2"}, tag="v2")
    p = project(host["tmp"], "simulator: carla\npilots: {}\nassets:\n"
                "  - {dataset: maps, repo: someone/maps, revision: v1}\n")
    assets.fetch(p, out=lambda s: None)
    e = lock(p)["datasets"]["maps"]
    assert e["commit"] == first and e["revision"] == "v1"
    assert (p / "assets/maps/M/lanelet2_map.osm").read_text() == "1"


def test_manifest_mismatch_refuses(host, hub):
    files = {"T/lanelet2_map.osm": b"good", "U/lanelet2_map.osm": b"u"}
    sha = hub.commit("jerry73204/m", files, tag="v1", manifest=True)
    c = hub.repos["jerry73204/m"]["commits"][sha]["files"]
    c["T/lanelet2_map.osm"] = b"tampered"
    p = project(host["tmp"], "simulator: carla\npilots: {}\nassets: [{dataset: m, take: [T]}]\n")
    with pytest.raises(assets.AssetError, match="the manifest says"):
        assets.fetch(p, out=lambda s: None)
    assert not (p / "swb.lock").exists()
    assert not (host["cache"] / "datasets/jerry73204/m" / sha / "T/lanelet2_map.osm").exists()
    # only taken files are verified: U is fine, T is not taken
    (p / "swb.yaml").write_text("simulator: carla\npilots: {}\n"
                                "assets: [{dataset: m, take: [U]}]\n")
    assets.fetch(p, out=lambda s: None)
    assert (p / "assets/maps/U").is_symlink()


def test_take_matching_nothing_refuses(host, maps_hub):
    p = project(host["tmp"], MAP_PROJECT.replace("Town01", "Town99"))
    with pytest.raises(assets.AssetError, match="Town99 matches nothing"):
        assets.fetch(p, out=lambda s: None)


def test_take_globs():
    files = ["Town01/a", "Town01/b/c", "Town010/x", "Town05/a", "README.md"]
    assert assets.select(files, ["Town01"])[0] == ["Town01/a", "Town01/b/c"]
    assert assets.select(files, ["Town0?"]) == (["Town01/a", "Town01/b/c", "Town05/a"], [])
    assert assets.select(files, None)[0] == sorted(files)
    assert assets.select(files, ["nope"])[1] == ["nope"]


def test_usd_set_and_other_datasets(host, maps_hub):
    maps_hub.commit("jerry73204/isaac-road-physics-stages", {"surfaces/mud.usda": "m"}, tag="t")
    p = project(host["tmp"], """simulator: isaac
pilots: {}
assets:
  - {dataset: carla-0.9.16-isaac-usd, take: [Town01, vehicles]}
  - {dataset: isaac-road-physics-stages}
""")
    assets.fetch(p, out=lambda s: None)
    usd = p / "assets" / "carla-usd"
    assert (usd / "Town01/Town01_physics.usda").read_text() == "usda"
    assert not (usd / "Town02").exists()
    assert (p / "assets/isaac-road-physics-stages/surfaces/mud.usda").is_file()


def test_path_assets(host, hub):
    shared = host["tmp"] / "shared"
    (shared / "my_map").mkdir(parents=True)
    (shared / "my_map" / "lanelet2_map.osm").write_text("m")
    (shared / "stuff").mkdir()
    p = project(host["tmp"], """simulator: carla
pilots: {}
assets:
  - {path: ../shared/my_map}
  - {path: ../shared/stuff, as: things}
""")
    assets.fetch(p, out=lambda s: None)
    assert Path(os.readlink(p / "assets/maps/my_map")) == (shared / "my_map").resolve()
    assert (p / "assets/things").is_symlink()
    assert lock(p)["paths"] == {"my_map": {"path": "../shared/my_map"},
                                "things": {"path": "../shared/stuff"}}
    assert assets.check(p) == []


def test_dropped_asset_is_unlinked(host, maps_hub):
    p = project(host["tmp"], MAP_PROJECT.replace("[Town01]", "[Town01, Town05]"))
    assets.fetch(p, out=lambda s: None)
    assert (p / "assets/maps/Town05").is_symlink()
    (p / "swb.yaml").write_text(MAP_PROJECT)
    out = []
    assets.fetch(p, out=out.append)
    assert not (p / "assets/maps/Town05").exists()
    assert "unlinked assets/maps/Town05" in out


def test_refetches_a_missing_cache_file(host, maps_hub):
    p = project(host["tmp"], MAP_PROJECT)
    assets.fetch(p, out=lambda s: None)
    e = lock(p)["datasets"]["carla-town-autoware-maps"]
    cdir = host["cache"] / "datasets" / MAPS / e["commit"] / "Town01"
    os.chmod(cdir, 0o755)
    (cdir / "pointcloud_map.pcd").unlink()
    assert any("missing from the cache" in f.what for f in assets.check(p))
    out = []
    assets.fetch(p, out=out.append)
    assert (cdir / "pointcloud_map.pcd").read_text() == "pcd01"
    assert out[0] == "carla-town-autoware-maps: 1 missing file(s) fetched again"
    assert assets.check(p) == []


def test_check(host, maps_hub, capsys):
    p = project(host["tmp"], MAP_PROJECT)
    assert assets.main(["--project", str(p), "check"]) == 3  # nothing fetched: refused
    assert "never fetched" in capsys.readouterr().out
    assert assets.main(["fetch", f"project:={p}"]) == 0
    assert assets.check(p) == []
    assert assets.main(["--project", str(p), "check"]) == 0
    commit = lock(p)["datasets"]["carla-town-autoware-maps"]["commit"]
    f = host["cache"] / "datasets" / MAPS / commit / "Town01/pointcloud_map.pcd"
    os.chmod(f, 0o644)
    f.write_text("changed")
    capsys.readouterr()
    assert assets.main(["--project", str(p), "check"]) == 1
    out = capsys.readouterr().out
    assert "problem: swb.lock: carla-town-autoware-maps: Town01/pointcloud_map.pcd differs" in out
    # swb.yaml moved on since the fetch
    (p / "swb.yaml").write_text(MAP_PROJECT.replace("[Town01]", "[Town05]"))
    assert any("changed since the fetch" in x.what and x.refused for x in assets.check(p))


def test_check_finds_a_broken_link(host, maps_hub):
    p = project(host["tmp"], MAP_PROJECT)
    assets.fetch(p, out=lambda s: None)
    (p / "assets/maps/Town01").unlink()
    [f] = assets.check(p)
    assert f.file == "assets/maps/Town01" and f.refused


def test_schema_findings(host):
    p = project(host["tmp"], """simulator: carla
pilots: {}
assets:
  - {dataset: a, tkae: [x]}
  - {dataset: b, path: c}
  - 7
""")
    _, _, found = assets.parse_assets(p, yaml.safe_load((p / "swb.yaml").read_text()))
    assert len(found) == 3
    assert "unknown key tkae" in found[0].what
    with pytest.raises(assets.AssetError, match="tkae"):
        assets.fetch(p, out=lambda s: None)


def test_unknown_dataset(host, hub):
    p = project(host["tmp"], "simulator: carla\npilots: {}\nassets: [{dataset: nope}]\n")
    assert assets.main(["--project", str(p), "fetch"]) == 3


def test_unreachable_hub_is_infrastructure(host, monkeypatch):
    p = project(host["tmp"], MAP_PROJECT)
    (p / ".swb").mkdir()
    (p / ".swb" / "local.yaml").write_text("hub_url: http://127.0.0.1:9\n")
    monkeypatch.setattr(assets.time, "sleep", lambda s: None)
    assert assets.main(["--project", str(p), "fetch"]) == 2


def test_list(host, maps_hub, capsys):
    p = project(host["tmp"], MAP_PROJECT)
    assets.fetch(p, out=lambda s: None)
    more = MAP_PROJECT.replace("pilots:", "  - {dataset: haruna-tsukuba-courses}\npilots:")
    p.joinpath("swb.yaml").write_text(more)
    assert assets.main(["--project", str(p), "list"]) == 0
    out = capsys.readouterr().out
    for name, _, _ in assets.KNOWN:
        assert name in out
    assert "take Town01  3 file(s)" in out
    assert "haruna-tsukuba-courses  not fetched" in out


def test_make_hub_prefers_http_for_another_url():
    assert isinstance(assets.make_hub({"hub_url": "http://x"}), assets.HttpHub)


def test_sha256sums_manifest(host, hub):
    sums = f"{hashlib.sha256(b'other').hexdigest()}  ./a.usda\n"
    hub.commit("jerry73204/s", {"a.usda": "a", "SHA256SUMS": sums}, tag="v1")
    p = project(host["tmp"], "simulator: carla\npilots: {}\n"
                "assets: [{dataset: s, take: [a.usda]}]\n")
    with pytest.raises(assets.AssetError, match="the manifest says"):
        assets.fetch(p, out=lambda s: None)


def test_root_docs_are_not_held_to_the_manifest(host, hub):
    sums = f"{hashlib.sha256(b'old card').hexdigest()}  ./README.md\n"
    hub.commit("jerry73204/s", {"README.md": "new card", "SHA256SUMS": sums}, tag="v1")
    p = project(host["tmp"], "simulator: carla\npilots: {}\nassets: [{dataset: s}]\n")
    assets.fetch(p, out=lambda s: None)
    assert assets.check(p) == []


def test_prune_keeps_what_a_lock_names(host, maps_hub):
    p = project(host["tmp"], MAP_PROJECT)
    assets.fetch(p, out=lambda s: None)
    commit = lock(p)["datasets"]["carla-town-autoware-maps"]["commit"]
    cdir = host["cache"] / "datasets" / MAPS / commit
    stale = host["cache"] / "datasets" / MAPS / ("0" * 40)
    (stale / "Town01").mkdir(parents=True)
    (stale / "Town01" / "f").write_text("x")
    os.chmod(stale / "Town01" / "f", 0o444)
    os.chmod(stale / "Town01", 0o555)
    out = []
    assert assets.prune(host["cache"], dry_run=True, out=out.append) == [stale]
    assert stale.exists() and "would free" in out[-1]
    assert assets.prune(host["cache"], out=out.append) == [stale]
    assert not stale.exists() and cdir.is_dir()
    # the project gone: its commit goes too
    (p / "swb.lock").unlink()
    assert assets.prune(host["cache"], out=out.append) == [cdir]


def test_a_dataset_the_host_keeps_is_linked_from_its_copy_and_never_locked(host, maps_hub):
    """G8: a private dataset from the lab's disk: the project names it as any other."""
    p = project(host["tmp"], MAP_PROJECT.replace("carla-town-autoware-maps", "private-maps"))
    copy = host["tmp"] / "nas" / "private-maps"
    (copy / "Town01").mkdir(parents=True)
    (copy / "Town01" / "lanelet2_map.osm").write_text("<osm private/>")
    (copy / "Town05").mkdir()
    (copy / "Town05" / "lanelet2_map.osm").write_text("<osm other/>")
    hy = host["tmp"] / "user" / "host.yaml"
    h = yaml.safe_load(hy.read_text())
    h["datasets"] = {"private-maps": str(copy)}
    hy.write_text(yaml.safe_dump(h))
    out = []
    assets.fetch(p, out=out.append)
    link = p / "assets" / "maps" / "Town01"
    assert Path(os.readlink(link)) == copy / "Town01"
    assert not (p / "assets" / "maps" / "Town05").exists()      # take: [Town01]
    assert "private-maps" not in (lock(p).get("datasets") or {})  # nothing host's in the lock
    assert assets.check(p) == []
    import shutil
    shutil.rmtree(copy)
    [f] = assets.check(p)
    assert f.refused and "is not a directory" in f.what
