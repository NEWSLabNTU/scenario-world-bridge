"""The pip front end against a fake release served over HTTP on localhost.

Each test runs the real ``scensim`` command in a subprocess, with a fake release package
(``scensim_ros_<distro>``, with its dist-info) on ``PYTHONPATH`` and a fake tarball whose
``bin/scensim`` prints its arguments, so ``exec`` into the release is exercised too. The
acceptance rows are 243's user journey (docs/roadmap/081-study/user-journey.md), sections 1,
5 and 7.
"""

from __future__ import annotations

import hashlib
import http.server
import io
import json
import os
import subprocess
import sys
import tarfile
import threading
from pathlib import Path

import pytest
import zstandard

SRC = Path(__file__).resolve().parent.parent / "src"
VERSION = "0.1.0"


# --- a fake release ---------------------------------------------------------------------------


def make_tarball(
    dest: Path,
    version: str = VERSION,
    distro: str = "humble",
    side: str = "sim",
    top_dir: bool = False,
    links: dict[str, str] | None = None,
    members: dict[str, str] | None = None,
    relocate: str | None = None,
) -> Path:
    files = {
        # as 118's stage_sim.py writes it: a quoted version, nulls, lists and maps
        "scensim-release.yaml": f'version: "{version}"\nside: {side}\nros_distro: {distro}\n'
        f"arch: x86_64\nautoware: null\nbuilt: '2026-11-17T12:00:00Z'\n"
        f"build_seconds: 382\ncarla:\n  - {{version: 0.9.16, tier: 1}}\n"
        f"commits:\n  .: abc123\n  src/scensim_carla_world: def456\n",
        # as a colcon prefix's: sets what only it sets, and records the order of sourcing
        "setup.bash": f'export FAKE_SETUP_{side.upper()}=1\nexport ORDER="${{ORDER}}{side},"\n',
        "share/fake/package.xml": "<package/>\n",
    }
    if relocate is not None:
        files["relocate.sh"] = relocate
    if side == "sim":
        files["bin/scensim"] = (
            # like 241's: needs its prefix's environment, exits with the command's code
            "#!/bin/bash\n"
            '[ -n "$FAKE_SETUP_SIM" ] || { echo "fake scensim: no environment" >&2; exit 97; }\n'
            'echo "fake scensim: $* (setup=${FAKE_SETUP_SIM:-0},'
            ' vehicle=${FAKE_SETUP_VEHICLE:-0}) order=${ORDER}"\n'
            'case "$1" in exit) exit "$2";; esac\n'
        )
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w") as tar:
        for rel, text in files.items():
            data = text.encode()
            info = tarfile.TarInfo(("release/" if top_dir else "./") + rel)
            info.size = len(data)
            info.mode = 0o755 if rel.startswith("bin/") else 0o644
            tar.addfile(info, io.BytesIO(data))
        for name, target in (links or {}).items():   # symbolic links: name -> target
            info = tarfile.TarInfo("./" + name)
            info.type, info.linkname = tarfile.SYMTYPE, target
            tar.addfile(info)
        for name, text in (members or {}).items():   # files under any name, ../ included
            data = text.encode()
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    dest.write_bytes(zstandard.ZstdCompressor().compress(buf.getvalue()))
    return dest


def artefact(url: str, path: Path, sha: str | None = None, size: int | None = None) -> dict:
    data = path.read_bytes()
    return {
        "url": url,
        "size": len(data) if size is None else size,
        "sha256": sha or hashlib.sha256(data).hexdigest(),
    }


def make_release_package(root: Path, distro: str, manifest: dict) -> None:
    pkg = root / f"scensim_ros_{distro}"
    pkg.mkdir(parents=True, exist_ok=True)
    (pkg / "__init__.py").write_text(
        "import json, pathlib\n"
        "def manifest():\n"
        "    return json.loads((pathlib.Path(__file__).parent / 'manifest.json').read_text())\n"
    )
    (pkg / "manifest.json").write_text(json.dumps(manifest))
    info = root / f"scensim_ros_{distro}-{manifest['version']}.dist-info"
    info.mkdir(exist_ok=True)
    (info / "METADATA").write_text(
        f"Metadata-Version: 2.1\nName: scensim-ros-{distro}\nVersion: {manifest['version']}\n"
    )


class Server:
    """Serves ``directory``; ``cut_at`` sends only that many bytes of each body."""

    def __init__(self, directory: Path) -> None:
        self.hits = 0
        self.cut_at: int | None = None
        outer = self

        class Handler(http.server.SimpleHTTPRequestHandler):
            def __init__(self, *a, **kw):
                super().__init__(*a, directory=str(directory), **kw)

            def log_message(self, *a):  # quiet
                pass

            def copyfile(self, source, outputfile):
                outer.hits += 1
                data = source.read()
                if outer.cut_at is not None:
                    data = data[: outer.cut_at]
                outputfile.write(data)

        self.httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self) -> None:
        self.httpd.shutdown()


@pytest.fixture()
def env(tmp_path: Path):
    files = tmp_path / "files"
    files.mkdir()
    server = Server(files)
    pkgs = tmp_path / "site"
    pkgs.mkdir()
    shome = tmp_path / "scensim-home"

    def environ(ros: str | None = "humble", extra: dict | None = None) -> dict:
        e = {
            "PATH": "/usr/bin:/bin",
            "HOME": str(tmp_path),
            "SCENSIM_HOME": str(shome),
            "PYTHONPATH": f"{SRC}:{pkgs}",
            "PYTHONNOUSERSITE": "1",
        }
        if ros:
            e["ROS_DISTRO"] = ros
        e.update(extra or {})
        return e

    def run(*args: str, ros: str | None = "humble", extra: dict | None = None):
        return subprocess.run(
            [sys.executable, "-m", "scensim_frontend", *args],
            check=False,
            capture_output=True,
            text=True,
            env=environ(ros, extra),
            timeout=60,
        )

    ctx = type("Ctx", (), {})()
    ctx.files, ctx.server, ctx.pkgs, ctx.home, ctx.run = files, server, pkgs, shome, run
    ctx.environ = environ
    yield ctx
    server.close()


def publish(
    ctx,
    *,
    version: str = VERSION,
    tar_version: str = VERSION,
    distro: str = "humble",
    bad_sha: bool = False,
    size: int | None = None,
    vehicle: bool = False,
    top_dir: bool = False,
    links: dict[str, str] | None = None,
    members: dict[str, str] | None = None,
    relocate: str | None = None,
) -> dict:
    sim = make_tarball(
        ctx.files / f"scensim-sim-{version}-{distro}-x86_64.tar.zst",
        tar_version,
        distro,
        top_dir=top_dir,
        links=links,
        members=members,
        relocate=relocate,
    )
    arts: dict = {
        "sim": artefact(
            f"{ctx.server.url}/{sim.name}", sim, sha="0" * 64 if bad_sha else None, size=size
        )
    }
    if vehicle:
        veh = make_tarball(
            ctx.files / f"scensim-vehicle-{version}-autoware1.5.0-x86_64.tar.zst",
            tar_version,
            distro,
            side="vehicle",
        )
        arts["vehicle"] = {"autoware1.5.0": artefact(f"{ctx.server.url}/{veh.name}", veh)}
    manifest = {"version": version, "ros_distro": distro, "artefacts": {"x86_64": arts}}
    make_release_package(ctx.pkgs, distro, manifest)
    return manifest


# --- rows 1.x: install ------------------------------------------------------------------------


def test_version_never_downloads(env) -> None:
    publish(env)
    r = env.run("--version")
    assert r.returncode == 0
    assert "not installed for ROS humble: run `scensim install`" in r.stdout
    assert env.server.hits == 0 and not env.home.exists()


def test_install_downloads_checks_unpacks_and_prints_next_steps(env) -> None:
    publish(env)
    r = env.run("install")
    assert r.returncode == 0, r.stderr
    assert "downloading scensim 0.1.0 for ROS 2 humble" in r.stderr
    assert "sha256 ok" in r.stderr
    assert "sudo rosdep install --from-paths" in r.stderr and "/current/sim/share" in r.stderr
    assert f"source {env.home}/current/sim/setup.bash" in r.stderr
    assert (env.home / "current").resolve() == (env.home / "0.1.0-humble").resolve()
    assert (env.home / "0.1.0-humble" / "sim" / "bin" / "scensim").exists()
    assert not list((env.home / ".downloads").glob("*"))  # archive and .part removed
    v = env.run("--version")
    assert f"installed for ROS humble: {env.home}/0.1.0-humble" in v.stdout


def test_first_command_downloads_once_then_execs_the_release(env) -> None:
    publish(env)
    r = env.run("run", "my_first_drive")
    assert r.returncode == 0, r.stderr
    assert "first use: downloading scensim 0.1.0 for ROS 2 humble" in r.stderr
    assert "fake scensim: run my_first_drive (setup=1, vehicle=0)" in r.stdout
    hits = env.server.hits
    r2 = env.run("runs", "list", "my_first_drive")
    assert r2.returncode == 0 and "fake scensim: runs list my_first_drive" in r2.stdout
    assert env.server.hits == hits and "downloading" not in r2.stderr


def test_archive_with_one_top_directory(env) -> None:
    publish(env, top_dir=True)
    r = env.run("install")
    assert r.returncode == 0, r.stderr
    assert (env.home / "0.1.0-humble" / "sim" / "scensim-release.yaml").exists()


def test_vehicle_side_for_a_sourced_autoware(env) -> None:
    publish(env, vehicle=True)
    aw = {"AMENT_PREFIX_PATH": "/opt/autoware/1.5.0:/opt/ros/humble"}
    r = env.run("run", "x", extra=aw)
    assert r.returncode == 0, r.stderr
    assert "vehicle side for Autoware 1.5.0" in r.stderr
    assert "(setup=1, vehicle=1)" in r.stdout
    r = env.run("install", "--autoware", "2.0.0")
    assert r.returncode == 3
    assert "no prebuilt vehicle side for Autoware 2.0.0" in r.stderr and "autoware1.5.0" in r.stderr


# --- rows 5.x: the guards ---------------------------------------------------------------------


def test_an_autoware_without_a_prebuilt_vehicle_side_is_refused_on_every_run(env) -> None:
    """139's review, 1A: Autoware 2.0.0 sourced, a release with a 1.5.0 vehicle side only."""
    publish(env, vehicle=True)
    aw = {"AMENT_PREFIX_PATH": "/opt/autoware/2.0.0:/opt/ros/humble"}
    for _ in range(2):   # the rerun is refused too, never run without its vehicle side
        r = env.run("run", "x", extra=aw)
        assert r.returncode == 3, r.stderr
        assert "no prebuilt vehicle side for Autoware 2.0.0" in r.stderr
        assert "this release has: autoware1.5.0" in r.stderr
        assert "fake scensim" not in r.stdout
    assert env.server.hits == 0 and not (env.home / "current").exists()


def test_a_sim_only_install_gets_its_vehicle_side_once_autoware_is_sourced(env) -> None:
    """139's review, 1B: a first use without Autoware, then Autoware 1.5.0 sourced."""
    publish(env, vehicle=True)
    r = env.run("run", "x")
    assert r.returncode == 0 and "(setup=1, vehicle=0)" in r.stdout, r.stderr
    aw = {"AMENT_PREFIX_PATH": "/opt/autoware/1.5.0:/opt/ros/humble"}
    r = env.run("run", "x", extra=aw)
    assert r.returncode == 0, r.stderr
    assert "vehicle side for Autoware 1.5.0" in r.stderr
    assert "downloading scensim 0.1.0" not in r.stderr.replace("vehicle side", "")
    assert "(setup=1, vehicle=1)" in r.stdout
    assert (env.home / "current" / "vehicle-autoware1.5.0" / "setup.bash").exists()


def test_a_link_out_of_the_prefix_is_left_out_and_named(env) -> None:
    publish(env, links={"share/x/escape": "../../../outside", "bin/sh": "/bin/sh",
                        "lib/libx.so": "libx.so.1",
                        # inside the prefix, resolved from the link's own directory (Python
                        # 3.10.12 resolves these from the root and would refuse them)
                        "share/kashiwanoha_map/map/a/pointcloud_map.pcd": "../pointcloud_map.pcd",
                        "share/scensim_launch/data": "../../data"})
    r = env.run("install")
    assert r.returncode == 0, r.stderr
    assert "skipped share/x/escape: a link out of the install" in r.stderr
    assert "skipped bin/sh: a link out of the install" in r.stderr
    assert r.stderr.count("skipped ") == 2, r.stderr
    sim = env.home / "current" / "sim"
    assert not (sim / "share" / "x" / "escape").is_symlink()
    assert not (sim / "bin" / "sh").is_symlink()
    for kept in ("lib/libx.so", "share/kashiwanoha_map/map/a/pointcloud_map.pcd",
                 "share/scensim_launch/data"):
        assert (sim / kept).is_symlink(), kept


def test_a_file_out_of_the_prefix_refuses_the_archive(env, tmp_path: Path) -> None:
    publish(env, members={"../escaped.txt": "x"})
    r = env.run("install")
    assert r.returncode == 3, r.stderr
    assert "not a plain release" in r.stderr
    assert not (env.home / "escaped.txt").exists() and not (env.home / "current").exists()


# 118's relocate.sh (082-release-build 69ba2e1): the sh scripts' prefix placeholder
RELOCATE = (
    '#!/bin/sh\nset -e\nhere=$(cd "$(dirname "$0")" && pwd)\n'
    'sed -i "s|@SCENSIM_PREFIX@|$here|g" setup.sh\necho "$here" > .scensim-prefix\n'
)


# like colcon's setup.sh: refuses a build-time path that does not exist on this host
SETUP_SH = (
    '_colcon_prefix_sh_COLCON_CURRENT_PREFIX="@SCENSIM_PREFIX@"\n'
    'if [ ! -d "$_colcon_prefix_sh_COLCON_CURRENT_PREFIX" ]; then\n'
    '  echo "The build time path \\"$_colcon_prefix_sh_COLCON_CURRENT_PREFIX\\" '
    'doesn\'t exist." 1>&2\n  return 1\nfi\n'
)


def test_relocate_runs_in_the_final_prefix_before_current_moves(env) -> None:
    publish(env, members={"./setup.sh": SETUP_SH}, relocate=RELOCATE)
    r = env.run("install")
    assert r.returncode == 0, r.stderr
    sim = env.home / "0.1.0-humble" / "sim"
    assert (sim / ".scensim-prefix").read_text().strip() == str(sim)
    assert (env.home / "current").resolve() == sim.parent.resolve()
    s = subprocess.run(["sh", "-c", f". {env.home}/current/sim/setup.sh && echo sourced"],
                       capture_output=True, text=True, check=False)
    assert s.returncode == 0 and "sourced" in s.stdout, s.stderr


def test_a_release_without_relocate_is_installed_with_a_note(env) -> None:
    publish(env, members={"./setup.sh": SETUP_SH})
    r = env.run("install")
    assert r.returncode == 0, r.stderr
    assert "no relocate.sh in this release" in r.stderr
    s = subprocess.run(["sh", "-c", f". {env.home}/current/sim/setup.sh"],
                       capture_output=True, text=True, check=False)
    assert s.returncode != 0 and "doesn't exist" in s.stderr   # what relocate.sh fixes


def test_a_failing_relocate_removes_the_prefix_and_refuses(env) -> None:
    publish(env, relocate="#!/bin/sh\necho 'relocate: cannot write' >&2\nexit 1\n")
    r = env.run("install")
    assert r.returncode == 3, r.stderr
    assert "relocate.sh failed: relocate: cannot write" in r.stderr
    assert not (env.home / "0.1.0-humble" / "sim").exists()
    assert not (env.home / "current").exists()


def test_sha256_mismatch_removes_the_file(env) -> None:
    publish(env, bad_sha=True)
    r = env.run("install")
    assert r.returncode == 2
    assert "sha256 mismatch" in r.stderr and "run the command again" in r.stderr
    assert not list((env.home / ".downloads").glob("*"))
    assert not (env.home / "0.1.0-humble").exists()


def test_cut_download_then_retry(env) -> None:
    publish(env)
    env.server.cut_at = 100
    r = env.run("install")
    assert r.returncode == 2
    assert "partial file is removed" in r.stderr
    assert not list((env.home / ".downloads").glob("*.part"))
    env.server.cut_at = None
    r = env.run("install")
    assert r.returncode == 0, r.stderr


def test_short_file_with_matching_length_header(env) -> None:
    publish(env, size=10_000_000)  # the manifest says more than the server has
    r = env.run("install")
    assert r.returncode == 2 and "download cut" in r.stderr


def test_ros_distro_mismatch_names_the_release_package(env) -> None:
    publish(env)
    r = env.run("run", "x", ros="jazzy")
    assert r.returncode == 3
    assert "scensim is installed for humble, but ROS_DISTRO is jazzy (python " in r.stderr
    assert "pip install scensim-ros-jazzy" in r.stderr
    assert env.server.hits == 0


def test_release_package_version_skew(env) -> None:
    publish(env, version="0.0.9", tar_version="0.0.9")
    r = env.run("run", "x")
    assert r.returncode == 3
    assert "version skew: the scensim front end is 0.1.0, scensim-ros-humble is 0.0.9" in r.stderr


def test_tarball_version_skew_installs_nothing(env) -> None:
    publish(env, tar_version="0.0.9")
    r = env.run("install")
    assert r.returncode == 3 and "is scensim 0.0.9, the front end expects 0.1.0" in r.stderr
    assert not (env.home / "0.1.0-humble" / "sim").exists()


def test_placeholder_manifest_is_refused(env) -> None:
    make_release_package(
        env.pkgs,
        "humble",
        {
            "version": VERSION,
            "ros_distro": "humble",
            "artefacts": {"x86_64": {"sim": {"url": "x", "size": 0, "sha256": "written-later"}}},
        },
    )
    r = env.run("install")
    assert r.returncode == 3 and "development placeholder" in r.stderr


def test_distro_flag_disagreeing_with_sourced_ros(env) -> None:
    publish(env)
    r = env.run("install", "--distro", "jazzy")
    assert r.returncode == 3 and "--distro jazzy but the terminal sourced ROS humble" in r.stderr


# --- rows 7.x: no release package for this ROS ------------------------------------------------


def test_no_release_package_for_jazzy(env) -> None:
    r = env.run("run", "x", ros="jazzy")
    assert r.returncode == 3
    assert "no prebuilt scensim for ROS jazzy yet; build from source: https://" in r.stderr


def test_version_without_any_release_package(env) -> None:
    r = env.run("--version", ros="jazzy")
    assert r.returncode == 0
    assert "no prebuilt scensim for ROS jazzy yet" in r.stdout


def test_help(env) -> None:
    r = env.run("--help")
    assert r.returncode == 0 and "scensim install" in r.stdout


@pytest.mark.skipif(os.geteuid() == 0, reason="root ignores the permissions this needs")
def test_unknown_install_argument(env) -> None:
    publish(env)
    r = env.run("install", "--bogus")
    assert r.returncode == 3 and "unknown argument '--bogus'" in r.stderr


def test_v_prefixed_version_in_the_tarball(env) -> None:
    publish(env, tar_version="v0.1.0")
    r = env.run("run", "x")
    assert r.returncode == 0, r.stderr
    assert "fake scensim: run x" in r.stdout


def test_make_manifest_then_install(env, tmp_path: Path) -> None:
    """The release workflow's manifest writer feeds the front end as-is."""
    sim = make_tarball(env.files / "scensim-sim-0.1.0-humble-x86_64.tar.zst")
    veh = make_tarball(
        env.files / "scensim-vehicle-0.1.0-autoware1.5.0-x86_64.tar.zst", side="vehicle"
    )
    out = tmp_path / "manifest.json"
    tool = Path(__file__).resolve().parents[2] / "make_manifest.py"
    subprocess.run(
        [
            sys.executable,
            str(tool),
            "--version",
            "0.1.0",
            "--distro",
            "humble",
            "--base-url",
            env.server.url,
            "--out",
            str(out),
            str(sim),
            str(veh),
        ],
        check=True,
        capture_output=True,
    )
    manifest = json.loads(out.read_text())
    assert manifest["artefacts"]["x86_64"]["vehicle"]["autoware1.5.0"]["size"] == veh.stat().st_size
    make_release_package(env.pkgs, "humble", manifest)
    r = env.run("install", "--autoware", "1.5.0")
    assert r.returncode == 0, r.stderr
    assert (env.home / "current" / "vehicle-autoware1.5.0" / "setup.bash").exists()


def test_make_manifest_with_a_dashed_version(env, tmp_path: Path) -> None:
    """118's first build is 0.0.0-dev: the version holds a dash."""
    sim = make_tarball(env.files / "scensim-sim-0.0.0-dev-humble-x86_64.tar.zst", "0.0.0-dev")
    tool = Path(__file__).resolve().parents[2] / "make_manifest.py"
    out = tmp_path / "m.json"
    r = subprocess.run(
        [
            sys.executable,
            str(tool),
            "--version",
            "0.0.0-dev",
            "--distro",
            "humble",
            "--base-url",
            env.server.url,
            "--out",
            str(out),
            str(sim),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert r.returncode == 0, r.stderr
    assert json.loads(out.read_text())["artefacts"]["x86_64"]["sim"]["size"] == sim.stat().st_size


# --- the environment before exec (133, 2026-11-17) --------------------------------------------


def fake_root(tmp_path: Path, name: str, rel: str, var: str) -> Path:
    root = tmp_path / name
    (root / rel).mkdir(parents=True)
    (root / rel / "setup.bash").write_text(f'export {var}=1\nexport ORDER="${{ORDER}}{name},"\n')
    return root


def test_exec_needs_the_prefix_environment_and_passes_the_exit_code(env) -> None:
    publish(env)
    r = env.run("exit", "5")
    assert r.returncode == 5, r.stderr
    assert "no environment" not in r.stderr


def test_ros_autoware_sim_vehicle_order(env, tmp_path: Path) -> None:
    """No ROS sourced, Autoware not sourced but installed: ROS, Autoware, sim, vehicle."""
    publish(env, vehicle=True)
    ros = fake_root(tmp_path, "ros", "humble", "FAKE_ROS")
    aw = fake_root(tmp_path, "autoware", "1.5.0", "FAKE_AW")
    roots = {"SCENSIM_ROS_ROOT": str(ros), "SCENSIM_AUTOWARE_ROOT": str(aw)}
    r = env.run("install", "--distro", "humble", "--autoware", "1.5.0", ros=None, extra=roots)
    assert r.returncode == 0, r.stderr
    r = env.run("run", "x", ros=None, extra=roots)
    assert r.returncode == 0, r.stderr
    assert "order=ros,autoware,sim,vehicle," in r.stdout


def test_an_already_sourced_prefix_is_not_sourced_again(env) -> None:
    publish(env)
    assert env.run("install").returncode == 0
    sim = env.home / "0.1.0-humble" / "sim"
    r = env.run("run", "x", extra={"AMENT_PREFIX_PATH": str(sim), "FAKE_SETUP_SIM": "1"})
    assert r.returncode == 0, r.stderr
    assert "order=" in r.stdout and "order=sim" not in r.stdout


def test_help_adds_the_release_commands_once_installed(env) -> None:
    publish(env)
    r = env.run("--help")
    assert r.returncode == 0 and "fake scensim" not in r.stdout and env.server.hits == 0
    assert env.run("install").returncode == 0
    r = env.run("--help")
    assert r.returncode == 0
    assert "scensim install" in r.stdout and "fake scensim: --help" in r.stdout


# --- P2: the release's scensim forwards the front end's own commands here -----------------
# Once the release's setup.bash is sourced, its bin/scensim is first on PATH and hands
# --version, -V and install to scensim_frontend.cli.main(argv); these must never exec the
# release again, or the two would loop.


@pytest.mark.parametrize("args", [["--version"], ["-V"], ["install"], ["install", "--force"]])
def test_the_front_ends_own_commands_never_exec_the_release(env, args) -> None:
    publish(env)
    assert env.run("install").returncode == 0
    r = env.run(*args)
    assert r.returncode == 0, r.stderr
    assert "fake scensim" not in r.stdout + r.stderr


def test_main_is_a_callable_that_returns_the_exit_code(env) -> None:
    """241's release entry imports this and calls it; the contract is main(argv) -> int."""
    publish(env)
    code = (
        "import sys\n"
        "from scensim_frontend.cli import main\n"
        "for argv in (['-V'], ['--version'], ['install'], ['install', '--bogus']):\n"
        "    print('RET', argv[-1], main(argv))\n"
    )
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                       env=env.environ(), timeout=60, check=False)
    assert r.returncode == 0, r.stderr
    assert "RET -V 0" in r.stdout and "RET --version 0" in r.stdout
    assert "RET install 0" in r.stdout and "RET --bogus 3" in r.stdout
    assert "fake scensim" not in r.stdout


def test_help_is_the_front_ends_own_before_any_install(env) -> None:
    publish(env)
    r = env.run("-h")
    assert r.returncode == 0 and "scensim install" in r.stdout
    assert "fake scensim" not in r.stdout and env.server.hits == 0
