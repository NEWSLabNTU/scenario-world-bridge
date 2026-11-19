"""``scensim setup``, ``autoware-side build`` and ``carla fetch`` (roadmap 089), with no network,
no sudo and no ROS: sudo, apt, rosdep, vcs, colcon, ros2 and sysctl are stubs on PATH that
act on a fake /etc, a fake /proc/sys and a state directory, and log every call."""

from __future__ import annotations

import io
import hashlib
import tarfile
from pathlib import Path

import pytest

from test_frontend import SRC, env, fake_root, publish  # noqa: F401 (env is a fixture)

REPO = Path(__file__).resolve().parents[3]
ASV = "a" * 40

STUBS = {
    # sudo: `sudo bash -c <script>` runs it as "root" (counted); `sudo -u <user> -H cmd` runs cmd
    "sudo": r'''
echo "sudo $*" >> "$STUB_LOG"
if [ "$1" = -u ]; then shift 3; exec "$@"; fi
echo ROOT >> "$STUB_LOG"
SUDO_USER=tester exec "$@"
''',
    "dpkg-query": r'''
pkg="${@: -1}"
[ -f "$FAKE_STATE/apt/$pkg" ] && { printf 'install ok installed'; exit 0; }
exit 1
''',
    "apt-get": r'''
echo "apt-get $*" >> "$STUB_LOG"
mkdir -p "$FAKE_STATE/apt"
for a in "$@"; do case "$a" in -*|install|update) ;; *) touch "$FAKE_STATE/apt/$a";; esac; done
''',
    "rosdep": r'''
echo "rosdep $*" >> "$STUB_LOG"
case "$1" in
  init) mkdir -p "$SCENSIM_ETC/ros/rosdep/sources.list.d"
        touch "$SCENSIM_ETC/ros/rosdep/sources.list.d/20-default.list";;
  update) ;;
  install) touch "$FAKE_STATE/rosdep-installed";;
  check) [ -f "$FAKE_STATE/rosdep-installed" ];;
esac
''',
    "sysctl": r'''
echo "sysctl $*" >> "$STUB_LOG"
conf="${@: -1}"
grep -E '^[a-z]' "$conf" | while IFS== read -r k v; do
  f="$SCENSIM_PROC_SYS/${k//.//}"; mkdir -p "$(dirname "$f")"; echo "$v" > "$f"
done
''',
    "vcs": r'''
echo "vcs $*" >> "$STUB_LOG"
mkdir -p src/autoware_sim_vehicle
''',
    "colcon": r'''
echo "colcon $* (autoware=${FAKE_AUTOWARE:-0})" >> "$STUB_LOG"
mkdir -p install; echo 'export FAKE_SIDE=1' > install/setup.bash
''',
    "ros2": r'''
echo "ros2 $*" >> "$STUB_LOG"
case "$1 $2" in
  "pkg prefix") [ -n "$FAKE_SIDE" ];;
  "run autoware_sim_profile") [ "$3" = setup_autoware_data ] && [ -d "$4" ] \
      && mkdir -p "$HOME/autoware_data/lidar_centerpoint";;
  *) exit 0;;
esac
''',
}


@pytest.fixture()
def host(env, tmp_path: Path):
    """A host with ROS Humble and Autoware 1.5.0 under fake roots, the release published."""
    publish(env)
    bin_ = tmp_path / "stubs"
    bin_.mkdir()
    for name, body in STUBS.items():
        f = bin_ / name
        f.write_text("#!/bin/bash\n" + body.lstrip())
        f.chmod(0o755)
    ros = fake_root(tmp_path, "ros", "humble", "FAKE_ROS")
    aw = fake_root(tmp_path, "autoware", "1.5.0", "FAKE_AUTOWARE")
    (aw / "1.5.0" / "data").mkdir()
    (tmp_path / "etc").mkdir()
    (tmp_path / "proc" / "net" / "core").mkdir(parents=True)
    (tmp_path / "proc" / "net" / "core" / "rmem_max").write_text("212992\n")
    state = tmp_path / "state"
    state.mkdir()
    log = tmp_path / "calls.log"
    log.touch()
    extra = {
        "PATH": f"{bin_}:/usr/bin:/bin",
        "SCENSIM_ROS_ROOT": str(ros),
        "SCENSIM_AUTOWARE_ROOT": str(aw),
        "SCENSIM_ETC": str(tmp_path / "etc"),
        "SCENSIM_PROC_SYS": str(tmp_path / "proc"),
        "SCENSIM_CONFIG_HOME": str(tmp_path / "config"),
        "FAKE_STATE": str(state),
        "STUB_LOG": str(log),
    }

    def run(*args: str, more: dict | None = None):
        # the release in place, with the Autoware side's pin as stage_sim.py writes it
        if not (env.home / "current").exists():
            r = env.run("install", ros=None, extra=extra)
            assert r.returncode == 0, r.stderr
            y = env.home / "current" / "sim" / "scensim-release.yaml"
            y.write_text(y.read_text() + f'  "src/autoware_sim_vehicle": "{ASV}"\n')
        log.write_text("")
        return env.run(*args, ros=None, extra={**extra, **(more or {})})

    h = type("Host", (), {})()
    h.run, h.log, h.tmp, h.aw, h.ros = run, log, tmp_path, aw, ros
    h.calls = lambda: log.read_text().splitlines()
    return h


def test_the_sysctl_text_is_the_repository_file() -> None:
    import sys
    sys.path.insert(0, str(SRC))
    from scensim_frontend.setup import SYSCTL_TEXT
    assert SYSCTL_TEXT == (REPO / "config" / "99-cyclonedds-max.conf").read_text()


def test_setup_on_a_fresh_host_one_sudo_then_ready(host) -> None:
    r = host.run("setup")
    assert r.returncode == 0, r.stderr
    calls = host.calls()
    assert calls.count("ROOT") == 1, calls  # one sudo, so one password prompt
    calls = calls[calls.index("ROOT") + 1:]  # what ran as root, and after
    order = [c.split()[0] + " " + c.split()[1] for c in calls if c.startswith(("apt-get", "rosdep"))]
    assert order[:4] == ["apt-get update", "apt-get install", "rosdep init", "rosdep update"]
    assert any(c.startswith("rosdep install --rosdistro humble --from-paths") and
               "/current/sim/share" in c
               for c in calls)
    assert any(c.startswith("sudo -u tester -H rosdep update") for c in calls)
    assert (host.tmp / "etc" / "sysctl.d" / "99-cyclonedds-max.conf").is_file()
    assert (host.tmp / "proc" / "net" / "core" / "rmem_max").read_text().strip() == "2147483647"
    # the Autoware side, over the detected (only) Autoware, from the release's pin
    ws = host.tmp / "autoware_side"
    assert f"version: {ASV}" in (ws / "autoware-side.repos").read_text()
    assert any(c.startswith("colcon build --packages-up-to tesla_model3_launch") and
               "autoware_sim_profile" in c and c.endswith("(autoware=1)") for c in calls)
    assert any("setup_autoware_data" in c and str(host.aw / "1.5.0" / "data") in c
               for c in calls)
    env_lines = [l for l in (host.tmp / "scensim.bash").read_text().splitlines()
                 if l.startswith("source")]
    assert len(env_lines) == 4
    assert env_lines[0] == f"source {host.ros}/humble/setup.bash"
    assert env_lines[1] == f"source {host.aw}/1.5.0/setup.bash"
    assert env_lines[2].endswith("/current/sim/setup.bash")
    assert env_lines[3] == f"source {ws}/install/setup.bash"
    hy = (host.tmp / "config" / "host.yaml").read_text()
    assert f"setup: {host.aw}/1.5.0/setup.bash" in hy
    assert "ready: `source ~/scensim.bash`" in r.stderr
    assert "scensim autoware tensorrt" in r.stderr  # 1.5.0's note


def test_a_second_setup_changes_nothing_and_asks_no_password(host) -> None:
    assert host.run("setup").returncode == 0
    before = (host.tmp / "scensim.bash").read_text()
    r = host.run("setup")
    assert r.returncode == 0, r.stderr
    calls = host.calls()
    assert "ROOT" not in calls and not any(c.startswith(("vcs", "colcon")) for c in calls)
    assert "system: nothing to do" in r.stderr
    assert "is built for this release" in r.stderr
    assert "scensim.bash: unchanged" in r.stderr
    assert (host.tmp / "scensim.bash").read_text() == before


def test_a_different_env_file_is_kept_unless_forced(host) -> None:
    mine = host.tmp / "scensim.bash"
    mine.write_text("source /my/own\n")
    r = host.run("setup")
    assert "kept, it differs" in r.stderr and "-source /my/own" in r.stderr
    assert mine.read_text() == "source /my/own\n"
    r = host.run("setup", "--force")
    assert r.returncode == 0, r.stderr
    assert (host.tmp / "scensim.bash.bak").read_text() == "source /my/own\n"
    assert "source /my/own" not in mine.read_text()


def test_an_existing_host_yaml_is_kept_and_its_gaps_named(host) -> None:
    hy = host.tmp / "config" / "host.yaml"
    hy.parent.mkdir()
    hy.write_text("# mine\nlogs: /fast/logs\n")
    r = host.run("setup")
    assert hy.read_text() == "# mine\nlogs: /fast/logs\n"
    assert f"autoware.setup: {host.aw}/1.5.0/setup.bash" in r.stderr
    r = host.run("setup", "--force")
    text = hy.read_text()
    assert "logs: /fast/logs" in text and f"setup: {host.aw}/1.5.0/setup.bash" in text
    assert (hy.parent / "host.yaml.bak").read_text().startswith("# mine")


def test_no_sudo_prints_the_script_and_exits_3(host) -> None:
    r = host.run("setup", "--no-sudo")
    assert r.returncode == 3
    assert "ROOT" not in host.calls()
    assert "sudo bash -c" in r.stderr and "rosdep init" in r.stderr
    assert "FIX  system packages (rosdep)" in r.stderr
    assert "FIX  DDS socket buffers" in r.stderr


def test_a_different_sysctl_file_is_kept(host) -> None:
    conf = host.tmp / "etc" / "sysctl.d" / "99-cyclonedds-max.conf"
    conf.parent.mkdir()
    conf.write_text("net.core.rmem_max=1\n")
    r = host.run("setup")
    assert "differs from scensim's; kept" in r.stderr
    assert conf.read_text() == "net.core.rmem_max=1\n"
    assert r.returncode == 3  # the check: buffers short


def test_several_autowares_none_sourced_is_refused(host) -> None:
    (host.aw / "2.1.0" / "data").mkdir(parents=True)
    (host.aw / "2.1.0" / "setup.bash").write_text("")
    r = host.run("setup")
    assert r.returncode == 3
    assert "several Autowares are installed and none is sourced" in r.stderr
    r = host.run("setup", more={"AMENT_PREFIX_PATH": f"{host.aw}/2.1.0/opt/x:/opt/other"})
    assert r.returncode == 0, r.stderr
    assert "Autoware 2.1.0" in r.stderr


def test_without_autoware_the_setup_is_the_simulation_side(host) -> None:
    r = host.run("setup", "--autoware", "none")
    assert r.returncode == 0, r.stderr
    assert "no Autoware (simulation only)" in r.stderr
    assert not any(c.startswith(("vcs", "colcon")) for c in host.calls())
    text = (host.tmp / "scensim.bash").read_text()
    assert str(host.aw) not in text and "autoware_side" not in text


def test_check_alone_writes_nothing(host) -> None:
    r = host.run("setup", "--check")
    assert r.returncode == 3
    assert not (host.tmp / "scensim.bash").exists()
    assert "ROOT" not in host.calls()


def test_autoware_side_refuses_a_workspace_of_other_packages(host) -> None:
    other = host.tmp / "autoware_side" / "src" / "my_pkg"
    other.mkdir(parents=True)
    r = host.run("autoware-side", "build")
    assert r.returncode == 3
    assert "holds other packages (my_pkg)" in r.stderr
    r = host.run("autoware-side", "build", "--workspace", str(host.tmp / "ws2"))
    assert r.returncode == 0, r.stderr
    assert (host.tmp / "ws2" / "install" / "setup.bash").is_file()


# --- carla fetch ------------------------------------------------------------------------------


class RangeServer:
    """Serves one blob with HTTP Range, ending every response after ``cut`` bytes, as CARLA's
    server ends a long download after about 1 GB."""

    def __init__(self, blob: bytes, cut: int) -> None:
        import http.server
        import threading
        self.requests = 0
        outer = self

        class H(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                outer.requests += 1
                start = 0
                rng = self.headers.get("Range")
                if rng:
                    start = int(rng.split("=")[1].rstrip("-"))
                body = blob[start:]
                self.send_response(206 if rng else 200)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body[:cut])
                self.close_connection = True

        self.httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()


def carla_blob(size: int = 300_000) -> bytes:
    import os as _os
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tar:
        for name, data in (("./CarlaUE4.sh", b"#!/bin/sh\n"), ("./big.bin", _os.urandom(size))):
            info = tarfile.TarInfo(name)
            info.size, info.mode = len(data), 0o755
            tar.addfile(info, io.BytesIO(data))
    return buf.getvalue()


def test_carla_fetch_resumes_checks_unpacks_and_writes_the_root(tmp_path: Path, monkeypatch,
                                                                 capsys) -> None:
    import sys
    sys.path.insert(0, str(SRC))
    from scensim_frontend import carla
    blob = carla_blob()
    srv = RangeServer(blob, cut=(len(blob) + 2) // 3)  # three responses
    monkeypatch.setitem(carla.CARLA, "9.9.9", {
        "url": f"{srv.url}/CARLA_9.9.9.tar.gz", "size": len(blob),
        "sha256": hashlib.sha256(blob).hexdigest()})
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("SCENSIM_CONFIG_HOME", str(tmp_path / "config"))
    assert carla.fetch(["fetch", "9.9.9"]) == 0
    assert srv.requests == 3 and "resuming (2 of 50)" in capsys.readouterr().err
    assert (tmp_path / "CARLA_9.9.9" / "CarlaUE4.sh").is_file()
    assert not (tmp_path / ".scensim-downloads").exists()
    assert f"root: {tmp_path}/CARLA_9.9.9" in (tmp_path / "config" / "host.yaml").read_text()
    assert carla.fetch(["fetch", "9.9.9"]) == 0 and srv.requests == 3  # in place
    srv.httpd.shutdown()


def test_a_server_that_does_not_resume_fails_the_fetch(tmp_path: Path, monkeypatch) -> None:
    import sys
    sys.path.insert(0, str(SRC))
    from scensim_frontend import carla
    from scensim_frontend.fetch import DownloadError
    blob = carla_blob()
    srv = RangeServer(blob, cut=len(blob) // 3)
    monkeypatch.setitem(carla.CARLA, "9.9.9", {
        "url": f"{srv.url}/x.tar.gz", "size": len(blob) * 2,   # more than it will ever send
        "sha256": "0" * 64})
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("SCENSIM_CONFIG_HOME", str(tmp_path / "config"))
    with pytest.raises(DownloadError) as e:
        carla.fetch(["fetch", "9.9.9"])
    assert "download it yourself from" in str(e.value)
    assert not list((tmp_path / ".scensim-downloads").glob("*.part"))
    srv.httpd.shutdown()


def test_carla_fetch_refuses_an_unknown_version(env) -> None:
    r = env.run("carla", "fetch", "0.9.15")
    assert r.returncode == 3
    assert "no CARLA '0.9.15' to fetch" in r.stderr and "0.9.16" in r.stderr


def test_brotli_short_needs_hub_2_and_brotli_below_1_2(monkeypatch) -> None:
    import sys
    sys.path.insert(0, str(SRC))
    from scensim_frontend import setup as s
    have = {}
    monkeypatch.setattr(s, "_version", lambda name: have.get(name, ()))
    assert not s.brotli_short()
    have.update({"huggingface_hub": (2, 4, 0), "brotli": (1, 0, 9)})
    assert s.brotli_short()
    have["brotli"] = (1, 2, 0)
    assert not s.brotli_short()
    have.update({"huggingface_hub": (0, 36), "brotli": (1, 0, 9)})
    assert not s.brotli_short()


def test_missing_models_are_named_with_the_downloader(host) -> None:
    import shutil
    shutil.rmtree(host.aw / "1.5.0" / "data")
    dl = host.tmp / "stubs" / "autoware-download-models-1-5-0"
    dl.write_text("#!/bin/bash\n")
    dl.chmod(0o755)
    r = host.run("setup")
    assert r.returncode == 3
    assert "FIX  Autoware's ML models in ~/autoware_data: sudo autoware-download-models-1-5-0, " \
        "then scensim setup" in r.stderr
