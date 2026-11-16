"""Tests for scripts/ego_used.bash (roadmap 077, G64): a separate ego or background-AV stack
serves one scenario. Stacks are stood in for by processes whose command line reads as a
play_launch's (`bash -c 'sleep ...; :' play_launch launch --web-addr ...`), on ports no stack
uses; the markers go to a temporary directory.

    python3 -m pytest scripts/test_ego_used.py
"""

import os
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

LIB = Path(__file__).resolve().parent / "ego_used.bash"
BASE = 59100  # a web port base no stack here uses: ego +2, stack +4


def fake_stack(*args):
    """a process whose command line is `... play_launch launch <args>`"""
    # its own session, so tearDown ends bash and its `sleep` together (killing bash alone
    # left the sleep to init for its 300 s)
    p = subprocess.Popen(["bash", "-c", "sleep 300; :", "play_launch", "play_launch", "launch",
                          *args, "end"], start_new_session=True)
    time.sleep(0.05)
    return p


def web(port):
    return ["--web-addr", f"127.0.0.1:{port}"]


class EgoUsed(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name) / "used"
        self.procs = []

    def tearDown(self):
        for p in self.procs:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            p.wait()
        self.tmp.cleanup()

    def stack(self, *args):
        p = fake_stack(*args)
        self.procs.append(p)
        return p

    def sh(self, script):
        r = subprocess.run(["bash", "-c", f'source "{LIB}"; ego_used_dir="{self.dir}"; {script}'],
                           capture_output=True, text=True)
        return r.returncode, r.stdout.strip(), r.stderr.strip()

    def used(self, pid):
        return self.sh(f"ego_used_check {pid}")[0] == 0

    def test_a_fresh_stack_is_not_used_and_a_marked_one_is(self):
        ego = self.stack(*web(BASE + 2))
        self.assertFalse(self.used(ego.pid))
        self.assertEqual(self.sh(f"ego_used_mark {ego.pid}")[0], 0)
        self.assertTrue(self.used(ego.pid))

    def test_a_pid_reused_after_a_restart_is_fresh(self):
        ego = self.stack(*web(BASE + 2))
        rc, start, _ = self.sh(f"ego_used_start {ego.pid}")
        self.assertEqual(rc, 0)
        # the marker of an earlier process that had this pid: another start time
        self.dir.mkdir(parents=True)
        (self.dir / f"{ego.pid}-{int(start) - 1}").touch()
        self.assertFalse(self.used(ego.pid))
        self.assertFalse((self.dir / f"{ego.pid}-{int(start) - 1}").exists())  # removed

    def test_a_stale_marker_whose_process_is_gone_is_removed(self):
        gone = fake_stack(*web(BASE + 2))
        self.sh(f"ego_used_mark {gone.pid}")
        os.killpg(gone.pid, signal.SIGKILL)  # the stack and its sleep (its own session)
        gone.wait()
        self.assertEqual(len(list(self.dir.iterdir())), 1)
        ego = self.stack(*web(BASE + 2))
        self.assertFalse(self.used(ego.pid))
        self.assertEqual(list(self.dir.iterdir()), [])
        self.assertFalse(self.used(gone.pid))  # a pid that is gone is never used

    def test_the_first_started_play_launch_of_a_port_names_the_stack(self):
        wrapper = self.stack(*web(BASE + 2))
        time.sleep(0.02)
        self.stack(*web(BASE + 2))  # the real binary, started by the wrapper
        self.stack(*web(BASE + 3))  # another port
        self.assertEqual(self.sh(f"ego_used_pid {BASE + 2}")[1], str(wrapper.pid))
        self.assertEqual(self.sh(f"ego_used_pid {BASE + 9}")[0], 1)

    def test_which_separate_stacks_a_scenario_runs_against(self):
        with tempfile.NamedTemporaryFile("w", suffix=".xosc", delete=False) as f:
            f.write('<ScenarioObject name="ego"/><ScenarioObject name="bg_x_1"/>')
        sc = f.name
        ego = self.stack(*web(BASE + 2))
        bg = self.stack(*web(BASE + 3), "swb_launch", "background_av.launch.xml", "entity:=bg_x_1")
        self.stack(*web(BASE + 7), "swb_launch", "background_av.launch.xml", "entity:=bg_x_9")
        # no stack on +4 yet: stack_up starts it without an ego, so the separate one serves
        out = self.sh(f'ego_used_separate "{sc}" 1 {BASE}')[1].splitlines()
        self.assertIn(f"ego {ego.pid}", out)
        self.assertIn(f"bg bg_x_1 {bg.pid}", out)
        self.assertFalse(any("bg_x_9" in l for l in out))  # the scenario has no bg_x_9
        # no ego in the scenario: only the background AV
        self.assertEqual(self.sh(f'ego_used_separate "{sc}" 0 {BASE}')[1], f"bg bg_x_1 {bg.pid}")
        # a stack with its own ego on +4: the separate one does not serve
        own = self.stack(*web(BASE + 4), "swb_launch", "carla.launch.xml", "map_path:=/m")
        self.assertNotIn(f"ego {ego.pid}", self.sh(f'ego_used_separate "{sc}" 1 {BASE}')[1])
        own.kill()
        own.wait()
        self.stack(*web(BASE + 4), "swb_launch", "carla.launch.xml", "ego_launch:=none")
        self.assertIn(f"ego {ego.pid}", self.sh(f'ego_used_separate "{sc}" 1 {BASE}')[1])
        Path(sc).unlink()

    def test_refuse_names_the_stack_and_mark_all_marks_each(self):
        ego = self.stack(*web(BASE + 2))
        bg = self.stack(*web(BASE + 3), "background_av.launch.xml", "entity:=bg_x_1")
        listing = f"ego {ego.pid}\nbg bg_x_1 {bg.pid}"
        self.assertEqual(self.sh(f'ego_used_refuse "{listing}"')[0], 0)
        self.sh(f'ego_used_mark_all "{listing}"')
        self.assertTrue(self.used(ego.pid) and self.used(bg.pid))
        rc, _, err = self.sh(f'ego_used_refuse "{listing}"')
        self.assertEqual(rc, 1)
        self.assertIn(f"this ego stack already ran a scenario (pid {ego.pid}); restart it: "
                      "`just ego-av ...` (roadmap 077, G64)", err)
        rc, _, err = self.sh(f'ego_used_refuse "bg bg_x_1 {bg.pid}"')
        self.assertEqual(rc, 1)
        self.assertIn("`just bg-av bg_x_1 ...`", err)
        self.assertEqual(self.sh('ego_used_refuse ""')[0], 0)  # no separate stack


if __name__ == "__main__":
    unittest.main()
