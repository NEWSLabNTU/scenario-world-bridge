"""cut_in: a pilot of kind `command`, a small agent on the swb_agent SDK.

swb starts it per run (``lifetime: run``) as

    python3 <project>/pilots/cut_in.py --relay <url> --entity cut_in --seed <n> --run-dir <dir>

It drives the scenario's ``cut_in`` car with CARLA's BasicAgent (the SDK's ``carla_basic``
adapter), with one policy of its own: when the scenario gives it its goal, it waits at the
stop line until the ego (CARLA role_name ``hero``) comes within ``trigger_m`` of it, then
turns into the ego's road just ahead of it. The trigger distance is ``--trigger-m`` (default
40 m) plus a jitter of up to 5 m drawn from the run's seed, so a suite's repetitions differ
and each one can be repeated exactly. What it chose is written to
``<run dir>/logs/pilot-cut_in/cut_in.json`` (kept with the logs: ``logging.keep``).

CARLA's endpoint and its PythonAPI come from the swb host settings of this project
(``carla: {root, host, port}``, as ``ros2 run swb seat show`` and the stack read them).
"""
import json
import math
import random
import sys
import time
from pathlib import Path

import swb_agent
from swb_agent import protocol as P
from swb_agent.adapters.carla_basic import CarlaBasicAgent

PROJECT = Path(__file__).resolve().parents[1]
JITTER_M = 5.0


def carla_settings(project: Path = PROJECT) -> dict:
    """carla_host, carla_port and carla_pythonapi from the project's host settings."""
    try:
        from swb import config
        carla = config.host_settings(project).get("carla") or {}
    except Exception:  # swb not installed, a settings file that does not parse: defaults
        return {}
    out = {"carla_host": carla.get("host") or "localhost",
           "carla_port": int(carla.get("port") or 2000)}
    if carla.get("root"):
        out["carla_pythonapi"] = str(Path(carla["root"]).expanduser() / "PythonAPI" / "carla")
    return out


class CutIn(CarlaBasicAgent):
    """BasicAgent that holds its goal until the ego is close enough, then goes."""

    name = "cut_in"

    def __init__(self, config=None):
        config = dict(config or {})
        for k, v in carla_settings().items():
            config.setdefault(k, v)
        config.setdefault("role_name", config.get("entity", "cut_in"))
        config.setdefault("target_speed_kmh", 25.0)
        super().__init__(config)
        self.ego_role = str(self.config.get("ego_role_name", "hero"))
        rng = random.Random(self.seed)
        self.trigger_m = (float(self.config.get("trigger_m", 40.0))
                          + rng.uniform(-JITTER_M, JITTER_M))
        self.pending = None  # (goal, waypoints) held until the ego is close
        self._ego = None
        self._write({"triggered": False})

    # -- the policy ------------------------------------------------------------------------
    def _write(self, what: dict):
        if not self.run_dir:
            return
        try:
            out = {"entity": self.entity, "seed": self.seed, "trigger_m": self.trigger_m, **what}
            d = Path(self.run_dir, "logs", f"pilot-{self.entity}")
            d.mkdir(parents=True, exist_ok=True)
            (d / "cut_in.json").write_text(json.dumps(out, indent=1) + "\n")
        except OSError as e:
            self.log.warning(f"cannot write cut_in.json in {self.run_dir}: {e}")

    def ego_distance(self):
        """Metres from this car to the ego, or None when either is not in CARLA (or the
        world is the SDK's fake one, which has no ego)."""
        world = getattr(self.world, "world", None)
        me = getattr(self.world, "vehicle", None)
        if world is None or me is None:
            return None
        if self._ego is None or not self._ego.is_alive:
            self._ego = next((a for a in world.get_actors().filter("vehicle.*")
                              if a.attributes.get("role_name") == self.ego_role), None)
            if self._ego is None:
                return None
        a, b = self._ego.get_location(), me.get_location()
        return math.hypot(a.x - b.x, a.y - b.y)

    def _go(self, why: str):
        goal, waypoints = self.pending
        self.pending = None
        self.log.info(f"cut in: {why}")
        self._write({"triggered": True, "why": why, "t_wall": time.time()})
        super().on_goal(goal, waypoints)

    def _on_world_tick(self):
        if self.pending is not None:
            d = self.ego_distance()
            if d is None and not hasattr(self.world, "world"):
                self._go("no ego in this world (fake)")
            elif d is not None and d <= self.trigger_m:
                self._go(f"ego {d:.1f} m away (trigger {self.trigger_m:.1f} m)")
        super()._on_world_tick()

    # -- hooks -----------------------------------------------------------------------------
    def on_reset(self, ep):
        self.pending = None
        self._ego = None
        super().on_reset(ep)

    def on_goal(self, goal, waypoints):
        with self._ctl:
            self.pending = (goal, list(waypoints))
            self.mode = "hold"
        self.report(P.DRIVING, detail=f"waiting for the ego within {self.trigger_m:.0f} m")
        self.log.info(f"goal ({goal['x']:.1f}, {goal['y']:.1f}) held until the ego is within"
                      f" {self.trigger_m:.1f} m")

    def on_stop(self):
        self.pending = None
        super().on_stop()


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    # --trigger-m is this pilot's own; the rest are the SDK's (--relay, --entity, --seed,
    # --run-dir, --config, --log-level)
    trigger = None
    if "--trigger-m" in argv:
        i = argv.index("--trigger-m")
        trigger = float(argv[i + 1])
        del argv[i:i + 2]

    class Configured(CutIn):
        def __init__(self, config=None):
            config = dict(config or {})
            if trigger is not None:
                config["trigger_m"] = trigger
            super().__init__(config)

    Configured.__name__ = "CutIn"
    return swb_agent.main(Configured, argv)


if __name__ == "__main__":
    sys.exit(main())
