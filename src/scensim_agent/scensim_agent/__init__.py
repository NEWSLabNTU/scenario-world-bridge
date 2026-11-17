"""scensim_agent: drive scensim scenarios with any autopilot.

    import scensim_agent

    class MyAgent(scensim_agent.Agent):
        def on_reset(self, ep): ...
        def on_goal(self, goal, waypoints): ...
        def on_stop(self): ...

    if __name__ == "__main__":
        scensim_agent.main(MyAgent)

Roadmap 030; spec in docs/design/user-workflow.md ("The agent protocol (I3)").
"""

import argparse
import json
import logging
import os
import signal
import threading

from . import protocol
from .agent import Agent, Episode, Failed
from .client import Client, parse_relay
from .commander import Commander
from .protocol import (ARRIVED, DRIVING, EMERGENCY, FAULT_NONE, IDLE, INITIALIZING,
                       MINIMAL_RISK_MANEUVER, PLANNING, READY, STOPPED, UNAVAILABLE)

__all__ = ["Agent", "Episode", "Failed", "Client", "Commander", "main", "parse_args",
           "load_config", "protocol",
           "UNAVAILABLE", "INITIALIZING", "IDLE", "PLANNING", "READY", "DRIVING", "ARRIVED",
           "STOPPED", "FAULT_NONE", "MINIMAL_RISK_MANEUVER", "EMERGENCY"]

DEFAULT_RELAY = "tcp://localhost:5560"


def load_config(path):
    """A JSON or YAML file (YAML needs PyYAML) as a dict."""
    with open(path) as f:
        text = f.read()
    if path.endswith((".yaml", ".yml")):
        import yaml  # optional dependency, only for YAML configs
        return yaml.safe_load(text) or {}
    return json.loads(text)


def parse_args(agent_class, argv=None):
    """``(args, config)`` from an agent's command line; the config holds what the agent's
    constructor is given: the --config file, then ``entity``, ``seed`` and ``run_dir`` from
    the options when they are given (a ``command`` pilot is always given --relay, --entity
    and --seed, and --run-dir with a run)."""
    parser = argparse.ArgumentParser(description=f"scensim_agent: {agent_class.__name__}")
    parser.add_argument("--relay", default=None)
    parser.add_argument("--entity", default=None)
    parser.add_argument("--config", default=None)
    parser.add_argument("--seed", type=int, default=None,
                        help="the run's seed (Agent.seed); a pilot's runs get seed + k")
    parser.add_argument("--run-dir", default=None,
                        help="the run's directory (Agent.run_dir), for the agent's own files")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    config = load_config(args.config) if args.config else {}
    if args.entity:
        config["entity"] = args.entity
    if args.seed is not None:
        config["seed"] = args.seed
    if args.run_dir:
        config["run_dir"] = args.run_dir
    return args, config


def main(agent_class, argv=None):
    """Command line entry point: build `agent_class(config)` and run it until signalled.

    --relay (default $SCENSIM_RELAY, else tcp://localhost:5560), --entity (default the
    config's `entity`, else "ego"), --config (JSON or YAML, passed to the constructor),
    --seed and --run-dir (the run's, as a pilot is given them: `Agent.seed`, `Agent.run_dir`).
    """
    args, config = parse_args(agent_class, argv)
    logging.basicConfig(level=args.log_level.upper(),
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    relay = args.relay or config.get("relay") or os.environ.get("SCENSIM_RELAY") or DEFAULT_RELAY
    host, port = parse_relay(relay)
    agent = agent_class(config)
    client = Client(agent, host, port)
    done = threading.Event()

    def stop(*_):
        done.set()
        client.stop()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    logging.getLogger("scensim_agent").info(
        f"{agent_class.__name__} for entity {agent.entity!r}: relay tcp://{host}:{port}")
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()
    while not done.wait(0.5):
        pass
    thread.join(timeout=5.0)
    agent.close()
    return 0
