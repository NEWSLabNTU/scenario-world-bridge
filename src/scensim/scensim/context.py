"""A run's storyboard log (roadmap 077, G33): SSv2's ``/simulation/context`` as state changes.

    python3 -m scensim.context <out.jsonl>      # in the scenario domain, until SIGINT

The interpreter publishes the whole storyboard as one JSON snapshot every frame (about 4.5 KB
to 210 KB at 20 Hz, roadmap 033); this keeps only what changed, one JSON object per line:
``{"ev": "state", "sim": <sim s>, "element": "<story>/<act>/<group>/<maneuver>/<event>
[/<action>]", "from": <state or null>, "to": <state>}`` -- the log incident_judge reads
(``--context``). The runner starts it, as a run's recorder, when a judge ``needs: [context]``,
and writes ``context.jsonl`` into the run directory.
"""
import json
import signal
import sys

LEVELS = ("Story", "Act", "ManeuverGroup", "Maneuver", "Event", "Action")


def elements(data) -> dict:
    """``{path: currentState}`` of every storyboard element in one snapshot."""
    out = {}

    def as_list(v):
        return v if isinstance(v, list) else [v] if isinstance(v, dict) else []

    def walk(node, depth, prefix):
        if depth == len(LEVELS):
            return
        for child in as_list(node.get(LEVELS[depth])):
            name = child.get("name")
            if not isinstance(name, str):
                continue
            path = f"{prefix}/{name}" if prefix else name
            if child.get("currentState") is not None:
                out[path] = child["currentState"]
            walk(child, depth + 1, path)

    board = (((data or {}).get("OpenSCENARIO") or {}).get("Storyboard")) or {}
    walk(board, 0, "")
    return out


class Changes:
    """The state changes between consecutive snapshots."""

    def __init__(self):
        self.last = {}

    def feed(self, data, sim) -> list:
        now = elements(data)
        out = []
        for path, state in now.items():
            prev = self.last.get(path)
            if state != prev:
                out.append({"ev": "state", "sim": sim, "element": path, "from": prev, "to": state})
        self.last = now
        return out


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print("usage: python3 -m scensim.context <out.jsonl>", file=sys.stderr)
        return 2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from openscenario_interpreter_msgs.msg import Context

    changes = Changes()
    out = open(argv[0], "a", buffering=1)

    class Recorder(Node):
        def __init__(self):
            super().__init__("scensim_context_log")
            qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(Context, "/simulation/context", self.on_context, qos)

        def on_context(self, m):
            if not m.data:
                return
            try:
                data = json.loads(m.data)
            except ValueError:
                return
            sim = m.time if m.time == m.time else None   # NaN before the start
            for line in changes.feed(data, sim):
                out.write(json.dumps(line) + "\n")

    rclpy.init()
    node = Recorder()
    signal.signal(signal.SIGTERM, lambda *a: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):  # noqa: BLE001 (shutdown from a signal)
        pass
    out.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
