"""SSv2's run settings: what a run hands ``scenario.launch.xml`` besides the scenario.

docs/design/projects-and-pilots.md, "Run settings". A project gives defaults in ``swb.yaml``
(``ssv2:``), a suite its own (``ssv2:``), a suite's scenario item its own
(``scenarios: [{path: x.xosc, ssv2: {...}}]``), and the command line or ``run_one`` the last
word (``--timeout``, ``overrides["ssv2"]``); each layer replaces the keys it names.

    ssv2:
      global_timeout: 400            # s, wall clock per scenario (per variant)
      frame_rate: 20                 # SSv2 frames per simulated second (100: a 10 ms step)
      global_real_time_factor: 1.0   # simulated seconds per real second
      initialize_duration: 120       # s, the concealer's budget from start to engage
      managed_ego: true              # false: the scenario's ego is handed to no pilot

``managed_ego: false`` is SSv2's own switch: the concealer does not drive the ego through the
relay, so no pilot drives it and the run waits for none (an ego that stands, or that the
scenario moves itself; its scenario must not use autonomy actions such as RoutingAction).
"""

KEYS = ("global_timeout", "frame_rate", "global_real_time_factor", "initialize_duration",
        "managed_ego")
# scenario.launch.xml's defaults (the runner passes only what a layer names, and its timeout)
DEFAULTS = {"global_timeout": 400, "frame_rate": 20, "global_real_time_factor": 1.0,
            "initialize_duration": 120, "managed_ego": True}
POSITIVE = ("global_timeout", "frame_rate", "global_real_time_factor", "initialize_duration")


def _positive(v) -> bool:
    return not isinstance(v, bool) and isinstance(v, (int, float)) and v > 0


def problems(value, where="ssv2") -> list:
    """``(where, message)`` for every way an ``ssv2:`` mapping is wrong."""
    if value is None:
        return []
    if not isinstance(value, dict):
        return [(where, "is not a mapping: {global_timeout: 600, frame_rate: 20}")]
    out = []
    for k, v in value.items():
        if k not in KEYS:
            out.append((where, f"unknown key {k} (use only {', '.join(KEYS)})"))
        elif k in POSITIVE and not _positive(v):
            out.append((f"{where}.{k}", f"{v!r} is not a positive number"))
        elif k == "managed_ego" and not isinstance(v, bool):
            out.append((f"{where}.{k}", f"{v!r} is not true or false"))
    return out


def merge(*layers) -> dict:
    """The layers in order, later keys replacing earlier ones; None and {} skipped."""
    out = {}
    for layer in layers:
        if isinstance(layer, dict):
            out.update({k: v for k, v in layer.items() if k in KEYS})
    return out


def managed(settings: dict) -> bool:
    return (settings or {}).get("managed_ego", True) is not False


def _text(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    return str(v)   # 20 stays 20, 1.0 stays 1.0: SSv2 reads each as the type it was given


def launch_args(settings: dict) -> dict:
    """``scenario.launch.xml`` arguments for the keys ``settings`` names, as text."""
    return {k: _text(settings[k]) for k in KEYS if k in (settings or {})}
