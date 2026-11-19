"""078 P2g: the experiment patches, from the environment (p2g_screen.py's --pre module; czp_rows_check.py imports it
for --guard-extra and --cross-probe).

- P2G_GUARD_EXTRA (s): every czp car takes guard_extra (the guard counted from a rest is dwell + the other's launch
  dead time + see_delay + guard_extra, czp.convention.rest_guard);
- P2G_PROBE_WATCH (s): probe_watch, the probe's watch (czp: 0.8 s; P2g measured the seen start at up to 1.1 s
  after the other's first motion);
- P2G_CROSS_PROBE=1: the crossing's row probes a go from a mutual rest (round 2, the cycle, the last resort, the rest
  tie) as the all-way stop does: creep about probe_creep_dist, watch probe_watch, then go (czp.probe)."""
import os

import czp_adapter as CA

_params_for = CA.params_for
_conventions_for = CA.conventions_for


def params_for(*args, **kw):
    kv = {"guard_extra": float(os.environ.get("P2G_GUARD_EXTRA", "0"))}
    if os.environ.get("P2G_PROBE_WATCH"):
        kv["probe_watch"] = float(os.environ["P2G_PROBE_WATCH"])
    return _params_for(*args, **kw).override(source="078 P2g", **kv)


def conventions_for(*args, **kw):
    c = _conventions_for(*args, **kw)
    if os.environ.get("P2G_CROSS_PROBE") == "1":
        c = dict(c)
        c["cross_unsignalled"] = dict(c["cross_unsignalled"], probe_round2=True)
    return c


CA.params_for = params_for
CA.conventions_for = conventions_for
