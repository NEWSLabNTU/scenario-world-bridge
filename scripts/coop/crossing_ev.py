#!/usr/bin/env python3
"""Roadmap 078 Phase 2 on the crossing: the tables, from crossing_aw.py's runs and crossing_aw_bag.py's facts.

- **outcomes** per cell (rule, dt) and arm: valid runs, who entered Z first, contact, PET, clearance,
  2D TTC, each car's wait before Z, both stopped;
- **RTC** per run with the node on: its decision changes (non-stock), how many of its commands the
  intersection module's status showed within 1 s, whether each go removed the module's stop factor and
  B moved, whether each deny put a stop factor back, mode back to auto at the end, the node's cycle
  (p50 / p99) and the watchdog's trips (from the ego's play_log);
- **EV4** the role-time error e of A's nominal arrival (s / v) from Autoware's tracks against the truth,
  by A's distance to Z in 10 m bins to 60 m, on the pre-registered split (seeds 0-1 fit, 2-3 held out):
  bias (median), e99 centred, held-out coverage; first tracked distance; the stop instant;
- **EV5** the yield test (filtered acceleration <= -0.5 m/s^2 held 0.5 s) read on the tracks, against
  the same test on A's true speed, per run: agreement and lag.

  scripts/coop/crossing_ev.py --out <grid dir>     -> <dir>/tables.md, ev4.json, ev5.json, rtc.json
  scripts/coop/crossing_ev.py --out <grid dir> --fit-from <earlier grid dir> [--e 0.55]
      EV4 on a fresh held-out round (078 P2b): the FIT split is the earlier grid's seeds 0-1, every run of this
      grid is held out; also the held-out share within the node's e of the fit bias, and the breakdown per
      side and rule (-> ev4_heldout.json)
"""

import argparse
import csv
import glob
import json
import math
import re
import statistics as stt
from collections import defaultdict
from pathlib import Path

STOCK_EQ = ("clear", "cleared", "contested", "inside", "no conflict", "contract")
BINS = [(0, 10), (10, 20), (20, 30), (30, 40), (40, 50), (50, 60)]
FIT, HELD = (0, 1), (2, 3)
YIELD_A, YIELD_HOLD, SLOPE_W = -0.5, 0.5, 1.0


def q(xs, p):
    xs = sorted(xs)
    if not xs:
        return None
    k = (len(xs) - 1) * p
    i = int(math.floor(k))
    j = min(i + 1, len(xs) - 1)
    return xs[i] + (xs[j] - xs[i]) * (k - i)


def r(x, n=3):
    return None if x is None else round(x, n)


def load_runs(base):
    last = {}
    for line in open(base / "runs.jsonl"):
        if line.strip():
            x = json.loads(line)
            last[x["tag"]] = x
    return last


# --- EV4 ----------------------------------------------------------------------------------------
def track_errors(base, min_age=0.0):
    """(seed, tag, s_true, e) per track message of A, EV4's filters."""
    rows = list(csv.DictReader(open(base / "bagfacts" / "tracks.csv")))
    born = {}
    for x in rows:
        if x["kind"] == "track":
            k = (x["tag"], x["uid"])
            born[k] = min(born.get(k, 1e9), float(x["tr"]))
    out = []
    for x in rows:
        if x["kind"] != "track" or float(x["tr"]) - born[(x["tag"], x["uid"])] < min_age:
            continue
        s_true, v_true = -float(x["u_true"]), float(x["v_true"])
        s_trk, v_trk = -float(x["u_trk"]), float(x["v_trk"])
        if s_true <= 0 or v_true < 2.0 or s_true / v_true > 10.0 or v_trk < 0.5:
            continue
        out.append((int(x["seed"]), x["tag"], s_true, s_trk / v_trk - s_true / v_true))
    return out


def ev4_heldout(base, fit_base, runs, e_node, min_age=0.0):
    """078 P2b: the earlier round's fit split (seeds 0-1) against every message of this round (held out)."""
    fit = [(s, e) for seed, _, s, e in track_errors(fit_base, min_age) if seed in FIT]
    held = [(tag, s, e) for _, tag, s, e in track_errors(base, min_age)]
    meta = {t: (x.get("side", "left"), x.get("rule"), x.get("arm")) for t, x in runs.items()}

    def stats(f, h):
        if not f:
            return None
        b = stt.median(f)
        e99 = q([abs(v - b) for v in f], 0.99)
        return {"n_fit": len(f), "bias_fit": r(b), "e99_fit": r(e99), "n_held": len(h),
                "held_bias": r(stt.median(h)) if h else None, "held_e99": r(q([abs(v - b) for v in h], 0.99)) if h else None,
                "held_cover_e99fit": r(sum(abs(v - b) <= e99 for v in h) / len(h)) if h else None,
                "held_cover_e_node": r(sum(abs(v - b) <= e_node for v in h) / len(h)) if h else None}
    out = {"e_node": e_node, "min_age": min_age, "bins": []}
    for lo, hi in BINS:
        st = stats([e for s, e in fit if lo <= s < hi], [e for _, s, e in held if lo <= s < hi])
        out["bins"].append({"bin": f"{lo}-{hi}", **(st or {"n_fit": 0, "n_held": sum(lo <= s < hi for _, s, _ in held)})})
    f30 = [e for s, e in fit if s < 30]
    out["pool_0_30"] = stats(f30, [e for _, s, e in held if s < 30])
    groups = defaultdict(list)
    for tag, s, e in held:
        if s < 30:
            side, rule, _ = meta.get(tag, ("?", "?", "?"))
            groups[f"{side}/{rule}"].append(e)
    out["pool_0_30_by"] = {k: stats(f30, v) for k, v in sorted(groups.items())}
    return out


def ev4(base, runs, min_age=0.0):
    """min_age: only tracks at least this old (s since the uuid's first message): czp's track contract
    (0.5 s) makes a younger track a phantom, so e of what the role reads is e over tracks >= 0.5 s."""
    rows = list(csv.DictReader(open(base / "bagfacts" / "tracks.csv")))
    born = {}
    for x in rows:
        if x["kind"] == "track":
            k = (x["tag"], x["uid"])
            born[k] = min(born.get(k, 1e9), float(x["tr"]))
    rows = [x for x in rows if x["kind"] == "track" and float(x["tr"]) - born[(x["tag"], x["uid"])] >= min_age]
    by = {"fit": defaultdict(list), "held": defaultdict(list)}
    allv = {"fit": [], "held": []}
    for x in rows:
        if x["kind"] != "track":
            continue
        s_true, v_true = -float(x["u_true"]), float(x["v_true"])
        s_trk, v_trk = -float(x["u_trk"]), float(x["v_trk"])
        if s_true <= 0 or v_true < 2.0 or s_true / v_true > 10.0 or v_trk < 0.5:
            continue
        e = s_trk / v_trk - s_true / v_true
        split = "fit" if int(x["seed"]) in FIT else "held" if int(x["seed"]) in HELD else None
        if split is None:
            continue
        for lo, hi in BINS:
            if lo <= s_true < hi:
                by[split][(lo, hi)].append(e)
                allv[split].append((s_true, e, x["tag"]))
    out = {"bins": []}
    for lo, hi in BINS:
        f, h = by["fit"][(lo, hi)], by["held"][(lo, hi)]
        bias = stt.median(f) if f else None
        e99 = q([abs(v - bias) for v in f], 0.99) if f else None
        held_e99 = q([abs(v - bias) for v in h], 0.99) if (h and bias is not None) else None
        cover = (sum(abs(v - bias) <= e99 for v in h) / len(h)) if (h and e99 is not None) else None
        out["bins"].append({"bin": f"{lo}-{hi}", "n_fit": len(f), "bias": r(bias), "e99": r(e99), "sd": r(stt.pstdev(f)) if len(f) > 1 else None,
                            "n_held": len(h), "held_bias": r(stt.median(h)) if h else None, "held_e99": r(held_e99),
                            "held_cover": r(cover)})
    # pooled over 0-30 m (the band's range: a tie is read when both cars are a few seconds out)
    for name, lo, hi in (("pool_0_30", 0, 30), ("pool_0_60", 0, 60)):
        f = [e for s, e, _ in allv["fit"] if lo <= s < hi]
        h = [e for s, e, _ in allv["held"] if lo <= s < hi]
        if f:
            b = stt.median(f)
            e99 = q([abs(v - b) for v in f], 0.99)
            he = q([abs(v - b) for v in h], 0.99) if h else None
            out[name] = {"n_fit": len(f), "bias": r(b), "e99": r(e99), "n_held": len(h), "held_e99": r(he),
                         "held_cover": r(sum(abs(v - b) <= e99 for v in h) / len(h)) if h else None,
                         "raw_e99_uncentred": r(q([abs(v) for v in f], 0.99))}
    return out


def first_tracked(base, runs):
    out = []
    for tag, x in runs.items():
        p = base / "bagfacts" / f"{tag}.json"
        if not p.exists() or "error" in x:
            continue
        b = json.loads(p.read_text())
        ft = (b.get("first_tracked") or {}).get("track")
        st = b.get("stop_true")
        sk = b.get("stop_trk")
        out.append({"tag": tag, "first_dist_z": ft and ft["dist_z"], "first_dist_ego": ft and ft["dist_ego"],
                    "first_tr": ft and ft["tr"], "stop_true": st, "stop_trk": sk,
                    "stop_err": None if (st is None or sk is None or st < 0) else round(sk - st, 3)})
    return out


# --- EV5 ----------------------------------------------------------------------------------------
def slope(series, t, w=SLOPE_W):
    pts = [(a, b) for a, b in series if t - w <= a <= t]
    if len(pts) < 5 or pts[-1][0] - pts[0][0] < 0.5 * w:
        return None
    n = len(pts)
    mt = sum(p[0] for p in pts) / n
    mv = sum(p[1] for p in pts) / n
    den = sum((p[0] - mt) ** 2 for p in pts)
    return sum((p[0] - mt) * (p[1] - mv) for p in pts) / den if den > 0 else None


def yield_onset(series, t_end):
    """First t at which the filtered acceleration has been <= YIELD_A for YIELD_HOLD s (before t_end)."""
    since = None
    ts = [p[0] for p in series]
    for t in ts:
        if t > t_end:
            break
        a = slope(series, t)
        if a is not None and a <= YIELD_A:
            if since is None:
                since = t
            elif t - since >= YIELD_HOLD:
                return since
        else:
            since = None
    return None


def ev5(base, runs):
    trk = defaultdict(list)
    for x in csv.DictReader(open(base / "bagfacts" / "tracks.csv")):
        if x["kind"] == "track" and float(x["u_true"]) < 0:
            trk[x["tag"]].append((float(x["tr"]), float(x["v_trk"])))
    out = []
    for tag, x in runs.items():
        if "error" in x or tag not in trk:
            continue
        rows = [json.loads(line) for line in open(base / tag / "coop_a.jsonl") if line.strip()]
        ev = {q_["event"]: q_ for q_ in rows if "event" in q_}
        if "lock" not in ev:
            continue  # a run being redone
        t_lock = ev["lock"]["t"]
        tru = [(q_["t"] - t_lock, q_["va"]) for q_ in rows if "phase" in q_ and q_["ua"] < 0 and q_["t"] >= t_lock - 2]
        t_end = x.get("tA_in") if x.get("tA_in") is not None else 1e9
        s_trk = sorted(trk[tag])
        y_true = yield_onset([p for p in tru if p[0] >= 0], t_end)
        y_trk = yield_onset([p for p in s_trk if p[0] >= -1], t_end)
        out.append({"tag": tag, "rule": x["rule"], "dt": x["dt"], "arm": x["arm"], "yield_true": r(y_true, 2), "yield_trk": r(y_trk, 2),
                    "agree": (y_true is None) == (y_trk is None),
                    "lag": None if (y_true is None or y_trk is None) else round(y_trk - y_true, 2),
                    "a_first": x.get("first") == "A"})
    return out


# --- RTC ----------------------------------------------------------------------------------------
def ego_logdir(rd):
    m = re.search(r"Launcher log: (\S+)/play_launch.log", (rd / "ego.log").read_text(errors="replace"))
    return Path(m.group(1)) if m else None


def rtc(base, runs):
    out = []
    for tag, x in runs.items():
        if x["arm"] != "on" or "error" in x:
            continue
        b = json.loads((base / "bagfacts" / f"{tag}.json").read_text())
        if "error" in b:
            out.append({"tag": tag, "error": b["error"]})
            continue
        eff = b.get("effects", [])
        fac = b.get("factor_changes", [])
        # a decision the node acts on (RTC): not stock and not stock-equivalent (a clear band, a cleared or contested
        # other: what stock Autoware does anyway, convention.decide's stock_equivalent)
        eff = [e for e in eff if e["reason"] not in STOCK_EQ]
        gos = [e for e in eff if e["decision"] == "go"]
        denies = [e for e in eff if e["decision"] in ("wait", "yield", "wait_second_round") and e["want"] == "DEACTIVATE"]
        ch = b.get("rtc_changes", [])
        switches = sum(1 for a_, b_ in zip(ch, ch[1:]) if a_["auto"] != b_["auto"])
        def safe_at(t):
            cur = None
            for c in ch:
                if c["tr"] <= t + 1e-6:
                    cur = c["safe"]
            return cur

        unsafe_go = sum(1 for e in gos if safe_at(e["tr"]) is False)  # a go approved while the module judged it unsafe

        def stop_factor_at(t):
            cur = None
            for f in fac:
                if f["tr"] <= t + 1e-6:
                    cur = f["behavior"]
            return cur

        go_removed = sum(1 for e in gos if stop_factor_at(e["tr"] + 0.5) is None)
        deny_stop = sum(1 for e in denies if stop_factor_at(e["tr"] + 0.5) is not None)
        ld = ego_logdir(base / tag)
        wd_trips, cyc = None, None
        if ld and (ld / "node").exists():
            w = (ld / "node/interaction_strategy_watchdog/err")
            if w.exists():
                txt = re.sub(r"\x1b\[[0-9;]*m", "", w.read_text(errors="replace"))
                wd_trips = {"total": txt.count("no heartbeat for"), "held": len(re.findall(r"no heartbeat for [0-9.]+ s \(it held \['", txt))}
            n = ld / "node/interaction_strategy/err"
            if n.exists():
                m = re.findall(r'\{"cycles": \d+, "p50_ms": [0-9.]+, "p99_ms": [0-9.]+[^\n]*\}', n.read_text(errors="replace"))
                cyc = json.loads(m[-1]) if m else None
        nonstock = [d for d in b["decisions"] if d["decision"] != "stock"]
        out.append({"tag": tag, "dt": x["dt"], "rule": x["rule"], "seed": x["seed"],
                    "decisions": len(b["decisions"]), "non_stock": len(nonstock),
                    "kinds": sorted({f'{d["decision"]}:{d["reason"]}' for d in nonstock}),
                    "commands": len(eff), "commands_seen_1s": sum(e["rtc_seen"] is not None for e in eff),
                    "go": len(gos), "go_removed_stop": go_removed, "go_on_unsafe": unsafe_go, "mode_switches": switches, "deny": len(denies), "deny_stop_factor": deny_stop,
                    "final_auto": b.get("rtc_final_auto"), "hb_max_gap": b["heartbeat"]["max_gap"],
                    "watchdog": wd_trips, "cycle": cyc,
                    "first": x.get("first"), "contact": x.get("contact"), "pet": x.get("pet")})
    return out


# --- outcomes -----------------------------------------------------------------------------------
def outcomes(runs):
    cells = defaultdict(list)
    for x in runs.values():
        if "error" in x:
            continue
        cells[(x.get("side", "left"), x["rule"], x["dt"], x["arm"])].append(x)
    out = []
    for (side, rule, dt, arm), xs in sorted(cells.items()):
        v = [x for x in xs if x.get("valid")]
        out.append({"side": side, "rule": rule, "dt": dt, "arm": arm, "n": len(xs), "valid": len(v),
                    "dt_lock": [x.get("dt_lock") for x in xs],
                    "first": "".join(x.get("first") or "-" for x in xs), "contact": sum(x.get("contact", 0) for x in xs),
                    "pet": [x.get("pet") for x in xs], "clear": [x.get("clear") for x in xs],
                    "waitA": [x.get("waitA") for x in xs], "waitB": [x.get("waitB") for x in xs],
                    "both_stopped": [x.get("both_stopped_s") for x in xs], "cleared": sum(x.get("both_cleared", 0) for x in xs),
                    "tB_out": [x.get("tB_out") for x in xs], "tA_out": [x.get("tA_out") for x in xs],
                    "ttc2d": [x.get("ttc2d") for x in xs], "seeds": [x.get("seed") for x in xs]})
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fit-from", help="EV4 held out: the earlier grid whose seeds 0-1 are the fit split")
    ap.add_argument("--e", type=float, default=0.55, help="the node's e (for the held-out coverage)")
    a = ap.parse_args()
    base = Path(a.out)
    runs = load_runs(base)
    if a.fit_from:
        res = {"ev4_heldout": ev4_heldout(base, Path(a.fit_from), runs, a.e),
               "ev4_heldout_age05": ev4_heldout(base, Path(a.fit_from), runs, a.e, 0.5)}
        for k, v in res.items():
            (base / f"{k}.json").write_text(json.dumps(v, indent=1))
        print(json.dumps(res, indent=1))
    res = {"outcomes": outcomes(runs), "rtc": rtc(base, runs), "ev4": ev4(base, runs), "ev4_age05": ev4(base, runs, 0.5), "first_tracked": first_tracked(base, runs),
           "ev5": ev5(base, runs)}
    for k, v in res.items():
        (base / f"{k}.json").write_text(json.dumps(v, indent=1))
    print(json.dumps(res["ev4"], indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
