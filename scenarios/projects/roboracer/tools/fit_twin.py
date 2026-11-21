"""Roadmap 091 W10: fit the twin to the real car's logs. It reads only the logs (`--logs`),
never anything a generator sealed.

    python3 fit_twin.py --logs REAL_LOGS --spec SPEC.yaml --out FITTED.yaml
        [--previous FITTED.yaml --skip component]

The stages are protocol/fit.py's, in order:
1. the drive unit, by replay: the current limit from M1c, the latency on a grid, the speed
   loop's gains;
2. the mass, from the steady holds against the coast-down;
3. the servo's offset, from M5's left and right circles;
4. the tyres per axle, from each axle's slip angles and forces through the reference's sideslip;
5. the noise.

FITTED.yaml has the same sections as a perturbation (`vehicle`, `component`, `noise`), so
`twin_runs.py --fitted FITTED.yaml` runs the twin as fitted. `--skip component --previous F`
carries the slowest stage over from an earlier fit.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path[:0] = [str(ROOT / "protocol"), str(HERE), str(ROOT / "component" / "roboracer_vesc")]
import fit  # noqa: E402
import protocol as protomod  # noqa: E402
from compare_runs import load_runs  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--logs", required=True)
    ap.add_argument("--spec", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--protocol", default=str(ROOT / "protocol" / "protocol.yaml"))
    ap.add_argument("--previous", default="")
    ap.add_argument("--skip", default="", help="stages to carry over from --previous")
    args = ap.parse_args(argv)
    from roboracer_vesc.component import DEFAULTS, RoboRacerVesc

    doc = protomod.load(args.protocol)
    car = yaml.safe_load(Path(args.spec).read_text())
    runs = load_runs(args.logs)
    prev = yaml.safe_load(Path(args.previous).read_text()) if args.previous else {}
    skip = {s for s in args.skip.split(",") if s}
    report, t0 = {}, time.monotonic()

    # 1. the component
    if "component" in skip and prev.get("component"):
        comp = {k: prev["component"][k] for k in fit.FIT_COMPONENT}
    else:
        comp, rms, notes = fit.fit_component(runs, RoboRacerVesc, car, protomod.MOTORS, doc)
        report["component_rms_A"] = rms
        report["notes"] = notes
    report["component_s"] = round(time.monotonic() - t0, 1)

    # 2. the mass: the drive force per amp, ERPM per m/s over ERPM per motor rad/s, times kt
    p = {**DEFAULTS, **comp}
    force_per_amp = p["kt"] * p["erpm_per_mps"] / p["erpm_per_motor_rad_s"]
    wheel_mass = 2.0 * float(car["wheels"]["mass"])  # 4 x 0.5 m_w r^2 / r^2
    mass, c = fit.fit_mass(runs, doc, force_per_amp, wheel_mass)
    report["coast"] = {"c0": c[0], "c1": c[1], "c2": c[2]}

    # 3. the servo's offset
    conv = doc["conversion"]
    d_offset = fit.fit_servo_offset(runs, doc, car["geometry"]["wheel_base"],
                                    conv["steering_angle_to_servo_gain"])
    servo_offset = conv["steering_angle_to_servo_offset"] + d_offset
    offset_angle = -d_offset / conv["steering_angle_to_servo_gain"]

    # 4. the tyres, each axle from its own slip angles and forces (the reference's sideslip)
    pts = fit.axle_points(runs, doc, car, offset_angle)
    front, rear = fit.fit_axle(pts["front"]), fit.fit_axle(pts["rear"])
    tyres = front + rear
    report["tyre_points"] = {k: len(v) for k, v in pts.items()}
    not_identified = []
    for axle, f in (("front", front), ("rear", rear)):
        reached, top = fit.peak_reached(pts[axle], f)
        if not reached:
            not_identified.append(f"vehicle.axle_tires.{axle}.tire_friction")
            report.setdefault("notes", []).append(
                f"{axle} tyres: the logs never come near their limit (at most {top:.3f} of the "
                f"load); their friction is not identified, only at least that")

    # 5. the noise
    noise = fit.fit_noise(runs, doc)

    fitted = {
        "vehicle": {"mass": round(mass, 4), "axle_tires": fit.axle_tires_of(tyres)},
        "component": {**{k: round(float(v), 6) for k, v in comp.items()},
                      "servo_offset": round(servo_offset, 5)},
        "noise": {k: (round(v, 5) if v is not None else None) for k, v in noise.items()},
        "not_identified": not_identified,
        "tyres_fit": [round(x, 5) for x in tyres],
        "offset_angle": round(offset_angle, 6),
        "report": report,
        "logs": str(Path(args.logs).resolve()),
    }
    text = yaml.safe_dump(json.loads(json.dumps(fitted, default=float)), sort_keys=False)
    Path(args.out).write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
