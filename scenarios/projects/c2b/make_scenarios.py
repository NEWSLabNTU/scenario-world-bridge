#!/usr/bin/env python3
"""Write the C2b cut-in scenarios, one per cell (roadmap 074 C2b; 077 step 4, item 32).

Each cell (d0, dv) of scripts/coop/c2b.py becomes scenarios/c2b_d<d0>_v<dv>.xosc: coop_a and
coop_b on Town03's east straight, each an `agent` entity, the start moved back so that the
warm-up and the horizon stay on the straight (c2b.on_straight, from CARLA's map). The cell is
in the file's ParameterDeclarations (d0, dv), where the pilots and the judge read it; `pair`
is the suite's grid parameter (the rule pair).

    python3 make_scenarios.py --carla-port 2010     # CARLA with Town03 loaded
"""
import argparse
import re
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2] / "scripts" / "coop"))
import c2b  # noqa: E402


def name(d0, dv):
    return f"c2b_d{d0:g}_v{dv:g}.xosc"


def write(path, d0, dv, shift):
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t) / "c2b.xosc"
        c2b.scenario(tmp, d0, dv, shift)
        text = tmp.read_text()
    params = (f'<ParameterDeclarations>\n'
              f'\t\t<ParameterDeclaration name="d0" parameterType="double" value="{d0}" />\n'
              f'\t\t<ParameterDeclaration name="dv" parameterType="double" value="{dv}" />\n'
              f'\t\t<ParameterDeclaration name="pair" parameterType="string" value="eq" />\n'
              f'\t\t<ParameterDeclaration name="start_shift" parameterType="double" value="{shift:.1f}" />\n'
              f'\t</ParameterDeclarations>')
    text = text.replace("<ParameterDeclarations />", params, 1)
    text = text.replace("$(env CARLA_MAPS)/Town03", "$(env SWB_MAPS)/Town03", 1)
    text = text.replace("written by scripts/coop/c2b.py", "written by make_scenarios.py", 1)
    path.write_text(text)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--carla-port", type=int, default=2000)
    a = ap.parse_args()
    for d0, dv in c2b.CELLS:
        shift, short = c2b.on_straight(a.carla_port, d0, dv)
        write(HERE / "scenarios" / name(d0, dv), d0, dv, shift)
        print(f"{name(d0, dv)}: start moved back {shift:.1f} m"
              + (f", still {short:.0f} m off the straight" if short > 0 else ""))


def cell(xosc):
    """(d0, dv) from a scenario's ParameterDeclarations."""
    text = Path(xosc).read_text()
    get = lambda n: float(re.search(rf'name="{n}"[^>]*value="([^"]+)"', text).group(1))
    return get("d0"), get("dv")


if __name__ == "__main__":
    main()
