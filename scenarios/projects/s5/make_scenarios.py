#!/usr/bin/env python3
"""Write the S5 left-turn scenarios, one per gap (roadmap 074 S5 on the real stack; 077 step 4,
item 36).

Each gap becomes scenarios/s5_v<v>_g<gap>.xosc, from scripts/coop/s5_aw.scenario: B (the
ego, Autoware) 70 m north of Town03's junction 861, routed left onto road 62; A (coop_a, an
`agent` entity) oncoming in the inner northbound lane at v, timed to reach the conflict `gap`
s after B's front reaches its stop line. v and gap are in the file's ParameterDeclarations,
where the pilot and the judge read them.

    python3 make_scenarios.py [--v 10] [--gaps 5.5,6,6.5,7,8,9]
"""
import argparse
import re
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from coop_pilots_dir import coop_dir  # noqa: E402
sys.path.insert(0, str(coop_dir()))
import measures as M  # noqa: E402
import s5_aw  # noqa: E402


# Junction 861's traffic lights, all commanded green at Init (issue 073, 133's decision 2026-11-20):
# the regenerated Town03 (carla_map_gen with far_traffic_lights, acb a13efb4) signalizes the junction,
# so B's left turn is a permissive one on green. Every light way the junction's elements refer to: the
# converter groups CARLA's heads across approaches (one pole's heads serve several elements), so a red
# cross street would set the same CARLA light both ways; s5 has no cross traffic, and all green is the
# one consistent state. Way ids of the dataset revision's Town03.
GREEN = ("138070", "138415", "138418", "138430", "138439", "138442", "138448", "138451", "138454",
         "138460", "138661")


def green_init():
    return "".join(f'\n\t\t\t<GlobalAction><InfrastructureAction><TrafficSignalAction>'
                   f'<TrafficSignalStateAction name="{w}" state="green" /></TrafficSignalAction>'
                   f'</InfrastructureAction></GlobalAction>' for w in GREEN)


def name(v, gap):
    return f"s5_v{v:g}_g{gap:g}.xosc"


def write(path, v, gap):
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t) / "s5.xosc"
        s5_aw.scenario(tmp, v, gap, 40.0 + M.HORIZON)
        text = tmp.read_text()
    params = (f'<ParameterDeclarations>\n'
              f'\t\t<ParameterDeclaration name="v" parameterType="double" value="{v}" />\n'
              f'\t\t<ParameterDeclaration name="gap" parameterType="double" value="{gap}" />\n'
              f'\t</ParameterDeclarations>')
    text = text.replace("<ParameterDeclarations />", params, 1)
    assert text.count("<Init><Actions>") == 1
    text = text.replace("<Init><Actions>", "<Init><Actions>" + green_init(), 1)
    text = text.replace("$(env CARLA_MAPS)/Town03", "$(env SCENSIM_MAPS)/Town03", 1)
    text = text.replace("$(find-pkg-share sim_vehicle_spec)/catalogs", "$(env SCENSIM_VEHICLES)", 1)  # the stack's catalog (083)
    text = text.replace("written by scripts/coop/s5_aw.py", "written by make_scenarios.py", 1)
    path.write_text(text)


def cell(xosc):
    """(v, gap) from a scenario's ParameterDeclarations."""
    text = Path(xosc).read_text()
    get = lambda n: float(re.search(rf'name="{n}"[^>]*value="([^"]+)"', text).group(1))
    return get("v"), get("gap")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--v", type=float, default=10.0)
    ap.add_argument("--gaps", default="5.5,6,6.5,7,8,9")
    a = ap.parse_args()
    for g in map(float, a.gaps.split(",")):
        write(HERE / "scenarios" / name(a.v, g), a.v, g)
        print(name(a.v, g))


if __name__ == "__main__":
    main()
