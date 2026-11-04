#!/usr/bin/env python3
"""The NTU twin's scenarios on both simulators from one source (roadmaps 059 and 072, parity P3).

Each `scenarios/ntu_twin/ntu_twin_<case>.xosc` (Isaac, roadmap 059) is the source of
`ntu_carla_<case>.xosc` (CARLA, roadmap 072). They differ only in:

- the map dir (`LogicFile`): `$(env SWB_NTU_TWIN)/map` on Isaac, `$(env SWB_NTU_TWIN)/carla/NTU`
  on CARLA (the bridge takes CARLA's town from the directory name);
- the header's description and the leading comment (which roadmap, which world);
- the Story name, which is the file's stem.

Edit the ntu_twin_ file and run this; never edit an ntu_carla_ file by hand.

    python3 scripts/ntu_twin/scenario_pair.py           # write every ntu_carla_<case>.xosc
    python3 scripts/ntu_twin/scenario_pair.py --check   # exit 1 if any is out of date
"""
import argparse
import difflib
import pathlib
import re
import sys

DIR = pathlib.Path(__file__).resolve().parents[2] / "scenarios" / "ntu_twin"
ISAAC_MAP = '<LogicFile filepath="$(env SWB_NTU_TWIN)/map" />'
CARLA_MAP = '<LogicFile filepath="$(env SWB_NTU_TWIN)/carla/NTU" />'
# (Isaac's words, CARLA's), applied in order to the description and the comment
WORDS = [
    ("roadmap 059 N8: ", "roadmap 072: "),
    ("Roadmap 059 N8: ", "Roadmap 072 (CARLA): "),
    ("the NTU campus twin (MLS scan)", "the NTU campus in CARLA 0.9.16 (map built from the MLS scan)"),
    ("the NTU campus twin", "the NTU campus in CARLA 0.9.16"),
    ("the NTU twin", "the NTU campus in CARLA"),
]


def carla_of(text: str, case: str) -> str:
    if text.count(ISAAC_MAP) != 1:
        sys.exit(f"ntu_twin_{case}.xosc: expected one {ISAAC_MAP}")
    out = text.replace(ISAAC_MAP, CARLA_MAP)
    out = out.replace(f'<Story name="ntu_twin_{case}">', f'<Story name="ntu_carla_{case}">')

    def words(m):
        s = m.group(0)
        for a, b in WORDS:
            s = s.replace(a, b)
        return s

    out = re.sub(r'description="[^"]*"', words, out, count=1)
    out = re.sub(r"<!--.*?-->", words, out, count=1, flags=re.S)
    note = (f"<!-- Generated from ntu_twin_{case}.xosc by scripts/ntu_twin/scenario_pair.py:"
            " edit that file, not this one. -->\n")
    head, _, rest = out.partition("\n")
    return f"{head}\n{note}{rest}"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--check", action="store_true", help="only compare, exit 1 on a difference")
    a = ap.parse_args()
    bad = 0
    for src in sorted(DIR.glob("ntu_twin_*.xosc")):
        case = src.stem.removeprefix("ntu_twin_")
        dst = DIR / f"ntu_carla_{case}.xosc"
        want = carla_of(src.read_text(), case)
        have = dst.read_text() if dst.exists() else ""
        if want == have:
            continue
        if a.check:
            bad += 1
            sys.stdout.writelines(difflib.unified_diff(
                have.splitlines(True), want.splitlines(True), str(dst), f"from {src.name}"))
        else:
            dst.write_text(want)
            print(f"wrote {dst.relative_to(DIR.parents[1])}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
