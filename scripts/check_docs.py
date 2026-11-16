#!/usr/bin/env python3
"""The docs check (roadmap 076 B6): the user docs keep their rules.

    python3 scripts/check_docs.py [--root <repo>]

User docs are README.md and docs/**/*.md except the developer pages (docs/development.md,
docs/design/, docs/roadmap/, docs/issues/, docs/proof/, docs/measurements/). In them:

- no `just` command (in a code block or an inline code span): user commands are ros2,
  colcon and play_launch;
- no em dash, en dash or middle dot;
- no host path (/mnt/..., /home/<user>/...);
- code lines at most 88 columns;
- README files name no roadmap phase or milestone;
- every tutorial (docs/tutorials/NN-*.md) has the tutorial format's parts: a **Goal** line,
  You will learn, Before you start, numbered steps each with a **Check**, What happened,
  Exercises, If it does not work, Next.

In every markdown file under docs/ and README.md, each relative link resolves to a file, and
a link into a user doc with an #anchor names one of its headings.

Exit 0 when clean, 1 with one line per problem: `<file>:<line>: <what>`.
"""
import argparse
import re
import sys
from pathlib import Path

DEV = ("docs/development.md",)
DEV_DIRS = ("docs/design/", "docs/roadmap/", "docs/issues/", "docs/proof/",
            "docs/measurements/")
BANNED = {"—": "em dash", "–": "en dash", "·": "middle dot"}
HOST_PATH = re.compile(r"/mnt/|/home/[a-z]")
README_WORDS = re.compile(r"\b(roadmap|phase|milestone)s?\b", re.IGNORECASE)
CODE_WIDTH = 88
LINK = re.compile(r"(?<!!)\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)|<img [^>]*src=\"([^\"]+)\"")
TUTORIAL = re.compile(r"docs/tutorials/\d\d-[^/]+\.md$")
TUTORIAL_PARTS = ("You will learn", "Before you start", "What happened", "Exercises",
                  "If it does not work", "Next")
STEP = re.compile(r"^## \d+\. ")


def is_user_doc(rel: str) -> bool:
    if rel == "README.md":
        return True
    if not rel.startswith("docs/") or rel in DEV:
        return False
    return not any(rel.startswith(d) for d in DEV_DIRS)


def slug(heading: str) -> str:
    """GitHub's anchor for a heading."""
    s = heading.strip().lower()
    s = re.sub(r"[`*]", "", s)
    s = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", s)
    s = re.sub(r"[^\w\- ]", "", s)
    return s.replace(" ", "-")


def anchors(path: Path) -> set:
    out, seen, fence = set(), {}, False
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.lstrip().startswith("```"):
            fence = not fence
            continue
        m = re.match(r"^#{1,6} (.+)$", line)
        if m and not fence:
            base = slug(m.group(1))
            n = seen.get(base, 0)
            seen[base] = n + 1
            out.add(base if n == 0 else f"{base}-{n}")
    return out


def lines_outside_code(text):
    """(line number, line, in_code) for every line."""
    fence = False
    for i, line in enumerate(text.splitlines(), 1):
        if line.lstrip().startswith("```"):
            fence = not fence
            yield i, line, True
            continue
        yield i, line, fence


def check_rules(rel: str, text: str, out: list):
    check_tables(rel, text, out)
    for i, line, code in lines_outside_code(text):
        for ch, name in BANNED.items():
            if ch in line:
                out.append(f"{rel}:{i}: {name}")
        if HOST_PATH.search(line):
            out.append(f"{rel}:{i}: a host path ({HOST_PATH.search(line).group(0)}...)")
        if code:
            if not line.lstrip().startswith("```"):
                if re.match(r"^\s*(\$\s+)?just(\s|$)", line):
                    out.append(f"{rel}:{i}: a `just` command (use ros2, colcon, play_launch)")
                if len(line) > CODE_WIDTH:
                    out.append(f"{rel}:{i}: code line of {len(line)} columns (at most "
                               f"{CODE_WIDTH}; break it with \\)")
        else:
            for span in re.findall(r"`([^`]+)`", line):
                if re.match(r"^just(\s|$)", span):
                    out.append(f"{rel}:{i}: a `just` command (use ros2, colcon, play_launch)")
        if Path(rel).name == "README.md" and not code:
            m = README_WORDS.search(re.sub(r"\([^)]*\)", "", line))
            if m:
                out.append(f"{rel}:{i}: '{m.group(0)}' in a README")


TABLE_RULE = re.compile(r"^\s*\|?\s*:?-{3,}:?\s*(\|\s*:?-{3,}:?\s*)*\|?\s*$")


def check_tables(rel: str, text: str, out: list):
    """A table whose header row is all blank (`| | |`): GitHub renders it as an empty first
    row. Name the columns instead."""
    rows = list(lines_outside_code(text))
    for (i, line, code), (_, nxt, ncode) in zip(rows, rows[1:]):
        if code or ncode or not line.lstrip().startswith("|") or not TABLE_RULE.match(nxt):
            continue
        cells = line.strip().strip("|").split("|")
        if all(not c.strip() for c in cells):
            out.append(f"{rel}:{i}: a table with an empty header row (name its columns)")


def check_tutorial(rel: str, text: str, out: list):
    lines = text.splitlines()
    if not any(line.startswith("**Goal**") for line in lines[:12]):
        out.append(f"{rel}:1: tutorial without a **Goal** line under its title")
    heads = [line[3:].strip() for line in lines if line.startswith("## ")]
    for part in TUTORIAL_PARTS:
        if part not in heads:
            out.append(f"{rel}:1: tutorial without a '## {part}' section")
    steps = [i for i, line in enumerate(lines) if STEP.match(line)]
    if not steps:
        out.append(f"{rel}:1: tutorial without numbered steps ('## 1. ...')")
    for n, start in enumerate(steps):
        end = next((j for j in range(start + 1, len(lines)) if lines[j].startswith("## ")),
                   len(lines))
        if not any("**Check**" in line for line in lines[start:end]):
            out.append(f"{rel}:{start + 1}: step without a **Check**: {lines[start][3:]}")


def check_links(root: Path, path: Path, text: str, out: list, cache: dict):
    rel = path.relative_to(root).as_posix()
    for i, line, code in lines_outside_code(text):
        if code:
            continue
        for m in LINK.finditer(line):
            target = m.group(1) or m.group(2)
            if re.match(r"^[a-z]+:", target) or target.startswith("#") and not target[1:]:
                continue
            file_part, _, anchor = target.partition("#")
            dest = path if not file_part else (path.parent / file_part)
            if not dest.exists():
                out.append(f"{rel}:{i}: broken link {target}")
                continue
            if anchor and dest.suffix == ".md" and dest.is_file():
                drel = dest.resolve().relative_to(root.resolve()).as_posix()
                if not is_user_doc(drel):
                    continue
                if drel not in cache:
                    cache[drel] = anchors(dest)
                if anchor not in cache[drel]:
                    out.append(f"{rel}:{i}: no heading #{anchor} in {drel}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--root", default=str(Path(__file__).resolve().parents[1]))
    args = ap.parse_args(argv)
    root = Path(args.root)
    files = [root / "README.md"] + sorted((root / "docs").rglob("*.md"))
    problems, cache = [], {}
    for path in files:
        rel = path.relative_to(root).as_posix()
        text = path.read_text(encoding="utf-8")
        check_links(root, path, text, problems, cache)
        if is_user_doc(rel):
            check_rules(rel, text, problems)
            if TUTORIAL.search(rel):
                check_tutorial(rel, text, problems)
    for p in problems:
        print(p)
    if problems:
        print(f"docs: {len(problems)} problem(s)", file=sys.stderr)
        return 1
    print(f"docs: ok ({len(files)} files)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
