"""What a check found, printed one per line, and the exit code it gives.

Exit codes (docs/design/projects-and-pilots.md, "Commands"): 0 nothing found, 1 a problem,
3 refused (a missing asset, bad input). A finding is ``refused`` when the stack must not start
on it whatever else holds (an asset not fetched, a project file that cannot be read).
"""
import argparse
from dataclasses import dataclass

OK, PROBLEM, INFRA, REFUSED = 0, 1, 2, 3


@dataclass(frozen=True)
class Finding:
    file: str
    what: str
    fix: str = ""
    refused: bool = False

    def __str__(self):
        fix = f" (fix: {self.fix})" if self.fix else ""
        return f"problem: {self.file}: {self.what}{fix}"


def exit_code(findings) -> int:
    if not findings:
        return OK
    return REFUSED if any(f.refused for f in findings) else PROBLEM


def ros_args(argv):
    """``name:=value`` (as a launch file takes it) to ``--name value``; the rest unchanged."""
    out = []
    for a in argv:
        if ":=" in a and not a.startswith("-"):
            k, v = a.split(":=", 1)
            out += ["--" + k.replace("_", "-"), v]
        else:
            out.append(a)
    return out


def add_project_option(*parsers):
    """``--project`` before or after the subcommand (``project:=`` comes after it)."""
    for i, p in enumerate(parsers):
        p.add_argument("--project", help="the project directory or pkg://<package>",
                       **({} if i == 0 else {"default": argparse.SUPPRESS}))
