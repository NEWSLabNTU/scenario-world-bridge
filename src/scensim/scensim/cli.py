"""``scensim <command> ...``: one entry for every scensim command (roadmap 081 step 3).
``scensim run ...`` is ``scensim run ...``; 082's front end puts ``scensim`` on PATH."""
import importlib
import sys

# command: (module, its entry, what it does), in the order `scensim --help` lists them
COMMANDS = {
    "run": ("scensim.run", "cli", "run a testbench, a suite or a scenario"),
    "runs": ("scensim.runs", "main", "list, show and explain past runs"),
    "stack": ("scensim.stack_cli", "main", "start, stop or list a testbench's stack"),
    "testbench": ("scensim.testbench_cli", "main", "new, list and check testbenches"),
    "assets": ("scensim.assets", "main", "fetch and check maps and datasets"),
    "seat": ("scensim.seat", "main", "this host's seats: ports and domains"),
    "project": ("scensim.project", "main", "new and check a testbench directory"),
    "drive": ("scensim.drive", "main", "drive a car by hand"),
    "isaac": ("scensim.isaac", "main", "Isaac Sim on this host"),
    "autoware": ("scensim.autoware", "main", "Autoware's carried fixes and runtime"),
}


def usage() -> str:
    return ("usage: scensim <command> [args...]\n\ncommands:\n"
            + "".join(f"  {c:10} {d}\n" for c, (_, _, d) in COMMANDS.items())
            + "\n`scensim <command> --help` for each\n")


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        sys.stdout.write(usage())
        return 0
    cmd, rest = argv[0], argv[1:]
    if cmd not in COMMANDS:
        sys.stderr.write(f"scensim: no command {cmd}\n{usage()}")
        return 3
    mod, entry, _ = COMMANDS[cmd]
    fn = getattr(importlib.import_module(mod), entry)
    # every entry reads sys.argv when given no argv: give it this command's own
    sys.argv = [f"scensim {cmd}"] + rest
    return fn(rest)


if __name__ == "__main__":
    sys.exit(main())
