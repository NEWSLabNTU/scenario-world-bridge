"""``scensim <command> ...``: one entry for every scensim command (roadmap 081 step 3).
``scensim run ...`` is ``ros2 run scensim run ...``; 082's front end puts ``scensim`` on PATH."""
import importlib
import sys

# command: (module, its entry), in the order `scensim --help` lists them
COMMANDS = {
    "run": ("scensim.run", "cli"),
    "runs": ("scensim.runs", "main"),
    "stack": ("scensim.stack_cli", "main"),
    "testbench": ("scensim.testbench_cli", "main"),
    "assets": ("scensim.assets", "main"),
    "seat": ("scensim.seat", "main"),
    "project": ("scensim.project", "main"),
    "drive": ("scensim.drive", "main"),
    "isaac": ("scensim.isaac", "main"),
    "autoware": ("scensim.autoware", "main"),
}


def usage() -> str:
    return "usage: scensim <command> [args...]\ncommands: " + ", ".join(COMMANDS) + "\n"


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        sys.stdout.write(usage())
        return 0 if argv else 3
    cmd, rest = argv[0], argv[1:]
    if cmd not in COMMANDS:
        sys.stderr.write(f"scensim: no command {cmd}\n{usage()}")
        return 3
    mod, entry = COMMANDS[cmd]
    fn = getattr(importlib.import_module(mod), entry)
    # every entry reads sys.argv when given no argv: give it this command's own
    sys.argv = [f"scensim {cmd}"] + rest
    return fn(rest)


if __name__ == "__main__":
    sys.exit(main())
