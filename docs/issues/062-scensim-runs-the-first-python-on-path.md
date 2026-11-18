# 062: The installed `scensim` ran whichever python came first on PATH

**Severity**: medium. On a host where a uv or conda python comes before the system one on
`PATH`, every `scensim` command failed at once with `No module named 'yaml'`. **Status**:
fixed on main. **Found**: 2026-11-19, coop 118, migrating its project to scensim.

## What happens

```
$ which python3
~/.local/share/uv/python/cpython-3.11.14-linux-x86_64-gnu/bin/python3
$ scensim --help
ModuleNotFoundError: No module named 'yaml'
```

`ros2 run scensim scensim --help` worked in the same shell.

## Cause

`<prefix>/bin/scensim` was installed as a data file, which keeps its first line as written:
`#!/usr/bin/env python3`. That line takes the first `python3` on `PATH`. The scripts
under `lib/scensim/`, which `ros2 run` uses, are console scripts. setuptools writes those
with the interpreter that built them (`#!/usr/bin/python3`), so they were not affected.

## Fix

`bin/scensim` is now a POSIX shell script, and it no longer names any python. It runs
`<prefix>/lib/scensim/scensim`, the console script for `scensim.cli:main`, so the command
uses the same interpreter as `ros2 run`. It finds the prefix from the path it was called
by. When that path does not lead to an install, it follows the path's symlinks instead, so
a link to it from elsewhere on `PATH` still works.

The pip front end is not changed. Its own `scensim` is the console script that pip
writes for the environment it was installed into, which is what it needs.

## Test

`test_bin_scensim.py` puts a `python3` that fails first on `PATH`, then runs `bin/scensim`
three ways: as a copy, as `--symlink-install` links it, and through a link from another
directory. Each run reaches the console script. Without the console script, the command
says how to fix it and exits 127.

Checked live on host 243: with uv's CPython 3.11 first on `PATH`, `scensim --help` works
from a `--symlink-install` build.
