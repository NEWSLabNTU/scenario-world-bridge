# 059: A crash dump's path with a space in it is cut at the space

**Severity**: low. The start still ends at once with the right cause, but the message points
at a directory that does not exist, so the dump is not found from it. **Status**: fixed on
branch `059-dump-path`. **Found**: 2026-11-19, swb 241, speed ladder run A (a GPU memory
clash crashed the Isaac world server 5 times).

## What happens

```
scensim run: error: the stack's start is ended: stack node scensim_isaac_world_server exited 139
(SIGSEGV) 5 times before the stack was ready ...; crash dump in /6.1 (...)
```

Kit wrote the dump to a path with a space in it, quoted, and zipped:

```
[Error] [carb.crashreporter-breakpad.plugin] [crash] Wrote dump file '/mnt/evo/aeon/isaac-sim/venv/lib/python3.12/site-packages/isaacsim/kit/data/Kit/Isaac-Sim Python/6.1/35c1fbf8-4364-4623-d8eb24ad-880a69b2.dmp.zip'
```

## Cause

`crashloop._DUMP` was `(/\S+\.dmp)\b`. A path cannot hold a space under it, so the match
began after "Isaac-Sim " (at `/6.1/...`), and the directory shown was `/6.1`.

## Fix

A quoted path is taken whole, spaces included, up to `.dmp` (Kit's `.dmp.zip` too). An
unquoted path keeps the old rule, which ends at whitespace.

## Test

`test_crashloop.py`: Kit's real line gives its whole dump directory. The unquoted
`crash dump written to /tmp/kit/dumps/abc-123.dmp` form still works.
