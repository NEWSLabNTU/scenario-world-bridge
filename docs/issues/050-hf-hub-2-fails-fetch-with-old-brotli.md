# 050: `scensim assets fetch` fails most files with huggingface_hub 2.x and the distro's brotli

**Severity**: high (blocks 0.1.0: the install docs' rosdep step leads straight into it).
**Status**: fixed on branch `050-hf-brotli-fallback`. **Found**: 2026-11-18 by swb 118, in a
ros:jazzy container; reproduced by swb 241 on Jazzy and on Humble.

## What happens

On a host with huggingface_hub 2.x and the distribution's `python3-brotli`, `scensim assets fetch`
fails for most files of a dataset:

```
problem: jerry73204/isaac-road-physics-stages/materials_surfaces.yaml: process() takes no
keyword arguments (fix: check the dataset's name and revision)
```

The fix it names is wrong: the dataset's name and revision are fine.

Reproduced with scensim's own `HfHub` against `jerry73204/isaac-road-physics-stages`, fetching six
of its files:

| Host | huggingface_hub | brotli | Files fetched |
|---|---|---|---|
| ros:jazzy (Python 3.12) | 2.4.0 | none | 6 of 6 |
| ros:jazzy (Python 3.12) | 2.4.0 | 1.1.0 (noble's python3-brotli) | 2 of 6 |
| newslab-server-241, Humble (Python 3.10) | 2.4.0 | 1.0.9 (jammy's python3-brotli) | 2 of 6 |

118 also found that upgrading brotli to 1.2.1 fixes it.

## Cause

huggingface_hub 2.x downloads through httpx2. httpx2 asks for brotli-compressed responses
whenever the `brotli` module imports. Its decoder then calls
`brotli.Decompressor().process(data, output_buffer_limit=...)`, a keyword that brotli added in
1.2 (`httpx2/_decoders.py`, line 191):

```
TypeError: process() takes no keyword arguments
```

Ubuntu 22.04 ships brotli 1.0.9 and 24.04 ships 1.1.0. Some files still fetch (README.md and
one small YAML in the table above), so the failure looks like a problem with particular files.

Who it hits: scensim's `package.xml` declares the rosdep pip key `python3-huggingface-hub-pip`.
A user who runs `rosdep install`, as the install docs say, gets the latest huggingface_hub
(2.4.0 today). If that host has `python3-brotli`, which many desktop installs pull in, most
fetches fail. A host without huggingface_hub never sees it, because scensim then fetches over
its own urllib path (`HttpHub`), which asks for no compression. That is why this host's
earlier runs never saw it.

## Fix

1. When a huggingface_hub call fails (a download, the refs or a revision), `HfHub` does it
   again over scensim's own HTTPS path (`HttpHub`). It notes the cause once, on stderr, in one
   line: the brotli mismatch when it applies, otherwise huggingface_hub's own error. A failure
   that remains is reported by `HttpHub`, whose messages tell a missing dataset (HTTP 404)
   apart from an unreachable Hub. So huggingface_hub's errors never blame the dataset.
2. `scensim testbench check` (and `project check`) gives a host finding, a warning that does
   not refuse, when a project fetches datasets from the Hub with huggingface_hub 2.x and a
   brotli older than 1.2 installed. Its fix is `pip install 'brotli>=1.2'`.

Pinning huggingface_hub below 2 was rejected: it moves the problem rather than removing it.

## Confirmed

On newslab-server-241 (Humble, Python 3.10), huggingface_hub 2.4.0 and brotli 1.0.9, the whole of
`isaac-road-physics-stages` (79 files, a fresh cache each time):

- **main 3dfa264:** `scensim assets fetch` exits 3 on the first file it downloads:
  `problem: jerry73204/isaac-road-physics-stages/LICENSE: process() takes no keyword arguments
  (fix: check the dataset's name and revision)`.
- **The fix:** it exits 0 and writes `scensim.lock` (79 files, commit 829381ae), with one line on
  stderr:
  `note: huggingface_hub failed on jerry73204/isaac-road-physics-stages/LICENSE (huggingface_hub
  2.4.0 decodes the Hub's brotli responses with brotli 1.2 or newer, and brotli 1.0.9 is
  installed); fetching over plain HTTPS instead (fix: pip install 'brotli>=1.2')`.
  The six files of the table above all fetch, and `scensim testbench check` gives the host
  warning.
