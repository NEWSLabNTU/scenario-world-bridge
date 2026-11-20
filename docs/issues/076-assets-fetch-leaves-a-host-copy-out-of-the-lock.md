# 076: `scensim assets fetch` leaves a dataset out of scensim.lock when the host has its own copy

**Severity**: medium (a committed lock that misses a dataset its testbench takes). **Status**:
fixed 2026-11-20: the lock names the dataset as the Hub has it, read from its manifest, without
downloading the files.
**Found**: 2026-11-20, swb 118 and 241: the committed locks of `isaac_soil` and `isaac_verge`
named `carla-town-autoware-maps` but not `carla-0.9.16-isaac-usd`, which both take
(`[Town01, vehicles]`).

## What happens

A host's settings can name a dataset the host keeps a copy of itself (`datasets: {<name>:
<directory>}`, roadmap 077 G8). `scensim assets fetch` then links the testbench's assets from
that copy instead of fetching them, and keeps the copy's file list in the testbench's
`state/assets.local.json`. It also left the dataset out of `scensim.lock`, unless an older lock
already had it. So a testbench whose lock was first written on such a host (118 keeps
`carla-0.9.16-isaac-usd` on its own disk) was committed with a lock that does not name that
dataset. On any other host, scensim refused to run the testbench until `scensim assets fetch`
was run again, and that run wrote a lock different from the committed one. 241, which has no
host copy, found it during the `isaac_soil` acceptance (NAS logs "2026-11-20 086 C4
isaac_soil on Isaac Town01 (241)"), and main 7e15f6f added the missing entries by hand.

## Cause

`assets.fetch` in `src/scensim/scensim/assets.py` skipped the Hub for a dataset in the host's
`datasets:`, and its comment said so on purpose: "scensim.lock keeps only what came from the
Hub". The reference page said the same (`docs/reference/host-settings.md`). The rule was meant
for a private dataset, which is not on the Hub at all, but it also covered a public dataset
the host happens to keep a copy of.

## Fix

For a dataset the host keeps a copy of, `assets fetch` still links from the copy, and now also
writes the lock entry the Hub would give (`fetch_dataset(..., lock_only=True)`):

- an entry that already matches the testbench (repo, take, revision) is kept, with no Hub
  call, so a fetch that changes nothing works offline as before;
- otherwise the revision is resolved and the manifest is downloaded into the cache. Each taken
  file's hash comes from the manifest. Only files the manifest does not cover (the card and
  notices) are downloaded, so a host with a copy does not download the data files;
- a dataset the Hub does not have (a private one) stays out of the lock, as before. If the Hub
  is out of reach, the old entry is kept, or the fetch says the dataset is not locked and to
  fetch again.

`assets check` already skipped the lock's cache files for a dataset the host keeps, so a lock
entry whose files are not in the cache is fine on that host. A host without the copy fetches
the locked commit. Tests: `test_assets.py::test_a_hub_dataset_the_host_keeps_is_locked_as_the_hub_has_it`
(only the manifest downloaded; the second fetch makes no Hub call; a host without the copy
fetches the same commit and the lock does not change) and
`::test_a_host_copy_keeps_its_lock_entry_when_the_hub_is_out_of_reach`.
