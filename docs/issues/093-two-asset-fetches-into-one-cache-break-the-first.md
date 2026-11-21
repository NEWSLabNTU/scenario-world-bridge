# 093: Two asset fetches into one cache break the first

**Severity**: medium. A long download dies part-way whenever another fetch of the same dataset
commit runs beside it, from any project on the host. **Status**: open; diagnosis only, the fix
is to be assigned. **Found**: 2026-11-21, swb 243, roadmap 091 W17's exit run.

## What happens

One `scensim assets fetch` was downloading `carla-0.9.16-isaac-usd`'s vehicles (commit
38347e26, 9706 files, about 1.7 MB/s). A second fetch, of another project that names the same
commit, ran and finished while it did; that one used a host copy of the dataset
(`datasets:` in the host settings) and downloaded nothing. The first then died:

```
  File ".../scensim/assets.py", line 539, in add
    os.replace(part, dest)
PermissionError: [Errno 13] Permission denied: '~/.cache/scensim/tmp/tmp4xtua9c3/file' ->
  '~/.cache/scensim/datasets/jerry73204/carla-0.9.16-isaac-usd/38347e26.../vehicles/walkers/
  .../T_Iris001_01_AO.png'
```

## Cause

The cache keeps each commit's directory tree read-only between fetches. `_Store` (assets.py)
makes it writable on entry and read-only again on exit, for the whole tree, unconditionally:

- `__enter__`: `_set_tree_mode(self.dir, True)`, every directory 0755;
- `__exit__`: `_set_tree_mode(self.dir, False)`, every directory 0555.

Nothing counts or locks the fetches using a commit's directory. So the second fetch's exit
sealed the tree while the first was still adding files, and the first's next `os.replace`
into a 0555 directory failed. A fetch that only locks a host copy (`lock_only`, issue 076)
still enters and leaves `_Store` for the commit, so it seals too, though it writes nothing.
`add` already allows for a second fetch (it skips a file another one completed meanwhile);
the directory modes do not.

## Expected

The second fetch never breaks the first. Either it waits on a lock held by the first (a
`flock` on a file per commit directory, or per cache), and seals only when it is the last one
out; or it refuses at once, naming the holder (its pid and project). A fetch that writes
nothing should not change the tree's modes at all.

## How it was confirmed

The traceback above, in the first fetch's log; the second fetch's output at the same minute
(`carla-0.9.16-isaac-usd: the host's copy /mnt/disk1/carla-usd, 2131 file(s)` then `locked as
the Hub has it, 38347e26ab35: 9706 file(s), 0 downloaded`); and `_Store.__exit__` in the code.
Not reproduced deliberately.
