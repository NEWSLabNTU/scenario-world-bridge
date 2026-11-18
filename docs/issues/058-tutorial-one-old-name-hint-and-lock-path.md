# 058: The old-name hint repeats on every command, and Tutorial 1's lock line differs

**Severity**: low. Nothing fails. On a host that ran scensim before the rename, every
command repeats the same hint, and Tutorial 1's step 3 shows a line that the command does
not print. **Status**: fixed on main. **Found**: 2026-11-18, swb 243, Tutorial 1 on host 243
(083 step 5).

## What happens

1. On a host that still has `~/.cache/swb` and `~/.local/state/swb` (from before the project
   was renamed), every `scensim` command opens with:

   ```
   scensim: /home/<user>/.cache/swb has the old name and is not read; move it:
       mv /home/<user>/.cache/swb /home/<user>/.cache/scensim
   ```

   Running `colcon build`, `scensim assets fetch`, `scensim testbench check` and
   `scensim run` prints it once for each command.

2. In Tutorial 1, step 3 (`scensim assets fetch my_first_drive`) shows
   `src/my_first_drive/scensim.lock written`. The command prints the absolute path, for
   example `/home/<user>/t1_ws/src/my_first_drive/scensim.lock written`.

## Cause

1. `config.check_renamed` remembered which directories it had hinted in a set that lives
   for one process. Every command is a new process, so the hint printed each time.
2. A testbench's `scensim.lock` is a link to the source package's copy. `assets.fetch`
   named the link's target with `os.path.realpath`, which is always absolute.

## Fix

1. Once the hint is printed, a marker `.scensim-rename-hinted` is left in the old
   directory, and the hint is not printed again while the marker is there. The marker does
   not go in the new directory: creating that directory would turn the printed
   `mv old new` into a move of `old` into `new`. If the old directory cannot be written,
   the hint falls back to once per process, as before. The hint now says that it is printed
   once.
2. A command run inside the workspace now gets the lock's path relative to the working
   directory, which matches the tutorial. A command run elsewhere still gets the absolute
   path.

## Test

`test_config.py`:
- a second process (a cleared hint set) prints no hint, the marker is in the old
  directory, and the new directory has not been created;
- an old directory that cannot be written is hinted once per process.

`test_assets.py`: a linked lock is named `src/tb/scensim.lock` from the workspace, and by
its absolute path from elsewhere.
