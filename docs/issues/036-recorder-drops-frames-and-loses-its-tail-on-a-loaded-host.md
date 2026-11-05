# 036: The Isaac recorder drops most frames on a loaded host and its last rows hit a closed file

**Severity**: medium for recordings (a video with gaps, unusable), none for a scenario's
verdict. **Status**: fixed in swb_isaac_world 9e351a7. **Found**: 2026-11-05 on
newslab-server-241, roadmap 066's road-conditions tour (`spikes/rock_world.py`,
`ROCK_COURSE=tour ROCK_RECORD=1`), host load 70-90 on 32 cores.

## What happens

The recorder (`swb_isaac_world/recorder.py`) captured 1024 frames of a 600 m drive and kept
133: `RECORDED ... frames 1024 dropped 891`. At its close a writer thread traceback followed:

```
File ".../recorder.py", line 316, in _write
    self.state.write(json.dumps(row) + "\n")
ValueError: I/O operation on closed file.
```

## Why

- `capture` hands each frame to a writer thread through a queue of 120 frames and drops a
  frame when the queue is full (`put_nowait`), so that a slow disk never holds up the world
  server's tick. On a host this loaded the writer, which JPEG-encodes five 960x540 pictures a
  frame, fell behind for good, and the queue stayed full.
- `close` waits 30 s for the writer, then closes `state.jsonl` whether it finished or not; a
  writer still draining the queue then writes to a closed file and dies, losing the frames it
  still held.

## Fix

- The recorder takes `block` (default False, as before): True waits for room in the queue
  instead of dropping, for a recording that must be whole (a demo, a proof video), at the cost
  of the simulation's pace. `rock_world.py` records with it.
- The writer closes `state.jsonl` itself after the last frame. `close` waits for it without a
  limit when blocking, 30 s otherwise; if the writer is still busy then, it logs that and
  leaves the file to the writer, so no row is written to a closed file.
