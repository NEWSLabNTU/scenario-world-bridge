# 011: the Isaac world server hangs instead of exiting when Kit cannot go on

**Severity**: medium. **Status**: fixed (swb_isaac_world: the world server exits 75 on a Kit
fatal and when it is not up in 20 minutes; its launch respawns it). The fatal's cause is not
found. **Found**: 2026-10-26, roadmap 054 step 8.

## What is wrong

Twice during step 8's runs the Isaac world server stopped answering without exiting, so its
launch's respawn never fired and `scripts/isaac_up.sh` waited out its 30 minutes:

- `play_log/isaac/2026-10-26_12-26-06`: `town01_mud_curve`'s first surface map, a soil zone of
  221 tiles over Town01's south-west bend. The zone was cut and built, and 2.5 minutes into the
  preload's renders Kit logged

  ```
  [Fatal] [omni.rtx] Out of resource descriptors!
  ```

  and the process went on, answering nothing.
- `play_log/isaac/2026-10-26_13-06-34`: a start that stayed at Kit's "Simulation App
  Starting" for 16 minutes (a start takes 1-2 minutes on a warm cache), its memory flat at
  6.1 GB, busy on the CPU.

## What it was not

The first looked like the zone's size: each tile is its own mesh, and smaller zones (32, 60,
90, 99 tiles) had loaded. It is not. The same 221-tile zone, loaded again in a world server
started on its own (`just isaac-world` with that `surfaces.yaml`), was up in 120 s and rendered three more minutes without a fatal; the fatal appears once in every Isaac log
on the host. Both hangs came on a host loaded by other work (load 36-85 on 24 cores, the
GPUs shared), several world servers started and stopped back to back; the kernel's log (Xid)
is not readable here, so whether the GPU reset as in issue 005 is not known.

## Fix

Whatever the cause, a world server that cannot go on should end, so its launch starts a fresh
one (as `swb_carla_world` exits 75 for a dead CARLA connection):

- a message consumer on Kit's log (`omni.log`, `add_message_consumer`) exits 75 on any FATAL
  message, logs flushed and no clean-up (a wedged renderer would hang in it);
- a start-up watchdog started before Kit exits 75 unless Kit's start and the worlds' preload
  are done in 20 minutes (a cold Town01 preload takes about 7; `isaac_up.sh` waits 30).

Checked: inside Kit, a FATAL on `omni.rtx` makes the process exit 75 at once (a probe); a
world server with both guards starts as before (Town01 up in 100 s). Tests:
`test_world_server.py` (the consumer passes only FATAL; the watchdog fires unless cancelled).
