# Phase 095: Isaac Sim 6.2 and Isaac Lab 3.0

Move every host's Isaac stack from Isaac Sim 6.1.0 and Isaac Lab v3.0.0-EA to Isaac Sim 6.2
and the Isaac Lab 3.0.0 release, and port WheeledLab to it.

**Status**: planned 2026-11-21 (user: "Move our hosts to Isaac Sim 6.2 and Isaac Lab 3.0
final. Port WheelLab too").

**Why**: Isaac Lab v3.0.0 (released 2026-10-19) pins isaacsim 6.2.0.0, so 092's K2 pinned
the early-access tag that matches 6.1. One stack for Isaac Sim, Isaac Lab, aib, the Isaac
world server and the batch executor removes that split, and 094 compares stock and ours on
the release users will install.

## Rules

- **Side by side, then switch.** Each host gets the 6.2 stack as a new environment beside
  the 6.1 one. The host's `ISAAC_ROOT` (a host setting in `.env`) moves only after the
  checks below pass on that host. The 6.1 environment is removed only with the user's OK.
- **No running job is disturbed.** A host's switch waits for its Isaac jobs to end (W10
  fits, K4 exit runs, 094 seeds), announced to the manager first.
- **Disk.** A host keeps its fast disk above 60 GB free. Where it cannot hold both stacks,
  the new one goes on another disk only after the load-time cost is measured, or the old one
  is removed first with the user's OK.
- **Drivers are not changed.** A host whose driver cannot run 6.2 is reported, not modified.

## Steps

| Step | What | Owner | Status |
|---|---|---|---|
| M1 | Survey: each host's GPU, driver, disk, Isaac installs and the jobs using them; Isaac Sim 6.2's driver requirement; the exact Isaac Lab 3.0.0 tag and what it pins (torch, warp, rsl_rl) | 243 | open |
| M2 | One host first (139): Isaac Sim 6.2 pip environment, Isaac Lab 3.0.0 from its tag (the overlay script moved to it), then aib, the Isaac world server and scensim_batch on it; every Isaac test lane, the K2 environment smoke, and one seats run (SSv2 on Isaac, an authored field) | 139 | open |
| M3 | Code and pins: whatever 6.2 changes in aib and scensim_isaac_world, landed with their pins; `scripts/isaaclab_overlay.sh` and the docs (getting started on Isaac, the simulator backends) name 6.2 and 3.0.0 | 139 | open |
| M4 | The other hosts (118, 241, 243): the same environment, the same checks, then the switch | each host | open |
| M5 | WheeledLab ported to Isaac Lab 3.0.0 on Isaac Sim 6.2 (the fork's `isaaclab-3.0` branch, retargeted from the early-access tag); 094's "stock, ported" form runs on it | 241 | open |

## Exit

Every host runs Isaac Sim 6.2 with Isaac Lab 3.0.0. `just test` is green there, a seats run
on Isaac passes, the K2 environment smoke passes, and WheeledLab's drift task trains on it.
No document or script names 6.1 or the early-access tag as current.
