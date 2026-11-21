# Phase 095: Isaac Sim 6.2 and Isaac Lab 3.0

Move every host's Isaac stack from Isaac Sim 6.1.0 and Isaac Lab v3.0.0-EA to Isaac Sim 6.2
and the Isaac Lab 3.0.0 release, and port WheeledLab to it.

**Status**: done 2026-11-21 (user: "Move our hosts to Isaac Sim 6.2 and Isaac Lab 3.0
final. Port WheelLab too"; then "switch to Isaac Sim 6.2 on all agents. Remove old 6.1 and
5.0": each host switches once its checks pass, then removes its 6.1 and 5.x stacks and the
early-access overlay, 118 after its running W10 fit ends).

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
| M1 | Survey: each host's GPU, driver, disk, Isaac installs and the jobs using them; Isaac Sim 6.2's driver requirement; the exact Isaac Lab 3.0.0 tag and what it pins (torch, warp, rsl_rl) | 243 | done 2026-11-21 (section below) |
| M2 | One host first (139): Isaac Sim 6.2 pip environment, Isaac Lab 3.0.0 from its tag (the overlay script moved to it), then aib, the Isaac world server and scensim_batch on it; every Isaac test lane, the K2 environment smoke, and one seats run (SSv2 on Isaac, an authored field) | 139 | done 2026-11-21: every check passed on 6.2 (Isaac lanes 367 + 89 + 8, aib 144 and its Isaac tests 95 of 97 before pytest was added, the Isaac Lab 3.0.0 overlay's 14 members, both K2 smokes, the seats run `isaac_rough_road` on Town01); 139 switched, its 6.1 stack and the early-access overlay removed. Warp brought to 1.17.0 by the pin in M3's follow-up |
| M3 | Code and pins: whatever 6.2 changes in aib and scensim_isaac_world, landed with their pins; `scripts/isaaclab_overlay.sh` and the docs (getting started on Isaac, the simulator backends) name 6.2 and 3.0.0 | 139 | done 2026-11-21: main 940595f3 (aib f7e6175, scensim_isaac_world 6c4d7b3: the overlay script at v3.0.0 with every workspace member, the current texts), then 53f5bfa7 (warp-lang 1.17.0 and torchvision 0.26.0+cu128 pinned in the install script; the spike script resolves its paths before its cd) |
| M4 | The other hosts (118, 241, 243): the same environment, the same checks, then the switch | each host | done 2026-11-21: 243, 241 and 118 switched after their checks (241's T3 Isaac suites passed as the reference; 118's RoboRacer aib run within 1 % of 6.1); every 6.1 stack, the spinning-disk copies, the Isaac Sim 5.x stacks and the early-access overlays removed. Check dangling links after a removal (139 found `~/.cache/warp` linked into its 6.1 root) |
| M5 | WheeledLab ported to Isaac Lab 3.0.0 on Isaac Sim 6.2 (the fork's `isaaclab-3.0` branch, retargeted from the early-access tag); 094's "stock, ported" form runs on it | 241 | done 2026-11-21: NEWSLabNTU/WheeledLab `isaaclab-3.0` trains the drift task on Isaac Sim 6.2 and Isaac Lab 3.0.0 through 094's harness; 094 P1's benchmarks run on 243 |

## M1: the survey (2026-11-21)

Read-only, each host through its own session. Fast disk means NVMe (`lsblk` ROTA 0).

| host | GPU, driver | fast disk free | Isaac Sim installs | Isaac Lab | using it |
|---|---|---|---|---|---|
| 243 | RTX 5090 32 GB, 580.105.08 | `/` 481 G (`/home` 39 G, 99 % used) | `/var/tmp/aeon/isaac-sim` 6.1.0.0, 44 G (`ISAAC_ROOT`); `/mnt/disk1/isaac-sim` 6.1.0.0, 44 G, the spinning-disk original | none | nothing running; K4's exit runs planned |
| 139 | RTX 5090 32 GB, 580.105.08, shared | `/` 87 G | `/mnt/wd/aeon/isaac-sim` 6.1.0.0, 41 G, spinning disk (`ISAAC_ROOT`) | v3.0.0-EA overlay, `/var/tmp/aeon/isaaclab`, 2.0 G | K2's smokes, aib's Isaac test lanes, the Isaac world server for seats runs |
| 118 | 2 × RTX 4090 24 GB, 615.71.09 | `/` 233 G; `/mnt/renegade` 47 G | `/mnt/renegade/aeon/isaac-sim` 6.1.0.0, 49 G (per command) | none | 091 W10's fits, 086 C5, 091 W23's regression |
| 241 | 2 × RTX 5090 32 GB, 610.57.04, shared | `/` 707 G; `/mnt/evo` 508 G | `/mnt/evo/aeon/isaac-sim` 6.1.0.0, 39 G (`ISAAC_ROOT`); `/mnt/toshiba/aeon/isaac-sim` 6.1.0.0, 40 G, spinning disk, unused; `/var/tmp/aeon/wl-stock` Isaac Sim 5.1.0.0 with Isaac Lab v2.3.2, 24 G, unused (fails at renderer start on 610) | v3.0.0-EA overlay, `/var/tmp/aeon/isaaclab`, 2.1 G; WheeledLab fork, branches `bench` and `isaaclab-3.0` | T3's Isaac suites, 091's aib runs, 094 P1's ported form |

**Drivers.** Isaac Sim 6.2 lists Linux 595.58.03 as the driver it was tested on, the same as
6.1; it states no minimum. Omniverse validates the Linux branches R620 (620.70.06), R595
(595.58.03) and R580 (580.95.05). 243 and 139 are on R580. 118 (R615) and 241 (R610) are on
branches NVIDIA does not list, and both run 6.1; per the rules they are reported, not changed.
What 6.2 enforces is its Kit's own `isaacsim/kit/driver-requirements.json` (read from the
installed 6.2.0.0 on 243). On Linux the RTX renderer refuses a driver below 550.90.07, from 570.0
to 570.169 and from 575.0 to 575.64 (by driver category), and recommends 580.95.05. All four
hosts' drivers (580.105.08, 610.57.04, 615.71.09) are outside every refused range, so 6.2
starts on each; 118 and 241 are only off the recommended branch.

**Isaac Lab 3.0.0.** The tag `v3.0.0` is commit `e3f7d9ff1f5d7f8280de3ece9000b7117f66bcf4`
(the early-access tag was `ae37b028`). Its dependencies are pinned once, in the root
`pyproject.toml` of a uv workspace; the per-package `setup.py` files are gone. It pins
`isaacsim[all,extscache]==6.2.0.0` (extra `isaacsim`), `torch==2.11.0`,
`torchvision==0.26.0`, `warp-lang==1.17.0` (early access: 1.16.0), `rsl-rl-lib==5.5.1`
(early access: 5.4.1), `newton[sim]==1.6.1` and Python 3.12. `isaacsim-core` 6.2.0.0 pins the
same torch and Newton. Isaac Sim 6.2 moves Kit from 110.3.0 to 110.4.0 and ships Warp 1.17.0;
its one breaking change, Cortex's removal, touches nothing of ours. The `isaaclab` package is
version 28.0.0 and adds workspace members (`isaaclab-tasks-experimental`,
`isaaclab-visualizers`), so `scripts/isaaclab_overlay.sh`'s package list is checked in M2.

**What names 6.1 or the early-access tag as current** (M3): `scensim_isaac_world`'s
`scripts/install_isaac.sh` and a docstring in `world.py`; this repository's
`scripts/isaaclab_overlay.sh`, `justfile` (`isaac-check`), `scensim/isaac.py` and
`scensim/tools.py`; and the current text of `README.md`, `docs/README.md`,
`docs/getting-started/install.md` and `isaac.md`, `docs/design/simulator-backends.md`,
`batch-executor.md`, `simulator-interface-split.md` and the diagrams `isaac-stack.typ` and
`vehicle-ecu.typ`. aib names neither. The issues and measurements that name 6.1 are dated
records and stay as written.

**Disk.** 243 and 241 hold a 6.2 stack beside 6.1 on NVMe. 139 puts it beside 6.1 on its
spinning disk, where 6.1 already runs, with the Isaac Lab overlay on NVMe. 118 cannot hold both
on `/mnt/renegade`, so its 6.2 goes on `/`, a different disk, after its load time is measured.

**Order.** 241's 6.2 install comes before 094's seeds, so they run on the release. The
spinning-disk 6.1 copies on 243 and 241, and 241's Isaac Sim 5.1 stack, are on the user's
cleanup list.

## Exit

Every host runs Isaac Sim 6.2 with Isaac Lab 3.0.0. `just test` is green there, a seats run
on Isaac passes, the K2 environment smoke passes, and WheeledLab's drift task trains on it.
No document or script names 6.1 or the early-access tag as current.
