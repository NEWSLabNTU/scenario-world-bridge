# Phase 075: User Docs the ROS Way, and the Assets Published

A user builds their own complete stack from the docs alone: their scenario, a simulator
(CARLA or Isaac Sim), their vehicle, their autopilot (Autoware or their own agent), and more
cars. Every command is a ROS one (rosdep, colcon, `ros2 run`, `play_launch launch`), not a
`just` recipe. The maps and models we built are published on Hugging Face, with a copy of each
on the NAS.

**Status**: started, 2026-11-14: audit done, the doc set and the launch fixes decided.

**Source**: user, 2026-11-14: "Start the user experience campaign on the swb project. Find
leftover UX jobs first. Review the user procedure and docs like readme. I would tell ways to
build their own complete stack from scenario, simulation to vehicle configuration and
autopilot connection, and multi car. I would avoid using just recipes because it's designed
for swb devs and the recipes change over time. I would provide ROS conventional commands, such
as colcon, ros2. Recommend users to use play_launch as the replacement of ros2 launch. Tell
ways to configure the ROS_DOMAIN_ID right. Be aware of doc readability. Break long commands.
Plus, find the assets we already have. I would upload them to huggingface." Then: "and plus
golfcart models"; "Ensure the nas has asset copies. I would place persistent map and vehicle
models to autoveh/dataset/..., and experiment logs and outcome, videos to autoveh/logs/...".

**Relation**: takes 070 step 8 (D, docs) and the doc half of 070 step 5; 070 keeps the code
(per-version CARLA launcher, `simple_pilot`, `starter/` scenarios, releases). Builds on 073:
the stack launch (`carla.launch.xml`, `isaac.launch.xml`, `stack.launch.py`) becomes the
user's main path.

## Audit (2026-11-14)

Every user doc read against the five goals and checked against the launch files at HEAD and
the submodule pins (commands checked, none run):

- **`just` in user docs: 47 uses in 9 files**, the worst in the README's quick start and its
  "Everyday commands". All but five have a ROS-native replacement; the five (demo tour,
  two-AV demo, CARLA start under systemd, the TensorRT runtime, vehicle check) are developer
  tools or need a documented manual equivalent.
- **Two run models, never reconciled.** Most docs teach four terminals in two domains; only
  tutorial 7 teaches the one-launch stack. Autoware has three entry points with different
  defaults (`ego_av.launch.xml`, acb's `carla_simulator.launch.xml`, the stack's ego).
- **`ros2 launch` leads and play_launch follows** in most docs; the Isaac guide and tutorials
  5 and 6 never mention play_launch. The required version differs (0.18.3 in tutorial 1, 0.30.0
  elsewhere; the stack needs 0.30.0, and its ego, agent and scenario are nested play_launches,
  so play_launch must be installed even under `ros2 launch`).
- **Domains are stated right but not explained.** Scenario 9, ego 1, background AVs 2 and up,
  at most 57 (Cyclone's `DomainGain 1000`: domain 58 needs ports past 65535). No doc says why
  the domains are split, how the stack maps its groups to them, or how `ros2 topic` reaches
  each. Tutorial 7 line 64 lands in domain 0 when `SCENARIO_ROS_DOMAIN_ID` is unset. "Any
  domain, any host" is true on CARLA only (Isaac's Autoware shares the world server's host).
- **Commands that fail as written**: bare `swb_world_check` and `swb-agent` are not on PATH
  after `source install/setup.bash` (`ros2 run swb_world_check swb_world_check`, `ros2 run
  swb_agent swb-agent`).
- **Stale text**: `design/user-workflow.md`'s "A session" (no world server, no vehicle agent,
  `acb_pilot` gone, bridge params that no longer exist); `map_alias` and the Traffic Manager
  documented in the bridge's config instead of `swb_carla_world.yaml`; background AVs said
  to run lighter perception (they do not); the Autoware overlay's branch and package count.
- **Coverage of "build your own stack"**: scenario authoring and judging good; simulator
  bring-up good but only in the four-terminal form; **vehicle configuration thin** (Isaac only,
  no CARLA "your car", no spec-hash refusal explained); autopilot good per piece but never
  shown with the stack; multi-car CARLA only and never with the stack.
- **Launch gaps the docs trip on**: `carla.launch.xml` cannot pass the world server's config
  file; `swb_launch/package.xml` lacks exec_depends on `swb_carla_world`, `swb_isaac_world`,
  `sim_vehicle_spec`, `aib_launch`, `swb_examples`; `stack.launch.py` does not refuse a domain
  above 57; `demo.launch.xml` (one domain for everything) and the `carla_scenario.launch.xml`
  alias (017's "kept for one phase") are still installed; `carla_simulation.launch.xml`
  describes 3 of its 16 arguments.

## Doc set, proposed

1. **README**: what it is, requirements, a ROS-native quick start (build, one
   `play_launch launch swb_launch carla.launch.xml ...`, the verdict), links. No `just`; one
   line pointing developers to `docs/development.md`.
2. **Getting started, CARLA and Isaac** (kept, the best user docs): stage 1 installs
   play_launch and writes an environment file; stage 3 runs the stack; the four-terminal form
   moves to an appendix.
3. **New: "Build your own stack"**, eleven sections: what you build (slots and domains); the
   environment file; build; the simulator; the scenario; the stack with Autoware; the
   vehicle; your autopilot; more cars; judging; two stacks on one host; troubleshooting.
   Replaces tutorial 7 and absorbs tutorial 6's "Another car" and "Not Autoware at all".
4. **New: "Domains and DDS"**, one page: why the split, the stack's mapping, the 57 limit,
   Cyclone required, looking into each domain, two stacks on one host.
5. **Tutorials, renumbered**: scenario; run and judge; other road users and more AVs;
   configure; demo and recording. Tutorial 1 retires (it repeats the CARLA guide).
6. **Reference**: the user guide becomes the launch and environment reference plus
   troubleshooting; the agent guide gains "with the stack"; scenario authoring fixed.
   `design/user-workflow.md` leaves the user index.

Style for all of them: one argument per line in commands (backslash continuation), at most
88 columns in code blocks, `play_launch launch` first with "`ros2 launch` takes the same
arguments" once per page, no em or en dashes, absolute dates, no host paths or user names.

## Assets

Persistent maps and models on the NAS under `autoveh/dataset/<date> <name>/` (README with
source, licence, how made and commit, plus SHA256SUMS); logs, outcomes and videos under
`autoveh/logs/2026 SSv2 on CARLA/`. Then a Hugging Face release, once the user names the
account and settles the licences.

| Asset | NAS (`autoveh/dataset/`) | Licence / source | Open question |
|---|---|---|---|
| CARLA 0.9.16 towns and actors in USD (5 towns, 41 vehicles, 97 props, 52 walkers) | `2026-10-23_carla-0.9.16-isaac-usd` (24 GB) | CARLA content, CC-BY 4.0 | -- |
| CARLA Town07_Opt in USD | `2026-10-18 CARLA Town07_Opt Isaac USD` | CC-BY 4.0 | -- |
| Mount Haruna and Tsukuba courses (Isaac world, Autoware map, scenario, raw open data) | `2026-10-30 Haruna and Tsukuba courses` | OSM (ODbL), GSI tiles (attribution), CARLA props | world.yaml's absolute path; props by a link to the CARLA set |
| NTU campus twin (Isaac world, Autoware map) | `2026-10-30 NTU twin map` (269 GB) | built from the vendor's MLS scan | published unlisted (decision 4) |
| NTU campus as a CARLA map | not yet (072 planned) | vendor scan | asked of the other hosts |
| Shalun (CARLA and Isaac) | `2026-10-30 SVL Shalun map` | from SVL's assets | published unlisted (decision 4) |
| CARLA towns' Autoware maps with swb's repairs (Town01, 02, 03, 05, 10; signal tables, Isaac world.yaml) | `2026-11-14 CARLA towns Autoware maps (swb)` (530 MB) | TUM's pack, LGPL-3.0; CARLA | isaac/world.yaml and carla/traffic_lights.yaml name absolute paths |
| Town05 with the ring at 110 km/h (063) | `2026-11-14 Town05 ring 110kmh (063)` (160 MB) | as above | -- |
| Road-physics test stages (13 surfaces, proving ground; towns' physics layers in the USD set) | `2026-10-29 Isaac road-physics test stages` | ours | 066/069 stages asked of the other hosts |
| Golf cart models (Isaac body and wheels, lidars, rig, spec, catalogue, RViz mesh) | `2026-11-08 Golf cart vehicle models` | 2026-golf-cart, Apache-2.0 (decision 4) | the reference photos stay out (licence plates) |

Not ours to publish: the NAS's terramechanics test data (third-party reports).

## Decisions (user, 2026-11-14)

1. **The doc set as proposed**: the stack is the main path; "Build your own stack" and
   "Domains and DDS" are new; the tutorials are renumbered and tutorial 1 retires; the
   four-terminal form stays as an appendix; no `just` in user docs.
2. **The launch gaps are fixed in this phase** (step 4), before the docs show the commands.
3. **Hugging Face: the user's personal account.**
4. **Licences**: the golf cart models as Apache-2.0. The NTU twin and Shalun maps are
   uploaded by the user, unlisted. swb 139 copied the TUM-repaired towns and the Town05 ring
   map to the NAS.

## Steps

1. [x] Audit (2026-11-14), above.
2. [x] Assets: inventory and NAS copies from 243, 118, 133, 139 and 241 (2026-11-14); inventories
   in the NAS's `home/aeon/hf-assets/`.
3. [x] The user's word on the doc set and the asset questions (Decisions).
4. [x] Launch fixes the docs need (342b3e2, 2026-11-14): package.xml exec_depends on what
   `stack.launch.py` resolves; `carla.launch.xml world_config:=` for the CARLA world server's
   config; `stack.launch.py` refuses a domain outside 0-57 and equal ego and scenario domains,
   before anything starts (a test pins it; checked under `ros2 launch` and play_launch 0.20.0,
   both print the message); `demo.launch.xml` and the `carla_scenario.launch.xml` alias
   deleted; every argument of `carla.launch.xml`, `isaac.launch.xml` and
   `carla_simulation.launch.xml` described.
5. [x] "Domains and DDS" and "Build your own stack" (8ae5110, 2026-11-14).
6. [x] README and the two getting-started guides onto the stack, no `just` (2026-11-14):
   the quick start is one `carla.launch.xml scenario:=... exit_on_scenario_end:=true`; the
   four-terminal form is each guide's appendix; swb_examples' README too.
7. [x] Tutorials 6 (CARLA's assets on Isaac) and 7 (road physics) added, e5d2147.
   Tutorials renumbered and fixed; reference docs fixed; the index (2026-11-14): five
   tutorials on the stack (first-run and the-stack retired into the guides), the user guide
   as the launch and environment reference, the agent guide's `ros2 run swb_agent swb-agent`
   and "With the stack", scenario-authoring's config homes, user-workflow marked a design
   record. Checked: no `just` recipe, no code line over 88 columns, no dash characters, no
   broken relative link in the user docs. Not yet run live (step 8).
8. [ ] Acceptance: every command in the guides run once on a quiet host (CARLA, Isaac);
   one guide followed verbatim by a fresh reader with nothing else.
9. [~] Hugging Face (2026-11-14): seven dataset repos under the user's account, all **private**
   for the user to review and open (decision: "Create them private first"), each with a card
   (source, licence, how made, use with swb):

   | Repo (`jerry73204/...`) | Files | Size | Licence |
   |---|---|---|---|
   | `carla-0.9.16-isaac-usd` (towns, Town07_Opt, vehicles, props, walkers) | 21829 | 25.5 GB | CC-BY-4.0 |
   | `haruna-tsukuba-courses` (Haruna, Tsukuba; `world.yaml` made relative) | 565 | 0.49 GB | ODbL + GSI |
   | `carla-town-autoware-maps` (TUM towns with swb's repairs, Town05 ring, Town01 point-cloud world) | 53 | 0.73 GB | LGPL-3.0 |
   | `isaac-road-physics-stages` (surfaces, proving ground, 066 procedural stages) | 77 | 2.6 MB | Apache-2.0 |
   | `newslab-golfcart-models` | 20 | 6 MB | Apache-2.0 |
   | `shalun-map` (`world.yaml` made relative) | 47 | 0.41 GB | SVL's, private |
   | `ntu-campus-maps` (twin map with its Isaac world versions, 133's additions, CARLA package) | 6303 | 20.4 GB | vendor scan, private |

   Left out of NTU: the raw 50 m scan tiles (184 GB), the build archive, the proof videos.
   Names carry no `swb-` prefix (user, 2026-11-14: "If it's not bound to swb, I prefer not to
   add swb- prefix"): the data is Isaac Sim's USD, Autoware's maps and CARLA's packages; only
   small sidecars (`world.yaml`, the vehicle catalogue JSON, `surfaces.yaml`) are swb's formats.
   **Public since 2026-11-14** (user: "For those except Shalun and NTU campus map, update
   licenses and notices on them and make them public"): `carla-0.9.16-isaac-usd`,
   `haruna-tsukuba-courses`, `carla-town-autoware-maps`, `isaac-road-physics-stages`,
   `newslab-golfcart-models`, each with a LICENSE and/or NOTICE (CARLA CC BY 4.0 with the
   trademark note; OSM ODbL and GSI's 加工 credit; TUM's LGPL-3.0 with the GPL-3 text and our
   modifications listed; Apache-2.0 with Nebula's VLP-32C table credited). The GSI tiles
   (`raw/dem5a/`, `raw/seamlessphoto/`, the debug mosaic and a stray DEM tile, unmodified
   server responses) were removed from the courses and its history squashed, so no revision
   serves them; `raw/jobs.txt` lists their URLs. `shalun-map` and `ntu-campus-maps` stay private.

## Gaps found writing tutorials 6 and 7 (2026-11-14)

Places where a user has no user-level path yet (the tutorials say so plainly):

- The 13-surface ground (`isaac-road-physics-stages`) has no lanelet map, so it runs only from
  its demo script, not the stack.
- `isaac.launch.xml` has no `record:=`; no ROS topic carries wheel or soil state.
- The rough road (rocks, potholes, washboard) is `road_detail.yaml` plus scripts under
  swb_isaac_world's `spikes/`, not an installed command.
- Building the CARLA USD set and a town's Isaac world are repository scripts
  (`isaac_town.sh`), not `ros2 run` targets.
- The Haruna course's `haruna_down.xosc` names the building host's map path and a
  `drift_mustang` ego; the user edits both.
- Fixed on the dataset side: `carla-0.9.16-isaac-usd`'s manifest now covers Town07_Opt and
  leaves out the card, so `sha256sum --check MANIFEST.sha256` and `asset_set check` pass on
  a download.

## Acceptance

- `grep -n 'just ' README.md docs/getting-started docs/tutorials docs/user-guide.md
  docs/agent-interface.md` finds no recipe.
- No code line over 88 columns in those files.
- Every command in "Build your own stack" ran once, with its output's key line recorded.
- A fresh reader with only the guide reaches a passed scenario on CARLA with Autoware, and
  with two cars.
