# Games and simulators: road tilt, load from grade and vertical curvature, road-edge transitions

Scope note: tyre-contact and suspension handling of rocks, steps and enveloping (ACC 5-point tyre,
sweep vs ray, unsprung mass, physics rates) are covered in `reports/Rocks and road roughness for
rally.md`; this file only references them. The design gap these notes serve is autoware-drift
issue 014 (one edge margin everywhere, no hazard class per side, no bank or crest load in the
planner).

## Q1. How do games and sims represent the track or stage, separately for physics and for the AI, and what fields do they store per point?

### Takeaway
Every documented title splits the road in two: physics reads a dense mesh (or laser scan) whose
triangles carry a surface material (grip, resistance, bumps, legal/illegal), while the AI reads a
sparse centreline or racing-line spline whose per-point record holds the lateral extents (road
edge, legal limit, collision limit) and, in Assetto Corsa's case, camber, grade, radius and the
surface normal. No title found stores an explicit "what is past the edge" hazard class per side;
the closest are rFactor 2's separate "cut" and "collision" widths and the physics surface's
`Legal`/`IS_VALID_TRACK` flag.

### Cited Findings
**Assetto Corsa (AI line, `fast_lane.ai`)**
- The AI line file (version 7) holds, per point, a position (x, y, z), a cumulative length and an id, then an "extra" record per point with these floats in order: `Speed, Gas, Brake, ObsoleteLatG, Radius, SideLeft, SideRight, Camber, Direction, Normal (vec3), Length, ForwardVector (vec3), Tag, Grade`; `Width = SideLeft + SideRight`. This is reverse-engineered by Content Manager's author (community, but the de facto reference). [actools AiPointExtra.cs](https://github.com/gro-ove/actools/blob/e5e5c94b8dee7511def0b670bfd5dbb233831c11/AcTools/AiFile/AiPointExtra.cs)
- AssettoServer's parser reads the same layout and multiplies `Camber` by `Direction` ("either 1 or -1") to get a signed camber; it also accepts a compact "-1" version with position, radius and camber per point. [AssettoServer FastLaneParser.cs](https://github.com/compujuckel/AssettoServer/blob/main/AssettoServer/Server/Ai/Splines/FastLaneParser.cs)
- AssettoServer's traffic AI uses only `Radius` for corner speed (`CalculateMaxCorneringSpeedSquared(point.Radius, AiCorneringSpeedFactor)`, default factor 0.65, then a braking distance from the speed excess); `Camber` is used only to roll the rendered car (rotation Z), with an optional `SmoothCamber`. Grade is read and discarded. [AssettoServer AiState.cs](https://github.com/compujuckel/AssettoServer/blob/main/AssettoServer/Server/Ai/AiState.cs); [AiParams.cs](https://github.com/compujuckel/AssettoServer/blob/main/AssettoServer/Server/Configuration/Extra/AiParams.cs)
- The line is recorded by driving a hotlap with the AI app (`fast_lane.ai.candidate`); a modder tip is to mark every surface but ROAD as `IS_VALID_TRACK=0` so the recorded borders (SideLeft/SideRight) come out at the legal edge (community). [assettocorsamods.net thread](https://assettocorsamods.net/threads/how-can-i-make-ai-path.234/page-3)

**Assetto Corsa (physics surfaces, `surfaces.ini`)**
- Per surface: `FRICTION` (KERB example 0.92, ASPHALT 1.0), `DAMPING` ("slow-down factor"), `DIRT_ADDITIVE`, `IS_VALID_TRACK` ("Invalid surfaces trigger the penalty system"), `BLACK_FLAG_TIME`, `SIN_HEIGHT` (bump height in m) and `SIN_LENGTH` (bump wavelength), `IS_PITLANE`, `VIBRATION_GAIN/LENGTH`, sound. Meshes are bound to a surface by name prefix (e.g. `1GRASS`) (community wiki). [hagn99 surfaces.ini](https://www.sites.google.com/site/hagn99/assettocorsa/modding/tracks/data/surfaces-ini); [overtake.gg](https://www.overtake.gg/threads/pit-lane-and-crazy-ai.175179/)
- ACC replaced the single tyre contact point with five (two at the front edge of the footprint, one in the middle, two at the rear) because one point gave "buggy behaviour over curbs" and took "the worse possible condition of the stepped curb" (Kunos blog by Aris Vasilakos, reposted; in ACC from 1.0.7). See the rocks report for detail. [overtake.gg 5-point blog](https://overtake.gg/threads/assetto-corsa-competizione-the-5-point-tyre-model-blog.171148/page-2)

**rFactor 2 (AI: `.AIW`; physics: `.TDF`)**
- AIW workflow (Studio 397 tutorial): record a Main Path down the middle of the track, then "find corridors": "AI use corridors to get information on the width of the track, what is a legal racing surface as opposed to an illegal surface". Corridors show as red and white lines left and right of each waypoint, editable per waypoint; at the Monaco chicane the white lines were "slowly tapered in" before the braking zone so the AI "believe the track was getting more and more narrow" and entered single file. Driving lines: "fastest", auto-generated "left" and "right", optional "BLOCK", and per-car-type lines referenced from the talent file. [rF2 AIW Creation Tutorial (PDF)](https://www.studio-397.com/wp-content/uploads/2016/12/rF2_AIW_Creation_Tutorial.pdf)
- Per-waypoint widths (community, not official): `wp_width` = road left, road right, far left, far right (far = full driveable width including run-off); `wp_dwidth` = collision left, collision right (driveable before collision objects, "affects AI only"), cut left, cut right (the corner-cut boundaries). [racesimcentral AIW editing](https://www.community.racesimcentral.net/threads/aiw-file-editing.4795); [RaceDepartment corner cut](https://www.racedepartment.com/threads/track-corner-cut-adjustment.65913/)
- TDF `[FEEDBACK]` per surface (official page, values only): Road `Resistance 0.0, Legal true, CollFrict 0.4`, bumps from `RoadBumpAmp 0.002 m`, `RoadBumpLen 5.7 m`; Grass `Dry 0.66, Resistance 600.0, BumpAmp 0.049, BumpWavelen 3.0, Sink 0.016, Legal false`; Tyre barrier `Spring 20000, Damper 9000, Legal false, WallSkids 1`. [Studio 397 TDF](https://docs.studio-397.com/x/lgFDAg)
- A forum poster found resistance changes mainly moved lap time and top speed (community). [racesimcentral](https://www.community.racesimcentral.net/goto/post?id=3833)

**Gran Turismo Sport / GT Sophy (AI observation built from a course map)**
- GT Sophy: "a static map defining the left and right edges and the centre line"; the approaching course is "60 equally spaced 3D points along each edge of the track and the centre line", their span "a function of the current velocity, so as to always represent approximately the next 6 s of travel", in the agent's egocentric frame. Other inputs: 3D velocity, angular velocity, acceleration, "load on each tyre and tyre-slip angles", progress (sin/cos), "the local course surface inclination", heading relative to centreline, barrier-contact and off-course flags, last steering/throttle/brake. 10 Hz. [Wurman et al., Nature 2022 (PDF)](https://www.cs.utexas.edu/~pstone/Papers/bib2html-links/nature22.pdf)
- The paper notes the map "did not have all of the information about the track": humans "could see the kerbs and surface material outside the boundaries, whereas GT Sophy could only sense these by driving on them." [Wurman et al. 2022](https://www.cs.utexas.edu/~pstone/Papers/bib2html-links/nature22.pdf)
- Ablation: representing the upcoming track as point sequences beat "wall lidar, curvature". [Wurman et al. 2022, Extended Data](https://www.cs.utexas.edu/~pstone/Papers/bib2html-links/nature22.pdf)
- Earlier GTS agent (Fuchs et al.): 13 rangefinders every 15 degrees over the front 180 degrees (100 m range) to track edges, wall-contact flag, and 10 curvature samples of the centreline from 1.0 to 2.8 s ahead. [Fuchs et al., arXiv 2008.07971](https://arxiv.org/pdf/2008.07971)

**Forza / Turn 10 (racing-line patent)**
- Racing line = one lateral parameter chi_n per waypoint, waypoints "spaced 2 meters apart along the center line", point = centreline + chi_n times a normal of magnitude equal to the half-width; chi = tanh(gamma) keeps it in [-1, 1]; cost = squared curvature; "lines in the horizontal plane only are considered and height information is ignored"; speed from maximum sustainable lateral acceleration. [US7892078B2](https://patents.google.com/patent/US7892078)
- Forza Motorsport Update 20 moved Drivatars to "a multi-line AI system that is trained on multiple lines of traffic" that can "blend between different racing lines", aiming to "better limit collisions and off-track behaviors". [forza.net](https://forza.net/news/forza-motorsport-drivatar-changes)

**BeamNG**
- Navgraph (`map.json`) segments carry `drivability` ("Pathing preference/cost multiplier"; ~1 normal, 0.5 less preferred, below 0.5 strongly discouraged), `oneWay`, `speedLimit`, lane counts, type; when unset, speed limits are estimated "based on road width/radius, drivability, and road direction". No elevation, banking or edge fields are documented. [BeamNG map.json](https://documentation.beamng.com/modding/levels/level_formats/map/); [docs.beamng.com](https://docs.beamng.com/modding/levels/level_formats/map/)
- Ground models: `staticFrictionCoefficient` (asphalt 1.0, deep mud 0.55), `slidingFrictionCoefficient` (0.9 / 0.35), `roughnessCoefficient`, `defaultDepth` (mud 0.25), `hydrodynamicFriction`, `stribeckVelocity`, `strength`, `fluidDensity`, `flowConsistencyIndex`, `flowBehaviorIndex`, `shearStrength`. [BeamNG groundmodels.json](https://documentation.beamng.com/modding/levels/level_formats/groundmodels/)
- AI "Risk"/aggression scales speed, cornering and acceleration toward tyre limits (community: range 0.42-1.44). [BeamNG forum](https://www.beamng.com/threads/any-tips-for-better-faster-ai-driving.88048); a BeamNG.tech forum answer says waypoint following uses a velocity profile from "road curvature, predicted friction limits, dynamic obstacles" and comfort, citing Heilmeier et al. 2020. [forum.beamng.tech](https://forum.beamng.tech/t/vehicle-ai-control/422)

**iRacing**
- Tracks are laser scanned, iRacing states accuracy "within 2 mm", reproducing bumps and cracks. [iracing.com](https://www.iracing.com/in-good-company/); a community view: small sharp bumps may be lost while slopes are captured well. [LFS forum](https://lfs.net/forum/post/1412128)

**Richard Burns Rally**
- Warthog's notes: ground built from polygons whose textures carry material maps, so one stretch mixes gravel depths and states; slip computed from wear, ground material and pressure; each tyre has its own mass. Community physics files list gravel `SinkFactor`, `SoilThickness`, `RollingResistance`. [bhms.racesimcentral RBR facts](https://bhms.racesimcentral.net/print/150)

**Colin McRae Rally 2.0 (Codemasters, Jeff Hannan)**
- Hannan recorded his own laps to derive both racing lines and neural-net training data; on narrow rally roads "the racing line could be considered to be the centre of the road"; road ahead sampled at distances like "5m, 10m, 20m ahead"; outputs were on/off controller flags; "a different model for each road surface". Exact inputs withheld as "commercially sensitive". [Hannan notes, repost](https://blog.csdn.net/xueyong1203/article/details/4212708)

### Inferences
- A useful per-station course record for an autonomous planner, assembled from these: s, x, y, z, heading, curvature, grade, bank (signed camber), surface normal, and per side: legal edge, driveable edge (run-off end), collision edge (barrier), cut edge, plus a surface class with friction/resistance/legal flag. AC supplies the geometric half, rF2 the four widths, TDF/surfaces.ini/BeamNG the material half. Issue 014's `drop / barrier / runoff` class maps to rF2's distinction between "far" (driveable) width and "collision" width: barrier = collision edge near legal edge; runoff = far edge well beyond legal edge; drop = neither (no driveable surface, no barrier).
- GT Sophy shows a learned agent can use raw 3D edge points (z included) plus surface inclination and tyre loads instead of hand-made bank and crest features; a planner can derive bank and vertical curvature from the same edge points.
- The Forza patent and AssettoServer both ignore height in the line and the speed profile; commercial AI typically handles elevation implicitly through recorded speeds (AC's per-point Speed/Gas/Brake) rather than explicitly.

### Gaps
- No official spec of rF2's AIW field semantics found; the four-value widths are community descriptions.
- Kunos's own use of `Camber` and `Grade` inside the AC/ACC AI was not found (only the community server's use).
- iRacing, Forza, Gran Turismo 7, Project CARS/AMS2, F1 games, EA WRC/DiRT and Trackmania AI track formats: no primary documentation found.
- BeamNG `ai.lua` internals (whether slope or bank enter its speed plan) not verified from source.

## Q2. How do AI drivers use bank, elevation and crests (speed on crests, camber in corners, downhill braking, jumps), and what do rally pace notes encode?

### Takeaway
Explicit bank or crest terms in game AI are almost undocumented: published systems either learn
them (GT Sophy sees 3D points, surface inclination and tyre loads; CMR2's nets learned per
surface) or bake them into a recorded speed trace (AC's per-point speed, gas, brake). Rally pace
notes are the richest human encoding of vertical and lateral road shape plus edge hazards, and
they are a ready vocabulary for course annotations.

### Cited Findings
- GT Sophy's observation includes "the local course surface inclination" and "load on each tyre" and 3D edge points; no hand-written crest or bank rule is described. [Wurman et al. 2022](https://www.cs.utexas.edu/~pstone/Papers/bib2html-links/nature22.pdf)
- The Fuchs et al. GTS agent saw only planar curvature and rangefinders, no elevation, and still reached super-human lap times on the tested tracks. [Fuchs et al. 2020](https://arxiv.org/pdf/2008.07971)
- The GameAIPro racing architecture describes AI skill applied by "multiplying the actual available grip in calculations of cornering speed and braking distances", and a mistake characteristic as "random errors in corner speed calculation"; it gives no crest or bank term. [Tomlinson and Melder, GameAIPro ch. 38](https://www.gameaipro.com/GameAIPro/GameAIPro_Chapter38_An_Architecture_Overview_for_AI_in_Racing_Games.pdf)
- AC AI points carry Camber and Grade (Q1), but the only public consumer found (AssettoServer) uses radius alone for speed. [AssettoServer AiState.cs](https://github.com/compujuckel/AssettoServer/blob/main/AssettoServer/Server/Ai/AiState.cs)
- Pace notes, general: severity scales differ ("British Club" 1 gentlest to 5 near 90 degrees; "McRae" 6 near straight to 1 hairpin); "square", "hairpin"; "flat (maximum speed) into crest"; "jump (caution!) into immediate right hand bend"; "caution, do not cut the corner, due to hazard on the inside"; "adverse camber" is a noted feature; "tightens". [Wikipedia: Pacenotes](https://en.wikipedia.org/wiki/Pacenotes)
- "Don't cut" (n.c.): do not take the inside, because of a hazard (rock, drop, tree, no road) on the inside; "over crest": a crest starting within a corner; "smCr"/"Cr" small crest / crest; "keep in" / "keep middle" / "stay L, middle, R" through a corner or crest (glossaries vary by crew and game). [racemarket.net](https://racemarket.net/blog/rally-pacenotes-what-why-how); [GameSkinny DiRT 4 guide](https://www.gameskinny.com/dy3uy/dirt-4-pacenote-guide-what-do-pacenotes-actually-mean); [crash.net](https://www.crash.net/wrc/feature/112400/1/team-talk-pace-notes-explained)
- EA SPORTS WRC: co-driver timing is adjustable from very late to early; "Simplified" pace notes exist; co-drivers warn of "crests and jumps on the road, bridges, water hazards, changes of surface". [EA beginner's guide](https://www.ea.com/games/ea-sports-wrc/wrc-24/news/ea-sports-wrc-beginners-guide). Players asked for more detail on "caution"/"don't cut" (e.g. "hairpin right, dont cut, rock inside") (community). [EA forum](https://forums.ea.com/discussions/wrc-general-discussion-en/tweaks-to-pacenotes-cautiondont-cut-more-detail-please/8367331)
- Jumps (real drivers): Rovanpera advises a light brake touch before the crest to balance the car [Red Bull](https://www.redbull.com/int-en/how-to-jump-a-rally-car-by-kalle-rovanpera); Team O'Neil warns not to brake or lift right at the crest because the nose dives [Motor Authority](https://www.motorauthority.com/news/1124943_let-team-o-neil-rally-school-teach-you-how-to-get-your-car-airborne); Mikkelsen: if a braking zone or corner follows, land on the front wheels and brake over the jump; the fastest way is flat out unless what follows forbids it [auto123 / wrc.com](https://www.auto123.com/en/news/rally-how-to-jump-a-world-rally-car/17964); cars "don't have wings and control surfaces" so attitude is set before take-off [DirtFish](https://dirtfish.com/?p=54296); braking too much ends "with a hard front-axle landing" [Skoda Storyboard](https://www.skoda-storyboard.com/en/models/how-to-fly-a-rally-car/).

### Inferences
- Standard physics for a planner (not from a game source): on a bank angle phi toward the corner centre, the friction-limited speed is v^2 = g R (mu + tan phi) / (1 - mu tan phi), with phi negative off-camber. Over a crest of vertical curvature kappa_v (1/m, convex), normal load per unit mass drops from g cos(theta) to g cos(theta) - v^2 kappa_v, reaching zero (take-off) at v^2 = g / kappa_v; in a compression it rises. These give the bank and crest terms that issue 014 lists as missing from L1/L2.
- Pace-note tokens map cleanly onto course annotations: crest severity -> kappa_v and take-off speed; "over crest" -> crest coincident with curvature (combined-load reduction); "opens/tightens" -> curvature derivative; "don't cut" / "rock inside" / "trees outside" / "drop" -> per-side hazard class and a forbidden inside band; "keep in/middle" -> a lateral corridor constraint; "caution" multiplicity -> risk weight.
- Jump technique translates to a planner rule: set pitch attitude (speed, brake release) before the take-off point; choose take-off speed by what the landing zone allows (a corner or braking zone after the landing caps speed).

### Gaps
- No primary source found for how Codemasters, Kylotonn, Polyphony or Turn 10 AIs explicitly slow for crests or use bank; nor any published rally-game AI that drives stages physically (rally opponents' times appear simulated, but no developer source confirms it).
- No authoritative EA WRC call list ("bad camber", "opens over crest", "caution drop", "into ditch") was retrievable; the Overtake.gg guide (403) and in-game audio are the remaining sources.

## Q3. What happens at the road edge in game AI: walls vs run-off, deliberate use of run-off, recovery, track limits, risk weighting by hazard?

### Takeaway
Game AI encodes edges as different widths (legal, cut, collision, driveable) and prices leaving
them with scalar penalties scaled by speed or kinetic energy; hazard type enters only through
which width or penalty applies. Recovery is a separate behaviour triggered by integrated
off-track utility. Track-limit rules (FIA, GT Sport via Sophy) count a car off when all wheels
(F1) or three or more (GT) cross the line, with kerbs on the outside.

### Cited Findings
- GT Sophy rewards: progress masked while off course ("To reduce the incentive to cut corners"); off-course penalty proportional to speed squared times time off course (speed, not squared, at Sarthe; doubled for the hardest chicanes); wall penalty proportional to time in wall contact times speed squared; tyre-slip penalty. Removing the off-course penalty gave slower laps and many fewer laps within bounds. "Off course" is the game's judgement, "three or more tyres are out of bounds", because "kerbs and other tarmac areas outside the track edges are often considered in bounds". [Wurman et al. 2022](https://www.cs.utexas.edu/~pstone/Papers/bib2html-links/nature22.pdf)
- Fuchs et al.: wall penalty c_w |v|^2 while in contact, c_w = 5e-4; without it the policy "did not brake and simply grinded along the track's walls"; with fixed penalties it either ignored them or stood still. The built-in GTS AI "drives with a safety margin to the track's walls, leading to tighter radii and a reduced speed". On a track where one border is a wall and the other a painted line, the agent used both. [Fuchs et al. 2020](https://arxiv.org/pdf/2008.07971)
- rF2: corridors define legal vs illegal surface for the AI; separate collision widths "affect AI only"; cut widths define corner-cut detection; corridors can be narrowed by hand to shape AI behaviour. [rF2 AIW tutorial](https://www.studio-397.com/wp-content/uploads/2016/12/rF2_AIW_Creation_Tutorial.pdf); [racesimcentral (community)](https://www.community.racesimcentral.net/threads/aiw-file-editing.4795)
- GameAIPro: a short-range avoidance layer handles "racing close to a solid wall"; recovery: "drive towards the nearest edge of the track at a modest speed, giving way to any cars", first stabilise and stop sliding; trigger by integrating utility over seconds, e.g. at 10 Hz "add 10 points for every meter off the track and 10 points for every wheel that has lost grip, but also subtract 20 points every frame". [GameAIPro ch. 38](https://www.gameaipro.com/GameAIPro/GameAIPro_Chapter38_An_Architecture_Overview_for_AI_in_Racing_Games.pdf)
- AC: surfaces flagged `IS_VALID_TRACK=0` trigger the cut/penalty system; `BLACK_FLAG_TIME 0` disables cut registration in races on some servers (community). [Emperor Servers wiki](https://wiki.emperorservers.com/assetto-corsa-server-manager/fixing-track-cut-detection-in-race-sessions)
- BeamNG: route cost scales with segment drivability; low-drivability roads are avoided when there is a choice (community and docs). [BeamNG map.json](https://documentation.beamng.com/modding/levels/level_formats/map/)
- FIA F1: white lines are part of the track, kerbs are not; a car has left the track when no part of it touches the track; "Once there's no part of the car on the white line, that's it" (Wittich). [PlanetF1](https://www.planetf1.com/news/explained-f1-track-limits-rules); [formula1.com, Red Bull Ring](https://www.formula1.com/en/latest/article/explained-how-key-changes-to-the-red-bull-ring-look-to-avoid-last-years.2ubjMYkbT3HHpwuEJ7PoHw)
- Physical deterrent instead of a rule: a 2.5 m gravel strip behind the kerb at Red Bull Ring Turns 9-10, because "when you run over the gravel, it's impossible to be faster". [formula1.com](https://www.formula1.com/en/latest/article/explained-how-key-changes-to-the-red-bull-ring-look-to-avoid-last-years.2ubjMYkbT3HHpwuEJ7PoHw)
- Forza Update 20 aimed to reduce "off-track behaviors" and "awkward mergers when they attempt to return to the ideal racing line". [forza.net](https://forza.net/news/forza-motorsport-drivatar-changes)
- Real-world penalty of a ditch: on Rally Poland cars that slid off into a ditch needed spectators to push them out (large time loss); "ditch-hooking" (dropping a wheel into a ditch to pull the car round) exists but is described as risky. [The Drive](https://www.thedrive.com/watch-this/9430/watch-these-beautiful-rally-cars-crash-into-a-ditch-one-by-one.md); [Jalopnik](https://jalopnik.com/behold-the-dumbest-thing-ive-ever-seen-on-a-rally-stage-1820590843)

### Inferences
- Borrowable penalty forms: a kinetic-energy-scaled cost (speed squared) for wall contact and for leaving onto run-off works where fixed penalties failed (Fuchs). Per hazard class: drop -> hard constraint (infinite cost, plus no planned slide); barrier -> v^2-scaled contact cost with a small margin; run-off -> v-scaled or time-loss cost from surface deceleration, allowing deliberate use when the gain exceeds it. That is a direct answer to issue 014's "margin per class".
- The "three or more wheels" and "no part on the white line" rules suggest scoring off-road events by wheel count, which a plant can report per step and which separates "two wheels off" (recoverable, often legal on kerbs) from leaving.
- GameAIPro's integrated utility trigger is a ready debounce for switching the planner into a recovery mode.

### Gaps
- No game source found that weights edge risk differently for a drop vs a barrier vs grass in the AI's line (beyond widths); touge/hillclimb games (e.g. Initial D-style, AC touge mods) not documented.
- How iRacing, ACC and F1 games' AI decide to use run-off deliberately was not found.

## Q4. How do sims model the physics of edges: surface transitions, gravel trap deceleration, kerbs and sausage kerbs, verge lip, split friction?

### Takeaway
Sims model edges almost entirely as material changes on the drivable mesh (friction scale,
rolling resistance, sink, bumps) plus geometry for kerbs; split friction falls out naturally
because each wheel samples its own surface. Real-world numbers for gravel beds (0.3 g coasting
to about 1.1 g) and edge drop-offs (scrubbing on a vertical lip; 30 degree Safety Edge) give
calibration targets.

### Cited Findings
- rF2 grass vs road: grip 0.66 vs 1.0 (scaled), Resistance 600 vs 0, BumpAmp 0.049 m vs 0.002 m, wavelength 3.0 vs 5.7 m, Sink 0.016, Legal false. [Studio 397 TDF](https://docs.studio-397.com/x/lgFDAg)
- AC kerb friction 0.92 vs asphalt 1.0, with DAMPING as a slow-down factor and SIN_HEIGHT/SIN_LENGTH bump synthesis (community wiki). [hagn99](https://www.sites.google.com/site/hagn99/assettocorsa/modding/tracks/data/surfaces-ini)
- BeamNG: static and sliding friction per ground model plus depth, fluid and non-Newtonian flow terms for soft ground (mud 0.55/0.35 static/sliding, depth 0.25). [BeamNG ground models](https://documentation.beamng.com/modding/levels/level_formats/groundmodels/)
- ACC's 5-point contact was introduced for kerbs, because a single centre point sees the worst of a stepped kerb; a community description: "one point in the air, one on the kerb, one on the tarmac, 2 on the grass" (details in the rocks report). [overtake.gg](https://overtake.gg/threads/assetto-corsa-competizione-the-5-point-tyre-model-blog.171148/page-2)
- Gravel traps: FIA's report on Schumacher's 1999 Silverstone crash gave an average 1.1 g in the gravel trap [AtlasF1 archive](https://archive.atlasf1.com/news/1999/1438.htm); passenger cars on arrester beds about 0.3 g coasting, up to 0.9 g braking (Ambroz et al.) [Univ. Ljubljana](https://kmtm.fs.uni-lj.si/?p=1041); 0.45 g unbraked at about 30 mph, rounded gravel slightly more effective than angular (TRRL, 1966) [TRID](https://trid.trb.org/View/29168); truck arrester beds 0.516 g average [TRID](https://trid.trb.org/View/287043).
- Sausage kerbs launch cars: one launched F3's Peroni at Monza's Parabolica and was removed; Spa's new Eau Rouge sausage kerbs were removed after day one; drivers cite back injuries; the FIA resisted a blanket removal. [Autosport, Parabolica](https://www.autosport.com/f1/news/145830/parabolica-sausage-kerb-removed-after-peroni-crash); [Autosport, Spa](https://www.autosport.com/f1/news/belgian-gp-f1-fia-removes-new-kerbs-at-spas-eau-rouge-5002508/5002508/); [Crash.net](https://crash.net/f1/news/928905/1/fia-warns-against-kneejerk-removal-sausage-kerbs)
- Pavement edge drop-off: heights range from under 1 inch to 6 inches or more; on re-entry "the side of the tire may scrub along the drop-off, resisting the driver's attempts"; the Safety Edge slopes the edge at 30 degrees to reduce that resistance; past criterion: drop-offs above 2 inches may affect safety. [FHWA-HRT-11-024](https://www.fhwa.dot.gov/publications/research/safety/11024/001.cfm); [FHWA EDC brochure](https://www.fhwa.dot.gov/innovation/pdfs/factsheets/edc/brochure.pdf); [Iowa State](https://cwims.intrans.iastate.edu/wp-content/uploads/2018/03/edge_drop-off.pdf) (minimum 45 degree taper, 30 preferred)
- Split friction on the road: Manitoba MPI advises not braking with wheels off because of uneven traction between pavement and gravel. [MPI archive](https://bac-lac.wayback.archive-it.org/web/20080502204829/http://www.mpi.mb.ca/english/dr_tips/OffRoadRecovery.html)

### Inferences
- An edge model for the testbed needs three separate things: a per-wheel surface class (friction, resistance, sink, so split-mu is automatic), the verge geometry (lip height and slope, which decides re-entry scrub), and the run-off deceleration (0.3-1.1 g by gravel type and braking) to turn "off road" into a recoverable stop vs crash in the plant scoring issue 014 asks for.
- A vertical lip above roughly 5 cm acts like the cleats in roadmap 066's fixtures; the rocks report's sweep/enveloping discussion applies to it directly.

### Gaps
- No sim documentation found for an explicit tarmac/verge lip parameter or for gravel-trap deceleration calibration in a game.
- Measured lateral friction of grass or gravel run-off in sims was not found beyond TDF/AC scalar values.

## Q5. What do real drivers and coaches teach about banked and off-camber corners, crests, and dropping wheels off the road?

### Takeaway
Coaching is consistent: on-camber acts as banking and allows more speed; off-camber and crown
crossings cut grip, so brake earlier and smoother and sometimes apex early to use the on-camber
part; ease power and open the hands over crests and feed it back on landing; with wheels off,
lift, straddle the edge, slow, then steer back with a small input and recentre.

### Cited Findings
- Team O'Neil: "the on camber slope of the road acts like a banking"; off camber "you can't carry nearly as much speed"; brake "smoother and earlier"; "The transition in camber upsetting the car is the cause for more crashes then you would think"; at the limit treat the crown "almost like a crest or a jump"; normally no early apex, but on an off-camber corner going inside early to use the on-camber section is the exception. [Team O'Neil: road camber](https://teamoneil.com/blog/road-camber-explained/)
- Kenton Koch (Mazda): over a crest the car goes light; come off the power slightly, open the hands, feed power as it lands and settles; a brake release that suits a smooth corner may be clumsy over a crest. [Mazda Motorsports tip #17](https://www.mazdamotorsports.com/2020/05/11/kenton-koch-on-driving-technical-corners/)
- Off-camber entries exaggerate understeer; brake release timing matters; precise trail braking helps rotation (Ross Bentley). [Speed Secrets](https://speedsecrets.com/q-how-do-i-adapt-my-braking-and-turn-in-for-off-camber-corners)
- Wheels off (road driver training, GM manuals): ease off the accelerator, straddle the pavement edge, steer back once the front tyre meets the edge, then straighten; Ontario guidance: recentre before accelerating; braking advice differs (MPI: do not brake; a fleet-safety tip: brake gently). [GMC manual via manualslib](https://www.manualslib.com/manual/574312/Gmc-Canyon.html?page=253); [MPI](https://bac-lac.wayback.archive-it.org/web/20080502204829/http://www.mpi.mb.ca/english/dr_tips/OffRoadRecovery.html); [Automotive Fleet](https://www.automotive-fleet.com/news/fleet-safety-tip-of-the-week-off-road-recovery-procedures)
- Jumps: see Q2 (Rovanpera, O'Neil, Mikkelsen, DirtFish, Skoda).

### Inferences
- These convert to planner rules: (1) bank-aware speed limit with signed camber; (2) treat camber transitions and crown crossings as grip dips (a short-scale friction reduction) and avoid peak combined slip there; (3) on crests limit longitudinal and lateral demand by the reduced normal load and avoid steering or braking peaks at the unloading point; (4) after an excursion, a recovery mode: low steering rate, straddle, reduced speed before re-entry over a lip.

### Gaps
- No primary touge-specific coaching source found (gutter use, "mizo-otoshi" style ditch hooking) beyond the rally ditch-hook anecdote; no source on "two wheels off" deliberately on rally stages beyond pace-note "cut" usage.

## Q6. Which studio papers and talks exist, and what is GT Sophy's observation space?

### Takeaway
Primary technical sources are few: the GT Sophy Nature paper (full observation and reward
spec), Fuchs et al. on GTS, the GameAIPro racing architecture chapter, Turn 10's racing-line
patent, rF2's AIW tutorial, and Hannan's CMR2 notes. GDC talks on Forza or Codemasters rally AI
were not retrievable.

### Cited Findings
- GT Sophy observation and reward: see Q1 and Q3 (60 3D points per edge and centre over about 6 s, surface inclination, tyre loads and slip, off-course and wall flags; speed-squared off-course and wall penalties; 10 Hz; trained on about 1,000 PS4s per press). [Wurman et al. 2022](https://www.cs.utexas.edu/~pstone/Papers/bib2html-links/nature22.pdf); [GTPlanet](https://www.gtplanet.net/how-gran-turismo-sophy-actually-works-more-details-on-polyphony-digital-and-sony-ais-new-technology/)
- GameAIPro ch. 38 (Tomlinson and Melder): strategic/tactical/control layers, collision avoidance near and far, personas scaling grip, recovery, offline line optimisation by bisection or GA (citing Biasillo 2002). [GameAIPro](https://www.gameaipro.com/GameAIPro/GameAIPro_Chapter38_An_Architecture_Overview_for_AI_in_Racing_Games.pdf)
- Turn 10 / Microsoft racing-line optimisation (Herbrich, Tipping, Hatton): curvature minimisation on a 2 m waypoint lattice. [US7892078B2](https://patents.google.com/patent/US7892078)
- Codemasters CMR2: neural nets from recorded laps, per-surface models. [Hannan notes](https://blog.csdn.net/xueyong1203/article/details/4212708)

### Inferences
- For an autonomous race planner the strongest borrowable package is GT Sophy's: an egocentric, speed-scaled horizon of 3D edge points (giving bank and crest implicitly) plus a per-edge annotation that GT Sophy lacked (kerb/surface/hazard outside the boundary, which the paper names as a human advantage).

### Gaps
- No GDC talk text found for Forza Drivatar internals, Codemasters/EA WRC AI, Kunos AI or Polyphony's own (non-Sophy) AI; Microsoft Research's Drivatar theory page was not retrieved.
