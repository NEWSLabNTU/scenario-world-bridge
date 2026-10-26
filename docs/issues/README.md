# Issues

One file per defect: what is wrong, why it matters, how it was confirmed, and the fix or
what is still open.

| # | Issue | Severity | Status |
|---|---|---|---|
| [001](001-soil-tile-rebuild-memory.md) | Each soil tile rebuild left cooked collision in the Isaac world server (~0.4 MB, then 64 KB); now no mesh is cooked after load | low | fixed |
| [002](002-mid-sand-mrm.md) | Emergency stops on town01_soil: NDT's step bound made it cycle between two poses to its iteration limit, a WARN that makes autonomous unavailable; fixed in the Autoware overlay | low | fixed |
| [003](003-world-reload-memory.md) | The Isaac world server grew ~130-150 MB per scenario: each ego's RTX lidar render product was never given back; the lidar is now kept per ego slot | low | fixed |
| [004](004-soil-resistance-at-rest.md) | Soil motion resistance was applied in full at standstill, so a car stopped on soil could not drive off at a normal request; now faded in below 0.1 m/s | medium | fixed |
| [005](005-isaac-town-render-loses-the-gpu.md) | Isaac Sim 6.1 loses the GPU (Xid 109, then 31) rendering a SimReady CARLA town on driver 615.71.09; needs a driver change (root) | high | open |
| [006](006-npc-bench-never-reaches-a-verdict.md) | The 20-NPC bench never got a verdict: SSv2 takes ~370 ms a frame with 20 NPCs, past the 400 s cap, and an unused ego stack ran beside it; no ego stack for ego-less scenarios, 1200 s cap | low | fixed |
| [007](007-onboard-control-in-the-bridge.md) | The car's onboard control (powertrain, ABS, TC, ESC, steering actuator) lived in aib, the vehicle bridge, and its ESC and ABS read PhysX's ground truth (friction, lateral speed, slip); moved to `sim_vehicle_ecu`, on the car's own sensors | medium | fixed |
| [008](008-vehicle-check-loses-the-dds-profile.md) | aib's `vehicle-check` waited forever when loopback multicast was on: Autoware's `setup.bash` replaced the caller's Cyclone profile, so the waiter never saw the Isaac ego | low | fixed |
| [009](009-ecu-slip-twice-physx.md) | The brake module's slip estimate divided by max(v, 4 m/s) where PhysX divides by v + 4 m/s: up to twice PhysX's slip at town speeds, so ABS and TC acted at half their tuned slip | medium | fixed |
| [010](010-soil-digs-at-physx-low-speed-slip.md) | Below town speeds the soil's slip sinkage read the wheel's real slip, where PhysX's tire (its 4 m/s slip denominator) needs ~0.5 m/s of overspeed for any drive: a creeping car's driven wheels read as spinning in place, dug in, and the car bogged down on mud; the soil now digs at the real slip with its denominator held at 4 m/s | medium | fixed |
| [011](011-world-server-hangs-on-a-kit-fatal.md) | The Isaac world server hung instead of exiting, once after RTX's "Out of resource descriptors!" and once in Kit's start; it now exits 75 on a Kit fatal or when not up in 20 min, and is respawned (the fatal's cause not found) | medium | fixed |
| [012](012-soil-grip-beyond-the-tires.md) | On soil the car's friction was the soil's whole shear limit, more than the measured tire's gross traction (soft mud let a rear-drive car drive that stalls), and on a snow layer's base the base's load had no grip (0.02); the soil's friction is now held at the class's measured `mu`, and the base grips at it | medium | fixed |
