# 013: every surface grips a third more than its class says

**Severity**: medium (off dry asphalt every ground is a third grippier than written: the
measured sand, mud and ice, and the textbook gravel, dirt, grass and snow). **Status**: fixed (swb_isaac_world: asphalt's `mu` 1.12, concrete's 1.05).
**Found**: 2026-10-27, roadmap 053 (ice), the car's realised curve against the measured one.

## What is wrong

The class table's `mu` is a peak friction coefficient, and the surface layer writes it to a
wheel relative to the reference class's (`surfaces.peak_mu`: `mu / table[reference].mu`), so
on dry asphalt a wheel keeps the rig's own tire friction and a map of asphalt drives as no
map. That holds only if the reference class's `mu` is what the car's tire actually does on
dry asphalt. It is not: asphalt's is 0.85, a textbook figure, while the Model 3's tire (the
rig from sim_vehicle_spec, its curve and `tire_friction`) peaks at 1.13 lengthways and
1.08-1.16 sideways on the dry pad (`play_log/manoeuvres/2026-10-25_t1-sweeps2`, sweep_long
front 1.127, rear 1.115; sweep_lat 1.157, 1.082). So every class is realised at 1.13 / 0.85 =
1.33 times its `mu`.

Seen on ice (`play_log/manoeuvres/2026-10-27_ice`, sweep_long): the class's 0.265, measured on
bare ice, came out 0.35-0.36 on the car; the water film's 0.125 came out 0.16-0.17. The
soil's friction goes the same way (`soil.grip` over the reference's `mu`), so the measured
gross traction of sand and mud (issue 012) is a third too high on the car as well.

The car's own 1.13 is plausible for a road tire on dry asphalt and agrees with the spec's
stop from 100 km/h in 42.9 m (sim_vehicle_spec, `carla.physics.tire_friction_scale`); it is
the table's reference that is off.

## Fix

The reference class's `mu` is the car's tire on dry reference ground, measured: asphalt
1.12 (the mean of the sweep's four axle peaks, 1.12). Every other class keeps its `mu` as an
absolute coefficient, now realised as written; concrete, the other paved road the tire meets,
keeps its ratio to asphalt (0.8 / 0.85: 1.05). A test holds the table's reference to the rig
(the sweep's numbers), so a tire change that moves it fails there.

Checked on the car (`play_log/manoeuvres/2026-10-27_ice-013`, sweep_long): bare ice now peaks
at 0.27 (class 0.265, measured 0.265), ice under a water film at 0.126 (0.125). The soil
classes drive as their measured gross traction says (`play_log/soil_drive/2026-10-27_013-*`):
sand and snow drive and stop (from 15 m/s in 33 and 43 m), sand_loose and mud_soft stall from
rest; mud_wet holds 10 m/s but no longer starts from rest (issue 014: what a real rear-drive
car does there). The Town01 soil scenarios after it: six of seven pass, `town01_mud_stop` as
before (053, step 5).

