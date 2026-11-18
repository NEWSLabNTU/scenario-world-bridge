# 063: An Isaac lidar with `rayType GAUSSIAN_BEAM` crashes the RTX sensor plugin

**Severity**: low. Nothing ships with it: aib's lidars run `IDEALIZED`. But a lidar that traces
its beam footprint is not available on Isaac 6.1 at the spec lidar's resolution. **Status**:
known limitation of Isaac 6.1, no workaround. **Found**: 2026-11-19, roadmap 084d, giving
the spec lidar (a Velodyne VLS-128) the datasheet's beam divergence.

## What happens

The world server starts. On the lidar's first frames it dies in the RTX sensor plugin:

```
[Fatal] [carb.crashreporter-breakpad.plugin] 002: libgpu.foundation.plugin.so!std::vector<unsigned char, ...>::_M_default_append(unsigned long)+0x9212
[Fatal] [carb.crashreporter-breakpad.plugin] 003: libgpu.foundation.plugin.so!...rtx::rendergraph::ResourceDebug...
[Fatal] [carb.crashreporter-breakpad.plugin] 004: librtx.rtxsensor.plugin.so!std::_Rb_tree<unsigned int, ...>
```

Its supervisor respawns it, and each respawn dies the same way (56 times in 30 minutes). The
client waiting on `Hello` times out.

## Repro

aib's spec lidar profile (`aib.lidar_profile`) on the Tesla Model 3's `top` lidar:

- 128 rings, -25..+15°, 1024 columns at 20 Hz (131 072 beams a scan);
- auxiliary output FULL (aib's intensity table, roadmap 084);
- `rayType GAUSSIAN_BEAM`;
- `divergenceHorDeg 0.1197482`, `divergenceVerDeg 0.06016057` (VLS-128 user manual, Table F-1).

`GAUSSIAN_BEAM` traces several rays per beam to model the footprint, so each scan traces some
multiple of those 131 072 rays. NVIDIA's lidar docs (the `omni.sensors.nv.lidar` 3.0.0
parameterization tutorials) warn of "a substantial performance cost". Here it crashes outright.
In aib, `ray_type: GAUSSIAN_BEAM` in a lidar's profile (branch `084d-divergence`, not landed)
reproduces it in Town01 within the first scan. Not tried: a lower resolution, auxiliary
output off, or a later Isaac.

## Consequence

On `IDEALIZED` rays, divergence enters only the return's power, not its geometry. A thin
object is hit as by a zero-width line: on a still ego, in every scan or in none, depending on
where the fixed columns fall (084d: a 0.105 m pole at 20 m, 0 hits a scan against 15
expected). Roadmap 084e approaches the real sensor's average instead, by drifting the column
phase from scan to scan.
