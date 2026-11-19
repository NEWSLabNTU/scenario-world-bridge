# 068: A share of Town03's lanelets sit about 1.3 m off their OpenDRIVE lanes

**Severity**: medium. On those lanes the ego's planned path, a scenario's lanelet poses and CARLA's
road disagree by more than a metre, and SSv2's lane matching sees it. **Status**: open.
**Found**: 2026-11-19, swb 241, checking the candidate revision of `jerry73204/carla-town-autoware-maps`
(roadmap 087; issue 067).

## What happens

`scripts/lanelet_opendrive_registration.py` measures each road lanelet against CARLA 0.9.16's OpenDRIVE.
On the Driving lanes, offset in metres:

| map | Town01 | Town02 | Town03 | Town05 | Town10 |
|---|---|---|---|---|---|
| e9411350 (projector scale 0.9996) | 0.159 | 0.123 | 1.446 | 0.106 | 0.060 |
| scale 1.0, repaired (PR #1) | 0.007 | 0.012 | **1.365** | 0.005 | 0.007 |

The scale factor brings every other town under 0.012 m (p95). Town03 still has a p95 of 1.365 m,
with a p50 of 0.057 m, so most of its lanes register and a share (5-10 %) sit about 1.3 m off. Its
other numbers:
- 2520 of 26943 sample points have no lane running the same way within 8 m;
- heading p99 28.5 degrees;
- the worst lanelets are 10-17 m off (132088, 116638, 102222, 116659, 100838 on Driving lanes).

The Bidirectional (p95 1.345 m) and Shoulder (1.407 m) lanes show the same band.

## Cause

Not known. The band near 1.3 m suggests one or more roads whose lanelets were drawn with another lane
width or lane offset than the OpenDRIVE gives, as with the pack's other width differences (width diff
p95 0.64 m, max 70.8 m). It is not the projector: the band is there at both scales. Next: map the
offsets by OpenDRIVE road and lane to see whether they cluster.

Not blocking the dataset's revision with the scale factor (swb 133's ruling).
