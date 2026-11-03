# 033: The Isaac world server reports the ego scraping the road's collision tiles as a contact with the static world

**Severity**: medium for roadmap 066/067 on Isaac: a scenario run on a road with detail would
fail its physical verdict for touching its own road. **Status**: fixed in swb_isaac_world 74882b2
(`world.py`, `_load_road`). **Found**: 2026-11-02 on newslab-server-241, reading the world
server's contact collection while working on 066 R4.

## What happens

The world server reports each reporting body's contacts in `StepReply.contacts`, except those
with the ground: `World._collect_contacts` drops a contact whose other prim is under one of
`self.ground` (the world's road and ground prims, or the ground plane). Everything else goes
out with `other = 0`, the static world.

A world with road detail (066 R3/R4, `road_detail.yaml`) streams collision tiles of the true
road surface under `/World/RoadCollision` (`rock_layer.COLL_ROOT`): the rocks rest on them,
and the ego's chassis and skid plate collide with them (the wheels' query does not). They
were not in `self.ground`. Where the body comes down to the road (the washboard and the water
bars of R3's corridor at speed: 1-6 kN s on the skid plate over 800 m at 80 km/h in the rock
spike), each tile it touched would go out as a static-world contact. The bridge files it in
the ego's run as `is_static`, and `physical_verdict` fails a scenario on any static contact
it did not declare (`csbExpectedContacts`), as "ego hit static world".

Not seen in a scenario run yet: no scenario has run on a world with road detail. Found by
reading the code against the rock spike's contact report.

## Fix

`_load_road` adds `rock_layer.COLL_ROOT` to `self.ground` once it makes the rock stream (and
clears the ground cache): the tiles are the road's surface, as the world's own road prims
are. Contacts with rocks (`/World/RoadRocks`) are still reported: a skid plate striking a rock
is an event, and a scenario on a rock road that means to allow it declares `static`.
