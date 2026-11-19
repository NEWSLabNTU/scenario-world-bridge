# 067: The local_coords map repair writes a second local_x/local_y on every node

**Severity**: high. Every node of a repaired map carries two `local_x` and two `local_y` tags, the
new values and the stale ones, and a reader that keeps the last tag of a key (Python's ElementTree
dict idiom, scensim's light table, the other repairs) reads the stale ones. **Status**: fixed in acb
(`carla_map_gen.repairs.local_coords` on the XML parser). **Found**: 2026-11-19, swb 241, checking
swb 243's candidate revision of the map dataset `jerry73204/carla-town-autoware-maps` (PR #1,
`a79f3ca5`, the roadmap 087 repairs and the scale factor over `e9411350`).

## What happens

In every town of the candidate, node 1 of Town01 for example:

```
<node id="1" ... lat="-1.805613721080238e-07" lon="0.003454829852879983">
  <tag k="local_x" v="384.5899"/>
  <tag k="local_y" v="-0.0200"/>
  <tag k="local_x" v="384.4361" />
  <tag k="local_y" v="-0.0200" />
  <tag k="ele" v="0" />
```

Town01 has 86164 `local_x` tags for 43082 nodes. The first pair is the projector's at scale 1.0. The second
is the dataset's old pair at 0.9996, 0.15 m off at x = 384 m. A run of `download_maps.sh`'s
`repair_maps` over `e9411350`, with acb at `8764a89`, gives byte-identical maps, so the cause is
the repair, not the upload.

## Cause

`repair_maps` runs the repairs in order. The ones before `local_coords` (traffic_lights, stop_lines,
subtypes, line_types, bound_directions) read and write the map through `xml.etree.ElementTree`. When
one of them changes the map, the whole file is written back, and ElementTree writes an empty element
as `<tag k="..." v="..." />`, with a space before `/>`.

`local_coords` edits the text with regular expressions and looks for the existing tags as
`<tag k="local_x" v="[^"]*"/>`, with no space. On a file one of the earlier repairs has rewritten, it
does not find them, takes them as missing and puts a new pair in front. On its next run it finds its
own pair (`"/>`) and updates that one, so the stale pair stays for good.

A map the earlier repairs leave alone keeps the pack's `"/>` and is repaired correctly. That is why
hosts that had already repaired their maps (this one's Town01 and Town05: one `local_x` per node)
did not show it.
A fresh install, or any repair that changes a map, triggers it: the scale factor (roadmap 087,
`1bbdd63`) needs local_coords to rewrite every node, and so do the traffic-light and stop-line repairs
on the dataset's unrepaired maps.

## Fix

`local_coords` reads and writes the map through ElementTree, as the other repairs do. On every node it
sets one `local_x` and one `local_y` from the projector, keeps the first of each, and removes any
duplicates. A new node's tags go first. The repair is idempotent.

An end-to-end test runs the whole chain, `carla_map_gen.repair` with every step in `download_maps.sh`'s
order, on a small map. The map's light makes the earlier repairs rewrite it, and its projector
declares no scale. The test then asserts exactly one `local_x` and one `local_y` per node, at the
projector's values.

**Hosts that ran `download_maps.sh` since the repairs moved to acb (roadmap 087) need to run it
again** after updating acb's pin. A rerun removes the duplicates. Check with
`grep -c 'k="local_x"' <town>/lanelet2_map.osm` against `grep -c '<node' <town>/lanelet2_map.osm`.
