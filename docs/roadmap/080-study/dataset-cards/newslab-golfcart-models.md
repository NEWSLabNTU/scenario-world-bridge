---
license: apache-2.0
pretty_name: NEWSLab golf cart simulation models
tags: [isaac-sim, usd, autoware, rviz, vehicle-model, lidar, golf-cart]
---

# NEWSLab golf cart: simulation models

Simulation models of NEWSLab's autonomous golf cart (National Taiwan University), from the
[2026-golf-cart](https://github.com/NEWSLabNTU/2026-golf-cart) repository (2026-11-10). All
are generated, and the generators are included.

| Path | Content |
|---|---|
| `golfcart_twin/isaac/body/vehicle.newslab.golfcart/` | the Isaac Sim body (default prim `/Vehicle`, a `headlights` variant set, off and on), its four wheels, and the catalogue entry for scensim |
| `golfcart_twin/isaac/lidars/` | Isaac RTX lidar profiles: Velodyne VLP-32C and Seyond Falcon |
| `golfcart_twin/isaac/newslab_golfcart.rig.yaml`, `ego.yaml`, `spec_hash.yaml` | the twin rig: PhysX vehicle and sensor poses |
| `golfcart_twin/vehicle/newslab_golfcart.yaml` | the vehicle spec the rig and catalogue come from |
| `golfcart_twin/catalogs/vehicles.xosc` | the OpenSCENARIO catalogue entry |
| `golfcart_twin/body/` | the generators: `make_body.py` (usd-core and numpy), `make_vehicle_mesh.py`, previews, the lidar clearance check |
| `rviz/golfcart.dae` | the RViz mesh, Autoware's convention: Y up, facing -X, origin on the ground at mid wheelbase |

The body was estimated from photos of the cart; its mass and tyre data are partly estimates.

Licence Apache-2.0 (`LICENSE`, `NOTICE`). The VLP-32C laser table comes from TIER IV's
[Nebula](https://github.com/tier4/nebula) (Apache-2.0).
