---
license: apache-2.0
pretty_name: Isaac Sim road-physics test stages
tags: [isaac-sim, usd, vehicle-dynamics, tire, terramechanics, scensim]
---

# Isaac Sim road-physics test stages

The stages [scensim](https://github.com/NEWSLabNTU/scensim)'s
road-physics tests run on in Isaac Sim. Self-contained USD, no external references.

| File | Content |
|---|---|
| `materials_ground.usda`, `materials_surfaces.yaml` | flat ground with 13 surface strips (dry and wet asphalt, gravel, dirt, sand, packed snow, ice, ...) and each surface's friction and soil parameters; the tests drive a start, a brake and a 25 deg steering step on each |
| `proving_ground.usda` | the vehicle model check's proving ground |

`procedural/` holds the stages that are built at run time rather than saved: the stage YAMLs
and the spike scripts that build them (surfaces, soft ground, sliding, drift and small-scale
roughness tests), each with its last commit in scensim_isaac_world, Town01's worn-rut grids, and
its README saying how to rebuild each.

The CARLA towns' physics layers (`<Town>_physics.usda`, the surface of each road mesh) are in
[carla-0.9.16-isaac-usd](https://huggingface.co/datasets/jerry73204/carla-0.9.16-isaac-usd).

Made by NEWSLab, National Taiwan University, for scenario-world-bridge, now scensim (2026-10-29).
Licence Apache-2.0 (`LICENSE`, `NOTICE`).
