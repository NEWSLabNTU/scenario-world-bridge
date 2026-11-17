---
license: cc-by-4.0
pretty_name: CARLA 0.9.16 towns and actors in USD for Isaac Sim
tags: [carla, isaac-sim, usd, autonomous-driving, simulation, scensim]
---

# CARLA 0.9.16 towns and actors in USD, for Isaac Sim

CARLA 0.9.16's content, exported from its Unreal Engine 4.26 editor with NVIDIA's SimReady
converter. It is wrapped for the Isaac Sim world server of
[scensim](https://github.com/NEWSLabNTU/scensim), which runs
OpenSCENARIO scenarios (tier4/scenario_simulator_v2) on CARLA or Isaac Sim with Autoware.

| Folder | Content |
|---|---|
| `Town01`, `Town02`, `Town03`, `Town05`, `Town10HD`, `Town07_Opt` | each town's USD stage, its props and materials, and `<Town>_physics.usda`, a physics layer giving each road mesh its surface |
| `vehicles/` | 41 vehicles (`vehicles.json`), 97 props (`props/props.json`), 52 walkers (`walkers/walkers.json`), the rider pose |
| `MANIFEST.sha256` | checksums of the 2026-10-23 export (Town07_Opt, exported 2026-10-18, is not in it) |

## Use with scensim

```bash
hf download jerry73204/carla-0.9.16-isaac-usd \
    --repo-type dataset \
    --local-dir ~/carla-usd
export CARLA_USD=~/carla-usd
export SCENSIM_VEHICLE_USD=~/carla-usd/vehicles
```

Then follow the repository's "Getting started on Isaac Sim". How the set is made and laid out:
`docs/design/isaac-assets.md` there.

## Licence

Full text: `NOTICE.md`.


CARLA's assets are licensed CC-BY 4.0 by the CARLA team (carla-simulator/carla). This export
keeps that licence; credit CARLA when you use it. Car makes, logos and badges on the models
are trademarks of their owners, not covered by the licence.
