# Your car

Drive a scenario with another car than the default Tesla Model 3: a car that ships, or one
of your own from its vehicle spec. One spec describes the car; the simulators' physics and
sensors, SSv2's catalog entry and Autoware's vehicle packages are all made from it, and a
run whose parts came from different specs is refused
([vehicle side](../design/vehicle-side.md)).

## A car that ships

Two cars ship: `tesla_model3` (the default) and `drift_mustang`. The car of an entity is
named twice, and the two must agree (`project check` says so when they do not):

- in the scenario, the entity's catalog entry: `<CatalogReference catalogName="vehicles"
  entryName="drift_mustang">`;
- in `swb.yaml`, the pilot's car: `ego: {kind: autoware, vehicle: drift_mustang}`.

On CARLA, the car's physics come from its spec only when the world server is given its
`carla_physics.yaml`; without it, CARLA's blueprint keeps its own mass, engine and tyres.
Pass it through the stack, beside the state directory the stack sets itself (both, or the
signal tables lose their place):

```bash
G=$(ros2 pkg prefix --share sim_vehicle_spec)/generated
play_launch launch --web-addr 127.0.0.1:8080 \
    swb_launch swb.launch.xml \
    stack_args:="sim_args:='state_dir:=$PWD/state \
vehicle_physics:=$G/drift_mustang/carla_physics.yaml'"
```

The pilot's vehicle bridge then reads the physics back from CARLA and checks them against
the spec. (A project key for this is planned; `stack_args` is the way until then, and has
not been run with play_launch's own argument parser on a live stack.)

## A car of your own

Start from a spec, `vehicles/<car>.yaml` in your project, in `sim_vehicle_spec`'s format
(its `docs/spec-format.md`). `sim_vehicle_spec.gen` writes what the bridge and CARLA need:

```bash
python3 -m sim_vehicle_spec.gen \
    --spec vehicles/my_car.yaml \
    --catalog catalogs/vehicles.xosc \
    --spec-hash config/spec_hash.yaml \
    --carla carla
```

| Output | Used by |
|---|---|
| `catalogs/vehicles.xosc` | the scenario's `CatalogLocations`, and the bridge (`vehicle_catalogs`) |
| `config/spec_hash.yaml` | the vehicle agent (`spec_hash_file`) |
| `carla/carla_physics.yaml` | CARLA's world server (`vehicle_physics`) |
| `carla/carla_sensors.yaml` | the CARLA vehicle bridge's sensors |

Autoware also needs the car's own packages, `<car>_description` (with its
`config/spec_hash.yaml` and pedal maps) and `<car>_sensor_kit_*`, built in a workspace you
source in `~/swb.bash` after this one. They are made from the spec in
`autoware_sim_vehicle`, and the pedal maps are measured, not generated
([a vehicle from another repository](../design/vehicle-side.md#a-vehicle-from-another-repository)).
The pilot then names the car, and Autoware loads its packages by that name:

```yaml
vehicles: [vehicles/my_car.yaml]
pilots:
  ego: {kind: autoware, vehicle: my_car}
```

`vehicles:` is checked by `project check` (the file must exist) but not yet loaded by the
stack (2026-11-15): give the bridge your catalog, and the world server your physics,
through `stack_args`:

```bash
play_launch launch --web-addr 127.0.0.1:8080 \
    swb_launch swb.launch.xml \
    stack_args:="vehicle_catalogs:=$PWD/catalogs/vehicles.xosc \
sim_args:='state_dir:=$PWD/state vehicle_physics:=$PWD/carla/carla_physics.yaml'"
```

On Isaac the car is an aib ego config (`rig:`, `body:`) inside the world server, and
`aib.gen` writes the rig from the spec (`python3 -m aib.gen --spec vehicles/my_car.yaml -o
isaac/my_car.rig.yaml`); the world server takes it as `ego:=<config>` in its `sim_args`.

## The spec check

Before the ego starts, the bridge compares the `spec_hash` of each part: the world
server's body (on CARLA only when it applied a `carla_physics.yaml`), the vehicle agent's,
and the catalog entry of the ego's body. When two differ the spawn fails:

```
Spawn 'ego': the ego's parts were built from different vehicle specs (the world
server (...) 480be4fd0d00, the vehicle agent 65f1ba86a28f): rebuild and restart the
stale one
```

Make all three from one spec; after changing a spec, run the generators again, rebuild,
and restart the stack. Two related refusals:

- `the ego's body on ... is not the car SSv2 declared (the vehicle catalog's), more than
  0.02 m off`: the scenario's catalog entry and the simulator's body are different cars.
- `vehicle catalogs disagree on <body>`: your catalog gives a body that the installed
  catalog already uses (`vehicle.tesla.model3`, `vehicle.ford.mustang`) another spec. Give
  your car a body of its own.
