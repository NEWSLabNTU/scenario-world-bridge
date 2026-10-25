// The ego's control path on CARLA and on Isaac Sim, with the car's onboard computers in a
// repository of their own (sim_vehicle_ecu). Fill = repository.
//   typst compile docs/design/vehicle-ecu.typ docs/design/vehicle-ecu.svg
#import "@preview/fletcher:0.5.8" as fletcher: diagram, node, edge

#set page(width: auto, height: auto, margin: 8mm, fill: white)
#set text(font: "DejaVu Sans", size: 8pt)

#let aw = rgb("#e6e6e6")     // Autoware
#let swb = rgb("#d9d2e9")    // scenario-world-bridge and the world servers
#let acb = rgb("#fce5cd")    // autoware_carla_bridge
#let aib = rgb("#cfe2f3")    // autoware_isaac_bridge
#let ecu = rgb("#d9ead3")    // sim_vehicle_ecu (new)
#let spec = rgb("#fff2cc")   // sim_vehicle_spec, autoware_sim_vehicle
#let sim = white             // the simulators' own code
#let grp = (paint: gray.darken(20%), dash: "dashed", thickness: 0.8pt)
#let b(pos, body, fill, name, width: 46mm) = node(pos, body, fill: fill, name: name,
  width: width, corner-radius: 3pt, stroke: 0.6pt + fill.darken(45%))
#let title(pos, body) = node(pos, place(left + horizon, dx: -2mm, dy: -1mm,
  box(width: 80mm, text(size: 9pt, weight: "bold", body))), stroke: none, width: 46mm,
  height: 4mm)
#let swatch(fill, body) = box(baseline: 20%, rect(width: 3mm, height: 3mm, fill: fill,
  stroke: 0.5pt + fill.darken(45%))) + h(1mm) + body

#diagram(
  spacing: (14mm, 13mm),
  edge-stroke: 0.7pt,
  label-size: 7pt,

  b((2, 0), [*Autoware* (autoware_sim_profile, the same on both) \ planning · trajectory
    follower \ out: acceleration, tire angle, gear \ in: sensors, /vehicle/status/\*], aw,
    <aw>, width: 64mm),

  // ── CARLA ────────────────────────────────────────────────────────────────────
  title((0, 0.95), [CARLA]),
  b((0, 1.5), [*acb_bridge* (own ROS node, Rust) \ vehicle interface \ acceleration →
    throttle and brake \ (pedal maps), steering lag], acb, <acb>),
  b((0, 2.7), [*CARLA 0.9.16 server* (UE4) \ its own vehicle model: engine, \ gearbox, brakes,
    tires \ no ABS, TC or ESC], sim, <cs>),
  b((0, 3.9), [*swb_carla_world* \ spawns the ego, \ applies carla_physics.yaml], swb,
    <cw>),
  node(enclose: ((0, 0.95), <acb>, <cs>, <cw>), stroke: grp, inset: 6pt, name: <carla>),

  // ── Isaac Sim ────────────────────────────────────────────────────────────────
  title((1, 0.95), [Isaac Sim 6.1 process]),
  b((2, 1.5), [*aib* vehicle interface (EgoRos) \ control_cmd → by-wire request \
    status and sensors (lidar, \ camera, IMU, GNSS)], aib, <aib>),
  b((3, 1.5), [*swb_isaac_world* (or aib.runner) \ lockstep 0.05 s; each 1/60 s \ substep:
    sensors → ECU step \ → torques → PhysX step], swb, <loop>),
  b((1, 2.7), [*VCU* \ request → motor torque \ regen blended with the \ friction brake,
    motor lag], ecu, <vcu>),
  b((2, 2.7), [*Brake module* \ distribution (EBD), hydraulics \ ABS · TC · *ESC* \ grip
    and sideslip estimated], ecu, <brk>),
  b((3, 2.7), [*EPS* \ tire angle request → \ steering actuator, \ lag and rate limit],
    ecu, <eps>),
  node(enclose: (<vcu>, <brk>, <eps>), stroke: 1pt + ecu.darken(45%), inset: 5pt,
    name: <ecus>),
  b((2, 3.9), [*PhysX vehicle* (built from the rig) \ drive-none: per-wheel drive \ and
    brake torque, tire angle \ the tire model], sim, <px>),
  b((3, 3.9), [*Surface layer* (swb_isaac_world) \ tire friction per wheel \ soil forces,
    ruts], swb, <surf>),
  node(enclose: ((1, 0.95), <aib>, <loop>, <ecus>, <px>, <surf>), stroke: grp, inset: 6pt,
    name: <isaac>),

  // ── Vehicle layer ────────────────────────────────────────────────────────────
  b((1, 5.1), [*sim_vehicle_spec* \ vehicles/tesla_model3.yaml], spec, <spec>),
  b((3, 5.1), [*autoware_sim_vehicle* \ the car's Autoware packages \ pedal maps, profile],
    spec, <asv>),

  // ── Run time ─────────────────────────────────────────────────────────────────
  edge(<aw>, (0, 0), <acb>, "-|>", [control_cmd], label-side: left, label-pos: 0.3),
  edge(<aw>, <aib>, "-|>", [control_cmd], label-side: left),
  edge(<acb>, <cs>, "-|>", [VehicleControl \ (RPC)], label-side: right),
  edge(<cw>, <cs>, "-|>", [spawn, \ physics], label-side: left),
  edge(<aib>, <ecus>, "-|>", [by-wire request: \ acceleration, tire angle, gear],
    label-side: right),
  edge(<ecus>, <px>, "-|>", [wheel torques, \ tire angle], label-side: left, shift: 9pt),
  edge(<px>, <ecus>, "-|>", [vehicle sensors: \ wheel speeds, yaw rate, \ acceleration,
    steering angle], label-side: left, shift: 9pt),
  edge(<surf>, <px>, "-|>", [friction, \ forces], label-side: right),

  // ── From the spec ────────────────────────────────────────────────────────────
  edge(<spec>, (0, 5.1), <cw>, "-|>", stroke: (dash: "dashed"), [carla_physics.yaml],
    label-side: right, label-pos: 0.25),
  edge(<spec>, <vcu>, "-|>", stroke: (dash: "dashed"), [ECU calibration: \ powertrain,
    brakes (abs, esc), \ traction_control, steering], label-side: right),
  edge(<spec>, (2, 5.1), <px>, "-|>", stroke: (dash: "dashed"), [Isaac rig],
    label-side: left, label-pos: 0.25),
  edge(<asv>, (3.78, 5.1), (3.78, 0), <aw>, "-|>", stroke: (dash: "dashed")),
)

#v(3mm)
#text(size: 7.5pt)[
  #swatch(aw)[Autoware] #h(3mm)
  #swatch(swb)[scenario-world-bridge, world servers] #h(3mm)
  #swatch(acb)[acb] #h(3mm)
  #swatch(aib)[aib] #h(3mm)
  #swatch(ecu)[*sim_vehicle_ecu* (new)] #h(3mm)
  #swatch(spec)[vehicle layer] #h(3mm)
  #swatch(sim)[simulator] \
  Solid: run time. Dashed: generated from the spec. Not drawn: SSv2 and the swb bridge,
  which drive either world server over the world protocol. \
  The ECUs see only the car's own sensors, never the ground's friction. CARLA does not use
  sim_vehicle_ecu: it takes throttle, brake and steering, not per-wheel torque.
]
