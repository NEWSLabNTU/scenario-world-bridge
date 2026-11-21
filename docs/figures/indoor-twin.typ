// The steps of an indoor twin: the field, the car, and joining them (docs/how-to/build-an-indoor-twin.md).
//   typst compile docs/figures/indoor-twin.typ docs/figures/indoor-twin.svg
#import "style.typ": *
#show: setup

#let you = rgb("#dfe4f7")    // steps you do with your own equipment and Isaac's editor
#let chk = rgb("#fbe7c0")    // checks
#let rel = rgb("#e3efd6")    // the release and its upkeep

#let b(pos, body, fill, name) = node(pos, body, fill: fill, name: name, width: 50mm,
  corner-radius: 3pt, stroke: 0.6pt + fill.darken(45%), inset: 5pt)

#diagram(
  spacing: (10mm, 7mm),
  edge-stroke: 0.7pt,
  label-size: 6.5pt,

  b((0, 0), [*F1 Survey* \ scan, control points, flatness], you, <f1>),
  b((0, 1), [*F2 Stage* \ visual, physics, evidence], you, <f2>),
  b((0, 2), [*F3 Materials* \ tilt, drop and push tests], you, <f3>),
  b((0, 3), [*F4 Maps* \ occupancy map, lanelet loop], you, <f4>),
  b((0, 4), [*F5 Check* \ map check, lidar against scans], chk, <f5>),
  edge(<f1>, <f2>, "-|>"), edge(<f2>, <f3>, "-|>"), edge(<f3>, <f4>, "-|>"),
  edge(<f4>, <f5>, "-|>"),
  title((0, -0.6), [THE FIELD]),
  grp((0, -0.6), <f1>, <f5>),

  b((2, 0), [*V1 Spec* \ geometry, mass, tyres], you, <v1>),
  b((2, 1), [*V2 Log the real car* \ M1 to M6, five times each], you, <v2>),
  b((2, 2), [*V3 Component* \ fitted from the logs], chk, <v3>),
  b((2, 3), [*V4 Car physics* \ tyres, suspension, rate], chk, <v4>),
  edge(<v1>, <v2>, "-|>"), edge(<v2>, <v3>, "-|>"), edge(<v3>, <v4>, "-|>"),
  title((2, -0.6), [THE CAR]),
  grp((2, -0.6), <v1>, <v4>),

  edge(<f3>, <v4>, "--", [friction two ways], label-side: center),

  b((1, 5.5), [*J1 Your stack* \ M7 laps against the real laps], chk, <j1>),
  b((1, 6.9), [*J2 Release* \ spec, component, stage hashes], rel, <j2>),
  b((1, 8.3), [*J3 Upkeep* \ floor, tyres or battery changed], rel, <j3>),
  edge(<f5>, <j1>, "-|>"), edge(<v4>, <j1>, "-|>"),
  edge(<j1>, <j2>, "-|>"), edge(<j2>, <j3>, "-|>"),
  title((1, 4.9), [JOINING THEM]),
  grp((1, 4.9), <j1>, <j3>),
)
