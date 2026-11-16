// How it works: a formal scenario, a simulator, and the bridge between them and the autopilot.
// The scenario is the shipped basic/town01_pedestrian.xosc, abridged; the screenshots are one
// frame of its run on CARLA Town01 (the ego stopped before the walker; RViz: obstacle_stop).
//   typst compile docs/figures/how-it-works.typ docs/figures/how-it-works.svg
#import "style.typ": scen, world, veh, spec
#set page(width: 308mm, height: 148mm, margin: 0mm, fill: white)
#set text(font: "DejaVu Sans", size: 10pt)
#show raw: set text(font: "DejaVu Sans Mono", size: 8.6pt)

#let ink = gray.darken(55%)
#let mm(x) = x * 1mm
// Absolute placement: everything in mm from the top left of a 4 mm margin.
#let at(x, y, body) = place(top + left, dx: mm(x + 4), dy: mm(y + 4), body)
#let panel(x, y, w, h, fill: white, body) = at(x, y, block(width: mm(w), height: mm(h),
  inset: 7pt, radius: 4pt, fill: fill, stroke: 0.6pt + gray.darken(20%), body))
#let unit(x, y, w, h, head, sub, fill) = at(x, y, block(width: mm(w), height: mm(h),
  inset: 4pt, radius: 4pt, fill: fill, stroke: 0.6pt + fill.darken(45%),
  align(center + horizon)[#text(size: 12.5pt, weight: "bold", head) \
    #text(size: 9pt, fill: gray.darken(40%), sub)]))
#let lbl(x, y, body, w: 30) = at(x, y, block(width: mm(w), text(size: 8.75pt, fill: ink, body)))
#let caption(x, y, w, body) = at(x, y, block(width: mm(w), align(center,
  text(size: 9.6pt, weight: "bold", fill: gray.darken(45%), body))))
// An orthogonal arrow through the given points (mm); the head at the last one.
#let arrow(..pts) = {
  let p = pts.pos().map(((x, y)) => (mm(x + 4), mm(y + 4)))
  place(top + left, curve(stroke: 0.8pt + ink, curve.move(p.at(0)),
    ..p.slice(1).map(q => curve.line(q))))
  let (a, b) = (p.at(-2), p.at(-1))
  let (dx, dy) = (b.at(0) - a.at(0), b.at(1) - a.at(1))
  let s = 1.6mm
  let head = if dx > 0mm { ((b.at(0) - s, b.at(1) - s / 2), (b.at(0) - s, b.at(1) + s / 2)) }
    else if dx < 0mm { ((b.at(0) + s, b.at(1) - s / 2), (b.at(0) + s, b.at(1) + s / 2)) }
    else if dy > 0mm { ((b.at(0) - s / 2, b.at(1) - s), (b.at(0) + s / 2, b.at(1) - s)) }
    else { ((b.at(0) - s / 2, b.at(1) + s), (b.at(0) + s / 2, b.at(1) + s)) }
  place(top + left, polygon(fill: ink, stroke: none, b, ..head))
}
#let k(body) = text(fill: rgb("#2b5fb0"), body)
#let c(body) = text(fill: gray, body)

// The scenario
#panel(0, 0, 86, 140, fill: scen.lighten(55%))[
  #text(size: 14pt, weight: "bold")[OpenSCENARIO file] \
  #text(size: 9.4pt, fill: gray.darken(40%))[town01_pedestrian.xosc, abridged]
  #v(1pt)
  #set text(font: "DejaVu Sans Mono", size: 8.6pt)
  #set par(leading: 0.5em)
  #k(`<Entities>`) \
  #h(5pt)#k(`<ScenarioObject name="ego">`) \
  #h(10pt)#k(`<CatalogReference`) \
  #h(26pt)#k(`entryName="tesla_model3"/>`) \
  #h(10pt)#k(`<Controller name="Autoware"/>`) \
  #h(5pt)#k(`</ScenarioObject>`) \
  #h(5pt)#k(`<ScenarioObject name="pedestrian_0">`) \
  #h(10pt)#k(`<Pedestrian`) \
  #h(26pt)#k(`name="walker.pedestrian.0001"/>`) \
  #h(5pt)#k(`</ScenarioObject>`) \
  #k(`</Entities>`) \
  #k(`<Init>`) #h(4pt)#c(`<!-- start and goal -->`) \
  #h(5pt)#k(`<TeleportAction>`) 320.0, -129.8 \
  #h(5pt)#k(`<AcquirePositionAction>`) \
  #h(26pt)135.0, -129.8 \
  #k(`</Init>`) \
  #k(`<Story>`) #h(4pt)#c(`<!-- the walker -->`) \
  #h(5pt)#k(`<Event name="cross">`) from 1 s: \
  #h(10pt)in the ego's lane at 12 s, \
  #h(10pt)stands in it until 40 s, \
  #h(10pt)across by 52 s \
  #k(`</Story>`) \
  #k(`<Checks>`) \
  #h(5pt)success: the ego within 5 m \
  #h(63pt)of its goal \
  #h(5pt)failure: any contact, closer \
  #h(63pt)than 1 m to the walker \
  #h(63pt)in its path, or 300 s \
  #k(`</Checks>`)
]

// The scenario runner stack
#panel(102, 0, 92, 140)[]
#unit(124, 8, 62, 25, [Scenario runner], [scenario_simulator_v2 \ story and verdict], scen)
#unit(124, 56, 62, 25, [World bridge], [scenario_world_bridge \ and the world server], spec)
#unit(124, 102, 62, 25, [Agent relay], [goals to the \ vehicle's autopilot], rgb("#ece3f7"))
#caption(102, 133, 92)[Scenario runner stack]

#arrow((86, 20), (124, 20))
#lbl(89, 8, [actors, \ events])
#arrow((155, 33), (155, 56))
#lbl(157, 38, [spawn, poses, \ signals, step], w: 34)
#arrow((124, 28), (110, 28), (110, 114), (124, 114))
#lbl(111.5, 66, [goal, \ engage], w: 13)

// The simulator
#panel(212, 0, 88, 58)[
  #text(size: 14pt, weight: "bold")[CARLA or Isaac Sim]
  #v(1pt)
  #image("hiw-carla.jpg", width: 100%)
]
#arrow((186, 68), (204, 68), (204, 26), (212, 26))
#lbl(195.5, 70, [world \ protocol], w: 16)

// The autopilot support package
#panel(212, 76, 88, 64, fill: veh.lighten(40%))[]
#at(216, 79, block(width: mm(80), inset: 4pt, radius: 3pt, fill: world, align(center,
  text(size: 9.4pt, weight: "bold")[Vehicle bridge \
    #text(weight: "regular")[acb (CARLA) or aib (Isaac)]])))
#at(216, 97, block(width: mm(38), text(size: 11pt, weight: "bold")[Autoware \
  #text(size: 9pt, weight: "regular", fill: gray.darken(40%))[tested with 1.5.0, \
  or another \ autopilot]]))
#at(258, 95, image("hiw-rviz.jpg", width: 36mm))
#caption(212, 133, 88)[Autopilot support package]
#arrow((186, 114), (212, 114))
#lbl(195.5, 103, [route, \ engage], w: 16)

#arrow((244, 58), (244, 79))
#lbl(214, 61, [camera, lidar, \ IMU, GNSS], w: 29)
#arrow((260, 79), (260, 58))
#lbl(262, 61, [steering, \ throttle, brake], w: 36)
