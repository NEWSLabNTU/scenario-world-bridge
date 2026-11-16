// How it works: a formal scenario, a simulator, and the bridge between them and the autopilot.
// The scenario is the shipped basic/town01_pedestrian.xosc, abridged; the screenshots are one
// frame of its run on CARLA Town01 (the ego stopped before the walker; RViz: obstacle_stop).
//   typst compile docs/figures/how-it-works.typ docs/figures/how-it-works.svg
#import "style.typ": scen, world, veh, spec
#set page(width: 266mm, height: 128mm, margin: 0mm, fill: white)
#set text(font: "DejaVu Sans", size: 8pt)
#show raw: set text(font: "DejaVu Sans Mono", size: 7pt)

#let ink = gray.darken(55%)
#let mm(x) = x * 1mm
// Absolute placement: everything in mm from the top left of a 4 mm margin.
#let at(x, y, body) = place(top + left, dx: mm(x + 4), dy: mm(y + 4), body)
#let panel(x, y, w, h, fill: white, body) = at(x, y, block(width: mm(w), height: mm(h),
  inset: 7pt, radius: 4pt, fill: fill, stroke: 0.6pt + gray.darken(20%), body))
#let unit(x, y, w, h, head, sub, fill) = at(x, y, block(width: mm(w), height: mm(h),
  inset: 4pt, radius: 4pt, fill: fill, stroke: 0.6pt + fill.darken(45%),
  align(center + horizon)[#text(size: 10pt, weight: "bold", head) \
    #text(size: 7.2pt, fill: gray.darken(40%), sub)]))
#let lbl(x, y, body, w: 30) = at(x, y, block(width: mm(w), text(size: 7pt, fill: ink, body)))
#let caption(x, y, w, body) = at(x, y, block(width: mm(w), align(center,
  text(size: 7.8pt, weight: "bold", fill: gray.darken(45%), body))))
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
#panel(0, 0, 76, 120, fill: scen.lighten(55%))[
  #text(size: 11pt, weight: "bold")[OpenSCENARIO file] \
  #text(size: 7.5pt, fill: gray.darken(40%))[town01_pedestrian.xosc, abridged]
  #v(2pt)
  #set text(font: "DejaVu Sans Mono", size: 7pt)
  #set par(leading: 0.62em)
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
  #k(`<Init>`) #h(6pt)#c(`<!-- ego start and goal -->`) \
  #h(5pt)#k(`<TeleportAction>`) 320.0, -129.8 \
  #h(5pt)#k(`<AcquirePositionAction>`) 135.0, -129.8 \
  #k(`</Init>`) \
  #k(`<Story>`) #h(6pt)#c(`<!-- the pedestrian's part -->`) \
  #h(5pt)#k(`<Event name="cross">`) from 1 s: \
  #h(10pt)reaches the ego's lane at 12 s, \
  #h(10pt)stands in it until 40 s, \
  #h(10pt)crosses on by 52 s \
  #k(`</Story>`) \
  #k(`<Checks>`) \
  #h(5pt)success: the ego within 5 m of its goal \
  #h(5pt)failure: any contact, closer than 1 m \
  #h(53pt)to the walker in its path, \
  #h(53pt)or 300 s \
  #k(`</Checks>`)
]

// The scenario runner stack
#panel(90, 0, 84, 120)[]
#unit(112, 8, 56, 21, [Scenario runner], [scenario_simulator_v2 \ story and verdict], scen)
#unit(112, 50, 56, 21, [World bridge], [scenario_world_bridge \ and the world server], spec)
#unit(112, 90, 56, 21, [Agent relay], [goals to the \ vehicle's autopilot], rgb("#ece3f7"))
#caption(90, 114, 84)[Scenario runner stack]

#arrow((76, 15), (112, 15))
#lbl(79, 6, [actors, \ events])
#arrow((140, 29), (140, 50))
#lbl(142, 34, [spawn, poses, \ signals, step])
#arrow((112, 24), (99, 24), (99, 100), (112, 100))
#lbl(100.5, 60, [goal, \ engage], w: 11)

// The simulator
#panel(188, 0, 74, 48)[
  #text(size: 11pt, weight: "bold")[CARLA or Isaac Sim]
  #v(1pt)
  #image("hiw-carla.jpg", width: 100%)
]
#arrow((168, 60), (180, 60), (180, 22), (188, 22))
#lbl(175.5, 62, [world \ protocol], w: 12)

// The autopilot support package
#panel(188, 62, 74, 58, fill: veh.lighten(40%))[]
#at(192, 65, block(width: mm(66), inset: 4pt, radius: 3pt, fill: world, align(center,
  text(size: 7.3pt, weight: "bold")[Vehicle bridge: acb (CARLA) or aib (Isaac)])))
#at(192, 76, block(width: mm(30), text(size: 8.6pt, weight: "bold")[Autoware \
  #text(size: 7.2pt, weight: "regular", fill: gray.darken(40%))[tested with 1.5.0, \
  or another autopilot]]))
#at(224, 75, image("hiw-rviz.jpg", width: 34mm))
#caption(188, 114, 74)[Autopilot support package]
#arrow((168, 100), (188, 100))
#lbl(175.5, 91, [route, \ engage], w: 12)

#arrow((210, 48), (210, 65))
#lbl(189, 50, [camera, lidar, \ IMU, GNSS], w: 20)
#arrow((226, 65), (226, 48))
#lbl(228, 50, [steering, \ throttle, brake], w: 24)
