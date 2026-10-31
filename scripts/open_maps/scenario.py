#!/usr/bin/env python3
"""Write a drive scenario on a generated map (roadmap 062, O6): the ego from a start to a goal,
through waypoints if given, with every traffic light of the map green from the start except the
red ones asked for.

    scripts/open_maps/scenario.py <lanelet2_map.osm> <out.xosc> --name <n> --map-path <LogicFile path>
        --start x,y,heading[,z] [--via x,y,heading[,z] ...] --goal x,y,heading[,z]
        [--red x,y ...] [--timeout 600] [--description "..."]

Positions are in the map frame (m, rad); z defaults to 0.3 (start) and 0.0 (waypoints and goal),
the generated maps' ground, and is needed on a map whose ground is not at 0 (059's NTU twin,
~12 m); the red lights' checks take the goal's. The lights: SSv2 names a signal by its traffic_light
way's id. Nothing else gives them a state, and Autoware stops for a light it has no state for
("the traffic signal data associated with regulatory element id ... is not received": the first
NTU run stopped at the main gate), so each gets a TrafficSignalStateAction in Init. The ids
change when the map is generated again, hence a script and not a hand-written file.

`--red x,y` starts the light whose stop line is nearest the point red. It turns green when the
ego has stood still for 3 s before that stop line, and the scenario fails if the ego reaches
the junction beyond it while it is still red: running the light fails, stopping releases it.
"""
import argparse
import math
import xml.etree.ElementTree as ET

TEMPLATE = """<?xml version="1.0"?>
<OpenSCENARIO>
	<FileHeader author="scenario-world-bridge" date="2026-10-30T00:00:00+00:00" description="{description}" revMajor="1" revMinor="2" />
	<ParameterDeclarations>
{params}	</ParameterDeclarations>
	<CatalogLocations><VehicleCatalog><Directory path="$(find-pkg-share sim_vehicle_spec)/catalogs" /></VehicleCatalog></CatalogLocations>
	<RoadNetwork>
		<LogicFile filepath="{map}" />
	</RoadNetwork>
	<Entities>
		<ScenarioObject name="ego">
			<CatalogReference catalogName="vehicles" entryName="tesla_model3">
				<ParameterAssignments>
					<ParameterAssignment parameterRef="maxSpeed" value="50" />
					<ParameterAssignment parameterRef="maxAcceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxDeceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxSteering" value="0.5" />
				</ParameterAssignments>
			</CatalogReference>
			<ObjectController>
				<Controller name="Autoware">
					<Properties>
						<Property name="maxJerk" value="1.5" />
						<Property name="minJerk" value="-1.5" />
					</Properties>
				</Controller>
			</ObjectController>
		</ScenarioObject>
	</Entities>
	<Storyboard>
		<Init>
			<Actions>
{lights}				<Private entityRef="ego">
					<PrivateAction>
						<TeleportAction>
							<Position>
								<WorldPosition x="{sx}" y="{sy}" z="{sz}" h="{sh}" p="0" r="0" />
							</Position>
						</TeleportAction>
					</PrivateAction>
					<PrivateAction>
						<RoutingAction>
{routing}						</RoutingAction>
					</PrivateAction>
				</Private>
			</Actions>
		</Init>
		<Story name="{name}">
			<Act name="drive_act">
				<ManeuverGroup maximumExecutionCount="1" name="ego_maneuvers">
					<Actors selectTriggeringEntities="false">
						<EntityRef entityRef="ego" />
					</Actors>
					<Maneuver name="end_conditions">
						<Event name="success" priority="parallel">
							<Action name="exit_success">
								<UserDefinedAction>
									<CustomCommandAction type="exitSuccess" />
								</UserDefinedAction>
							</Action>
							<StartTrigger>
								<ConditionGroup>
									<Condition name="reach_goal" delay="0" conditionEdge="none">
										{at_goal}
									</Condition>
								</ConditionGroup>
							</StartTrigger>
						</Event>
						<Event name="timeout" priority="parallel">
							<Action name="exit_failure">
								<UserDefinedAction>
									<CustomCommandAction type="exitFailure" />
								</UserDefinedAction>
							</Action>
							<StartTrigger>
								<ConditionGroup>
									<Condition name="timeout_condition" delay="0" conditionEdge="none">
										<ByValueCondition>
											<SimulationTimeCondition value="{timeout}" rule="greaterThan" />
										</ByValueCondition>
									</Condition>
								</ConditionGroup>
							</StartTrigger>
						</Event>
{red_events}					</Maneuver>
				</ManeuverGroup>
				<StartTrigger>
					<ConditionGroup>
						<Condition name="start" delay="0" conditionEdge="none">
							<ByValueCondition>
								<SimulationTimeCondition value="0" rule="greaterThan" />
							</ByValueCondition>
						</Condition>
					</ConditionGroup>
				</StartTrigger>
			</Act>
		</Story>
		<StopTrigger />
	</Storyboard>
</OpenSCENARIO>
"""
LIGHT = ('\t\t\t\t<GlobalAction><InfrastructureAction><TrafficSignalAction><TrafficSignalStateAction '
         'name="{i}" state="{state}" /></TrafficSignalAction></InfrastructureAction></GlobalAction>\n')
ACQUIRE = ('\t\t\t\t\t\t\t<AcquirePositionAction><Position><WorldPosition x="{x}" y="{y}" z="{z}" '
           'h="{h}" p="0" r="0" /></Position></AcquirePositionAction>\n')
WAYPOINT = ('\t\t\t\t\t\t\t\t\t<Waypoint routeStrategy="shortest"><Position><WorldPosition x="{x}" y="{y}" '
            'z="{z}" h="{h}" p="0" r="0" /></Position></Waypoint>\n')
REACH = ('<ByEntityCondition><TriggeringEntities triggeringEntitiesRule="any"><EntityRef entityRef="ego" />'
         '</TriggeringEntities><EntityCondition><ReachPositionCondition tolerance="{t:.1f}"><Position>'
         '<WorldPosition x="{x}" y="{y}" z="{z}" h="0" p="0" r="0" /></Position></ReachPositionCondition>'
         '</EntityCondition></ByEntityCondition>')
STILL = ('<ByEntityCondition><TriggeringEntities triggeringEntitiesRule="any"><EntityRef entityRef="ego" />'
         '</TriggeringEntities><EntityCondition><StandStillCondition duration="3" /></EntityCondition>'
         '</ByEntityCondition>')
RED = """						<Event name="red_{k}_released" priority="parallel" maximumExecutionCount="1">
							<Action name="green_{k}"><GlobalAction><InfrastructureAction><TrafficSignalAction><TrafficSignalStateAction name="{i}" state="green" /></TrafficSignalAction></InfrastructureAction></GlobalAction></Action>
							<Action name="clear_{k}"><GlobalAction><ParameterAction parameterRef="red_{k}"><SetAction value="0" /></ParameterAction></GlobalAction></Action>
							<StartTrigger><ConditionGroup>
								<Condition name="before_{k}" delay="0" conditionEdge="none">{before}</Condition>
								<Condition name="stopped_{k}" delay="0" conditionEdge="none">{still}</Condition>
							</ConditionGroup></StartTrigger>
						</Event>
						<Event name="red_{k}_run" priority="parallel" maximumExecutionCount="1">
							<Action name="ran_red_{k}"><UserDefinedAction><CustomCommandAction type="exitFailure" /></UserDefinedAction></Action>
							<StartTrigger><ConditionGroup>
								<Condition name="beyond_{k}" delay="0" conditionEdge="none">{beyond}</Condition>
								<Condition name="still_red_{k}" delay="0" conditionEdge="none"><ByValueCondition><ParameterCondition parameterRef="red_{k}" value="1" rule="equalTo" /></ByValueCondition></Condition>
							</ConditionGroup></StartTrigger>
						</Event>
"""


def stop_lines(root):
    """Each traffic_light way's stop line: its two ends, left and right across travel."""
    xy = {}
    for n in root.findall("node"):
        t = {x.get("k"): x.get("v") for x in n.findall("tag")}
        if "local_x" in t:
            xy[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.findall("way")}
    out = {}
    for r in root.findall("relation"):
        if not any(t.get("k") == "subtype" and t.get("v") == "traffic_light" for t in r.findall("tag")):
            continue
        refers = [m.get("ref") for m in r.findall("member") if m.get("role") == "refers"]
        line = [m.get("ref") for m in r.findall("member") if m.get("role") == "ref_line"]
        if refers and line:
            ids = ways[line[0]]
            out[refers[0]] = (xy[ids[0]], xy[ids[-1]])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map")
    ap.add_argument("out")
    ap.add_argument("--name", required=True)
    ap.add_argument("--map-path", required=True, help="the LogicFile path, e.g. $(env SWB_OPEN_MAPS)/ntu/map")
    ap.add_argument("--start", required=True)
    ap.add_argument("--via", action="append", default=[], help="x,y,heading; repeated, in order")
    ap.add_argument("--goal", required=True)
    ap.add_argument("--red", action="append", default=[], help="x,y near a stop line: that light starts red")
    ap.add_argument("--timeout", type=float, default=600)
    ap.add_argument("--description", default="")
    a = ap.parse_args()
    root = ET.parse(a.map).getroot()
    lights = [w.get("id") for w in root.findall("way")
              if any(t.get("k") == "type" and t.get("v") == "traffic_light" for t in w.findall("tag"))]
    stops = stop_lines(root)
    sx, sy, sh, sz = (a.start.split(",") + ["0.3"])[:4]
    gx, gy, gh, gz = (a.goal.split(",") + ["0.0"])[:4]
    red, params, events = [], "", ""
    for k, q in enumerate(a.red):
        x, y = map(float, q.split(","))
        i = min(stops, key=lambda s: math.dist((x, y), [(u + v) / 2 for u, v in zip(*stops[s])]))
        (lx, ly), (rx, ry) = stops[i]
        mx, my, half = (lx + rx) / 2, (ly + ry) / 2, math.dist((lx, ly), (rx, ry)) / 2
        fx, fy = -(ry - ly) / (2 * half), (rx - lx) / (2 * half)  # travel: left-to-right turned a quarter left
        red.append(i)
        params += f'\t\t<ParameterDeclaration name="red_{k}" parameterType="integer" value="1" />\n'
        events += RED.format(
            k=k, i=i, still=STILL,
            before=REACH.format(t=half + 6.0, x=f"{mx - 8 * fx:.3f}", y=f"{my - 8 * fy:.3f}", z=gz),
            beyond=REACH.format(t=half + 1.0, x=f"{mx + 6 * fx:.3f}", y=f"{my + 6 * fy:.3f}", z=gz))
        print(f"red {k}: light {i}, stop line at ({mx:.1f}, {my:.1f}), {2 * half:.1f} m across")
    block = "".join(LIGHT.format(i=i, state="red" if i in red else "green") for i in lights)
    if a.via:
        routing = ('\t\t\t\t\t\t\t<AssignRouteAction>\n\t\t\t\t\t\t\t\t<Route name="route" closed="false">\n'
                   + "".join(WAYPOINT.format(x=x, y=y, h=h, z=z) for x, y, h, z in
                             [(v.split(",") + ["0.0"])[:4] for v in a.via] + [[gx, gy, gh, gz]])
                   + '\t\t\t\t\t\t\t\t</Route>\n\t\t\t\t\t\t\t</AssignRouteAction>\n')
    else:
        routing = ACQUIRE.format(x=gx, y=gy, h=gh, z=gz)
    with open(a.out, "w") as f:
        f.write(TEMPLATE.format(description=a.description, map=a.map_path, params=params, lights=block,
                                sx=sx, sy=sy, sh=sh, sz=sz, routing=routing, name=a.name, timeout=a.timeout,
                                at_goal=REACH.format(t=5.0, x=gx, y=gy, z=gz), red_events=events))
    print(f"{a.out}: {len(lights)} traffic lights, {len(red)} of them red at the start")


if __name__ == "__main__":
    main()
