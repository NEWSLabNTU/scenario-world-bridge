#!/usr/bin/env python3
"""Write a drive scenario on a generated map (roadmap 062, O6): the ego from a start to a goal,
with every traffic light of the map green from the start.

    scripts/open_maps/scenario.py <lanelet2_map.osm> <out.xosc> --name <n> --map <LogicFile path>
        --start x,y,heading --goal x,y,heading [--timeout 600] [--description "..."]

Positions are in the map frame (m, rad). The lights: SSv2 names a signal by its traffic_light
way's id. Nothing else gives them a state, and Autoware stops for a light it has no state for
("the traffic signal data associated with regulatory element id ... is not received": the first
NTU run stopped at the main gate), so each gets a TrafficSignalStateAction in Init. The ids
change when the map is generated again, hence a script and not a hand-written file.
"""
import argparse
import xml.etree.ElementTree as ET

TEMPLATE = """<?xml version="1.0"?>
<OpenSCENARIO>
	<FileHeader author="scenario-world-bridge" date="2026-10-30T00:00:00+00:00" description="{description}" revMajor="1" revMinor="2" />
	<ParameterDeclarations>
	</ParameterDeclarations>
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
								<WorldPosition x="{sx}" y="{sy}" z="0.3" h="{sh}" p="0" r="0" />
							</Position>
						</TeleportAction>
					</PrivateAction>
					<PrivateAction>
						<RoutingAction>
							<AcquirePositionAction>
								<Position>
									<WorldPosition x="{gx}" y="{gy}" z="0.0" h="{gh}" p="0" r="0" />
								</Position>
							</AcquirePositionAction>
						</RoutingAction>
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
										<ByEntityCondition>
											<TriggeringEntities triggeringEntitiesRule="any">
												<EntityRef entityRef="ego" />
											</TriggeringEntities>
											<EntityCondition>
												<ReachPositionCondition tolerance="5.0">
													<Position>
														<WorldPosition x="{gx}" y="{gy}" z="0.0" h="0" p="0" r="0" />
													</Position>
												</ReachPositionCondition>
											</EntityCondition>
										</ByEntityCondition>
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
					</Maneuver>
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


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map")
    ap.add_argument("out")
    ap.add_argument("--name", required=True)
    ap.add_argument("--map-path", required=True, help="the LogicFile path, e.g. $(env SWB_OPEN_MAPS)/ntu/map")
    ap.add_argument("--start", required=True)
    ap.add_argument("--goal", required=True)
    ap.add_argument("--timeout", type=float, default=600)
    ap.add_argument("--description", default="")
    a = ap.parse_args()
    root = ET.parse(a.map).getroot()
    lights = [w.get("id") for w in root.findall("way")
              if any(t.get("k") == "type" and t.get("v") == "traffic_light" for t in w.findall("tag"))]
    block = "".join(
        f'\t\t\t\t<GlobalAction><InfrastructureAction><TrafficSignalAction><TrafficSignalStateAction '
        f'name="{i}" state="green" /></TrafficSignalAction></InfrastructureAction></GlobalAction>\n'
        for i in lights)
    sx, sy, sh = a.start.split(",")
    gx, gy, gh = a.goal.split(",")
    with open(a.out, "w") as f:
        f.write(TEMPLATE.format(description=a.description, map=a.map_path, lights=block, sx=sx, sy=sy,
                                sh=sh, gx=gx, gy=gy, gh=gh, name=a.name, timeout=a.timeout))
    print(f"{a.out}: {len(lights)} traffic lights green")


if __name__ == "__main__":
    main()
