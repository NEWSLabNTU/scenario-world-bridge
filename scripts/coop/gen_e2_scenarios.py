#!/usr/bin/env python3
"""Roadmap 074 E2: write the two E2 scenarios (scenarios/coop/town03_e2_*.xosc).

Town03's east straight (CARLA road 65; lanelets 69738 and 69252, map frame): three lanes heading
+y, x = 246.0, 242.5 and 239.0 at y = 16.7 (lanelets 69738, 69252, 68766; outer | inner is a solid line,
inner | innermost dashed), drifting +0.025 m per m of y, flat, straight to y ~ 136.
(Town05 was tried first: its multi-lane straights are the ring, which ramps to 10 m and is one
831 m lanelet per lane; one NPC on it made SSv2's frame take 2.2 s.)
"""
import math
from pathlib import Path

OUT = Path(__file__).resolve().parents[2] / "scenarios/coop"
LANE = {"outer": 246.0, "inner": 242.5, "innermost": 239.0}
H = math.atan2(1.0, 0.025)


def xy(lane, y):
    return (round(LANE[lane] + 0.025 * (y - 16.7), 2), y)


EGO = '''		<ScenarioObject name="ego">
			<CatalogReference catalogName="vehicles" entryName="tesla_model3">
				<ParameterAssignments>
					<ParameterAssignment parameterRef="maxSpeed" value="{vmax}" />
					<ParameterAssignment parameterRef="maxAcceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxDeceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxSteering" value="0.5" />
				</ParameterAssignments>
			</CatalogReference>
			<ObjectController><Controller name="Autoware"><Properties>
				<Property name="maxJerk" value="1.5" /><Property name="minJerk" value="-1.5" />
			</Properties></Controller></ObjectController>
		</ScenarioObject>'''
LEAD = '''		<ScenarioObject name="lead">
			<Vehicle name="lead" vehicleCategory="car">
				<BoundingBox><Center x="1.5" y="0.0" z="0.9" /><Dimensions width="2.1" height="1.8" length="4.5" /></BoundingBox>
				<Performance maxSpeed="20" maxAcceleration="15.0" maxDeceleration="15.0" />
				<Axles>
					<FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.8" positionX="3.1" positionZ="0.3" />
					<RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8" positionX="0.0" positionZ="0.3" />
				</Axles>
				<Properties />
			</Vehicle>
			<ObjectController><Controller name=""><Properties></Properties></Controller></ObjectController>
		</ScenarioObject>'''


def pos(p):
    h = p[2] if len(p) > 2 else H
    return f'<WorldPosition x="{p[0]}" y="{p[1]}" z="0.0" h="{h:.4f}" p="0" r="0" />'


def reach(ent, p, tol=3.0):
    return (f'<ByEntityCondition><TriggeringEntities triggeringEntitiesRule="any"><EntityRef entityRef="{ent}" />'
            f'</TriggeringEntities><EntityCondition><ReachPositionCondition tolerance="{tol}"><Position>{pos(p)}'
            '</Position></ReachPositionCondition></EntityCondition></ByEntityCondition>')


def speed(v, rate):
    return ('<PrivateAction><LongitudinalAction><SpeedAction><SpeedActionDynamics dynamicsShape="linear" '
            f'value="{rate}" dynamicsDimension="rate" /><SpeedActionTarget><AbsoluteTargetSpeed value="{v}" />'
            '</SpeedActionTarget></SpeedAction></LongitudinalAction></PrivateAction>')


def ev(name, action, cond):
    return (f'					<Event name="{name}" priority="parallel" maximumExecutionCount="1">\n'
            f'						<Action name="{name}_a">{action}</Action>\n'
            f'						<StartTrigger><ConditionGroup><Condition name="{name}_c" delay="0" '
            f'conditionEdge="none">{cond}</Condition></ConditionGroup></StartTrigger>\n					</Event>')


def doc(name, desc, ego, goal, vmax, lead=None):
    ents = EGO.format(vmax=vmax) + ("\n" + LEAD if lead else "")
    init = (f'			<Private entityRef="ego">\n				<PrivateAction><TeleportAction><Position>{pos(ego)}'
            f'</Position></TeleportAction></PrivateAction>\n				<PrivateAction><RoutingAction>'
            f'<AcquirePositionAction><Position>{pos(goal)}</Position></AcquirePositionAction></RoutingAction>'
            '</PrivateAction>\n			</Private>')
    if lead:
        init += (f'\n			<Private entityRef="lead">\n				<PrivateAction><TeleportAction><Position>'
                 f'{pos(lead["start"])}</Position></TeleportAction></PrivateAction>\n				<PrivateAction>'
                 f'<RoutingAction><AcquirePositionAction><Position>{pos(lead["end"])}</Position>'
                 '</AcquirePositionAction></RoutingAction></PrivateAction>\n				<PrivateAction><LongitudinalAction>'
                 '<SpeedAction><SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time" />'
                 '<SpeedActionTarget><AbsoluteTargetSpeed value="0.0" /></SpeedActionTarget></SpeedAction>'
                 '</LongitudinalAction></PrivateAction>\n			</Private>')
    evs = [ev("goal", '<UserDefinedAction><CustomCommandAction type="exitSuccess" /></UserDefinedAction>', reach("ego", goal)),
           ev("timeout", '<UserDefinedAction><CustomCommandAction type="exitFailure" /></UserDefinedAction>',
              '<ByValueCondition><SimulationTimeCondition value="150" rule="greaterThan" /></ByValueCondition>')]
    groups = ('				<ManeuverGroup maximumExecutionCount="1" name="ego_group"><Actors selectTriggeringEntities="false">'
              '<EntityRef entityRef="ego" /></Actors>\n				<Maneuver name="ego_m">\n' + "\n".join(evs)
              + '\n				</Maneuver></ManeuverGroup>')
    if lead:
        moving = ('<ByEntityCondition><TriggeringEntities triggeringEntitiesRule="any"><EntityRef entityRef="ego" />'
                  '</TriggeringEntities><EntityCondition><SpeedCondition value="1.0" rule="greaterThan" />'
                  '</EntityCondition></ByEntityCondition>')
        done = ('<ByValueCondition><StoryboardElementStateCondition storyboardElementType="event" '
                'storyboardElementRef="lead_brake" state="completeState" /></ByValueCondition>')
        lev = [ev("lead_go", speed(lead["v"], 2.0), moving),
               ev("lead_brake", speed(0.0, lead["decel"]) + '</Action><Action name="lead_brake_log"><UserDefinedAction>'
                  '<CustomCommandAction type="test(e2, lead_brake)" /></UserDefinedAction>', reach("lead", lead["brake_at"], 2.0)),
               ev("lead_resume", speed(lead["v"], 2.0), done)]
        groups += ('\n				<ManeuverGroup maximumExecutionCount="1" name="lead_group"><Actors selectTriggeringEntities="false">'
                   '<EntityRef entityRef="lead" /></Actors>\n				<Maneuver name="lead_m">\n' + "\n".join(lev)
                   + '\n				</Maneuver></ManeuverGroup>')
    return f'''<?xml version="1.0"?>
<!-- Roadmap 074 E2: {desc}. Written by scripts/coop/gen_e2_scenarios.py; edit that, not this. -->
<OpenSCENARIO>
	<FileHeader author="scenario-world-bridge" date="2026-11-13T00:00:00+00:00" description="074 E2 {name}: {desc}" revMajor="1" revMinor="2" />
	<ParameterDeclarations />
	<CatalogLocations><VehicleCatalog><Directory path="$(find-pkg-share sim_vehicle_spec)/catalogs" /></VehicleCatalog></CatalogLocations>
	<RoadNetwork><LogicFile filepath="$(env CARLA_MAPS)/Town03" /></RoadNetwork>
	<Entities>
{ents}
	</Entities>
	<Storyboard>
		<Init><Actions>
{init}
		</Actions></Init>
		<Story name="e2"><Act name="e2_act">
{groups}
			<StartTrigger><ConditionGroup><Condition name="start" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
		</Act></Story>
		<StopTrigger />
	</Storyboard>
</OpenSCENARIO>
'''


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "town03_e2_lead_brake.xosc").write_text(doc(
        "lead_brake", "Town03 east straight, outer lane: a lead car 28 m ahead at 6 m/s brakes to a stop at 4 m/s^2 "
        "at y = 90, then drives on; perception latency, tracking error, cruise reaction",
        xy("outer", 22.0), xy("outer", 128.0), 9.0,
        lead={"start": xy("outer", 50.0), "end": xy("outer", 135.0), "v": 6.0, "decel": 4.0, "brake_at": xy("outer", 90.0)}))
    (OUT / "town03_e2_lane_change.xosc").write_text(doc(
        "lane_change", "Town03 east straight: start in the innermost lane, goal in the inner lane 106 m on, across "
        "a dashed line; the route forces a lane change; no other traffic", xy("innermost", 22.0), xy("inner", 128.0), 9.0))
    # The route-forced change from a standstill stalled on Town03 (E2): here the change is
    # needed only 220 m on, past the curve (lanelet 68766 at arc length 220 m), so Autoware is
    # already moving when its lane-change module acts.
    (OUT / "town03_e2_lane_change_long.xosc").write_text(doc(
        "lane_change_long", "Town03 east straight and curve: start in the inner lane, goal in the innermost lane "
        "220 m on, past the curve; the lane change happens on the move; no other traffic",
        xy("inner", 22.0), (183.72, 199.55, 3.1261), 9.0))
    # Both scenarios above change into the "innermost" lane, which is CARLA's Bidirectional lane 1
    # of road 65: the tunnel's divider, with walls at x 240.2 from y 62 on. Autoware's map has it
    # as a driving lanelet (68766), so the lane change is legal, and AEB stops it at the walls
    # (E2, 2026-11-13). The inner | outer line is solid. Town03's west straight has two real
    # driving lanes with a dashed line for 244 m, no signals (lanelets 5750 | 5518, then a
    # junction, then 8395 | 8209), heading -y.
    (OUT / "town03_e2_lane_change_west.xosc").write_text(doc(
        "lane_change_west", "Town03 west straight: start in the right lane (5750), goal in the left lane (8209) "
        "200 m on, past an unsignalled junction; the lane change happens on the move; no other traffic",
        (-88.6, 118.0, -1.568), (-84.6, -80.0, -1.568), 12.0))  # above the ego's max_velocity 10


if __name__ == "__main__":
    main()
