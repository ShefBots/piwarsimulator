#!/usr/bin/env python3
import numpy as np
from pygame import Color
from shapely.affinity import scale
from shapely.geometry import LineString
from shapely.geometry import Point
from brains.ExecutionState import ExecutionState
from brains.RobotBrain import RobotBrain
from world.WorldObject import *
from world.ObjectType import *


def find_goal(brain, use_front=False):
    """find the closest TARGET or ZONE depending on execution state"""

    if len(brain.holding) > 0:
        barrel = brain.holding[0]
        if barrel.color == Color("darkgreen"):
            holding_color = "darkgreen"
        elif barrel.color == Color("red"):
            holding_color = "red"
        else:
            print("yikes shouldn't get here")
        # goal_color = brain.GOAL_MAPPING[holding_color]
        goal_color = Color(brain.GOAL_MAPPING[holding_color])
    else:
        goal_color = None
    # treat the middle of the gripper as the center
    gripper_center = (0, brain.robot.height / 2 + 0.025)

    if brain.state == ExecutionState.MOVE_TO_ZONE:
        # need to have find_closest return ranked list? or another function to sort TheWorld?
        return brain.find_closest(ObjectType.ZONE, color=goal_color)

    elif brain.state == ExecutionState.MOVE_TO_BARREL:
        # treat the middle of the gripper as the center, so find barrel closest
        # to the front of the robot, 0.025 = barrel radius
        # finds the closest barrel that's in a straight line

        # need to ignore barrels in/near zones
        exclude = []
        for _, i in enumerate(brain.GOAL_MAPPING):
            zone, _ = brain.find_closest(
                ObjectType.ZONE, color=Color(brain.GOAL_MAPPING[i])
            )
            if not zone is None:
                for obj in brain.TheWorld[1:]:
                    if obj.object_type == ObjectType.BARREL and obj.outline.intersects(
                        zone.outline
                    ):
                        exclude.append(obj)

        if use_front == True:
            return brain.find_in_front(ObjectType.BARREL, relative_to=gripper_center)
        else:
            return brain.find_closest(
                ObjectType.BARREL, relative_to=gripper_center, exclude=exclude
            )
        # TODO fall back to closest if there's nothing in front?

    elif brain.state == ExecutionState.DROP_OFF_BARREL:
        # we have a barrel and we're near the zone, find the bit of the zone
        # closet to where we are to put the barrel

        # the zone
        zone, _ = brain.find_closest(ObjectType.ZONE, color=goal_color)

        barrel_center_point = Point(barrel.center)
        # the closest part of the zone to the barrel
        zone_intersection_point = zone.outline.exterior.interpolate(
            zone.outline.exterior.project(barrel_center_point)
        )

        # if we're not inside the zone, find a point inside it to aim towards

        # the line between the barrel and edge of zone
        line_to_zone = LineString([barrel_center_point, zone_intersection_point])

        if not zone.outline.intersects(barrel_center_point):
            # barrel is not inside the zone
            # scaled by *2 because scaling both ends
            fact = (barrel.radius * 3 + line_to_zone.length) / line_to_zone.length
            scaled_line = scale(line_to_zone, xfact=fact, yfact=fact)
            drop_point = scaled_line.coords[1]
        elif zone.outline.contains(barrel.outline):
            # the barrel is entirely in the zone, we're done
            return (barrel, 0)
        else:
            # slightly inside the zone
            # trying to find some point slightly inside the zone still
            # but more likely just some point slightly ahead of where we are
            drop_point = line_to_zone.interpolate(
                line_to_zone.length - barrel.radius * 3
            )
            drop_point = (drop_point.x, drop_point.y)
            # if we're just inside the zone then the interpolated location along
            # the line ends up being the end of the line instead of the correct barrel location
            # so we need to extrapolate instead
            if all(barrel.center == drop_point):
                fact = (barrel.radius * 3) / line_to_zone.length
                scaled_line = scale(line_to_zone, xfact=fact, yfact=fact)
                drop_point = scaled_line.coords[0]

        # temporary world object to move towards
        barrel_drop = WorldObject(
            object_type=ObjectType.DROP_SPOT,
            x=drop_point[0],
            y=drop_point[1],
            radius=barrel.radius,
            color="gray",
        )
        # work out the heading
        # possibly more useful to use the heading between the barrel and
        # drop off point instead of the robot and drop off point?
        offset_drop = drop_point - barrel.center
        barrel_drop.heading = math.degrees(
            # math.atan2(barrel_drop.center[0], barrel_drop.center[1])
            math.atan2(offset_drop[0], offset_drop[1])
        )

        brain.TheWorld.append(barrel_drop)
        return (
            barrel_drop,
            brain.TheWorld[0].get_distance(
                barrel_drop,
                relative_to=gripper_center,
            ),
        )
    else:
        return (None, 9e99)
