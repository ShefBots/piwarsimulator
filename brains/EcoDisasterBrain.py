#!/usr/bin/env python3
import math
import copy

from world.WorldObject import *
from world.ObjectType import *
from brains.RobotBrain import RobotBrain


class EcoDisasterBrain(RobotBrain):
    """logic for the ecodisaster challenge"""

    # will need to account for gripper in collisions, this'll probably need a constructor
    # will need to account for zones in collisions

    GOAL_MAPPING = {"darkgreen": "blue", "red": "yellow"}

    GRIPPER_ANGLE_TOLERANCE = 2  # degree
    GRIPPER_TOLERANCE = 0.02  # m

    def process(self):
        """do the basic brain stuff then do specific ecodisaster things"""

        # check sensors and stop if collision is imminent
        super().process()

        # TODO: redo all this logic for new architecture...

        # find something to move towards
        (goal, goal_distance) = self.find_goal()
        if goal == None:
            return

        # print(goal_distance)
        # print(self.radius() + self.GRIPPER_TOLERANCE)

        # if in range of target
        if goal_distance < self.GRIPPER_TOLERANCE:
            print("In range of goal!")
            self.controller.stop()
        else:
            # turn towards target
            if goal.heading > self.GRIPPER_ANGLE_TOLERANCE:
                self.controller.set_angular_velocity(self.turning_speed)
            elif goal.heading < -self.GRIPPER_ANGLE_TOLERANCE:
                self.controller.set_angular_velocity(-self.turning_speed)
            else:
                self.controller.set_angular_velocity(0)
                self.controller.set_plane_velocity([0, self.speed])

        # TODO if the angles don't match, backup rotate, try to grab again

        # if a barrel pick it up
        if goal.object_type == ObjectType.BARREL:
            pass
            # print("Grabbing barrel")
            # self.holding.append(goal)
            # goal.exterior.is_held = True  # this is a simulation thing

        # if a zone drop the barrel off
        if goal.object_type == ObjectType.ZONE:
            pass
            # print("Dropping off barrel")
            # self.holding.pop(0)
            # # TODO need some way to distinguish barrels in zones

    def find_goal(self):
        """find the closest TARGET or ZONE"""
        closest = None
        closest_distance = 9e99
        for obj in self.TheWorld[1:]:
            dist = self.TheWorld[0].get_distance(obj)
            # only look for a target if we're holding nothing
            if (
                obj.object_type == ObjectType.BARREL
                and not obj.exterior in self.holding
            ):
                if dist < closest_distance:
                    closest = obj
                    closest_distance = dist
            # otherwise find the zone that matches the target colour
            elif (
                len(self.holding) > 0
                and obj.object_type == ObjectType.ZONE
                and obj.color == self.GOAL_MAPPING[self.holding[0].color]
                and dist < closest_distance
            ):
                closest = obj
                closest_distance = dist

        return (closest, closest_distance)
