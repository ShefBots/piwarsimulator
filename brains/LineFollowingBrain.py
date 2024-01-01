#!/usr/bin/env python3
import math
import copy

from world.WorldObject import *
from world.ObjectType import *
from brains.RobotBrain import RobotBrain


class LineFollowingBrain(RobotBrain):
    """logic for the line following challenge"""

    def process(self):
        """do the basic brain stuff then do specific ecodisaster things"""

        # check sensors and stop if collision is imminent
        super().process()
        # don't do anything if the manual override is triggered
        if self.sensor_measurements["manual_control"]:
            return

        # find something to move towards
        (goal, goal_distance) = self.find_goal()
        if goal == None:
            self.controller.stop()
            return

       # TODO write line following logic

    def find_goal(self):
        """find the closest LINE"""
        closest = None
        closest_distance = 9e99
        for obj in self.TheWorld[1:]:
            dist = self.TheWorld[0].get_distance(obj)
            # only look for a target if we're holding nothing
            if obj.object_type == ObjectType.LINE:
                if dist < closest_distance:
                    closest = obj
                    closest_distance = dist

        return (closest, closest_distance)
