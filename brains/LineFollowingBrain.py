#!/usr/bin/env python3
from world.WorldObject import *
from world.ObjectType import *
from brains.RobotBrain import RobotBrain


class LineFollowingBrain(RobotBrain):
    """logic for the line following challenge"""

    def process(self):
        """do the basic brain stuff then do specific line following things"""

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
        return self.find_closest(ObjectType.LINE)
