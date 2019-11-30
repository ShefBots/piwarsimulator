#!/usr/bin/env python3
from WorldObject import *
from ObjectType import *

# note: at some point we can probably use child classes to define
# what type of AI challenge we're dealing with

class RobotBrain():

    def __init__(self, *args, **kwargs):
#        print("Building brain")
        self.robot = kwargs.get('robot', None)
        assert isinstance(self.robot, WorldObject)
        assert self.robot.objecttype == ObjectType.ROBOT

        # need to set default goal
        self.goal = None
        # need to initalise movement history

        # anything we're transporting
        self.holding = []

    def move(self, sensorinformation):
        if self.goal == None:
            self.goal = self.findGoal(sensorinformation)
        if self.goal == None:
            return

        self.robot.x += 0.1
        pass

    def findGoal(self, sensorinformation):
        """find the closest TARET"""
        closest = None
        closestdistance = 9e99
        for x in sensorinformation:
            if x.objecttype == ObjectType.TARGET:
                if x.distance < closestdistance:
                    closest = x
                    closestdistance = x.distance

        return closest
