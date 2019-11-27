#!/usr/bin/env python3
from WorldObject import *
from ObjectType import *

class RobotBrain():

    def __init__(self, *args, **kwargs):
#        print("Building brain")
        self.robot = kwargs.get('robot', None)
        assert isinstance(self.robot, WorldObject)
        assert self.robot.objecttype == ObjectType.ROBOT

        # need to set default goal
        self.goal = None
        # need to initalise movement history

    def move(self, sensorinformation):
        self.robot.x += 0.1
        pass
