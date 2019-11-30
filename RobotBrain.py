#!/usr/bin/env python3
from WorldObject import *
from ObjectType import *
import math
import copy

# note: at some point we can probably use child classes to define
# what type of AI challenge we're dealing with

class RobotBrain():

    def __init__(self, *args, **kwargs):
#        print("Building brain")
        self.robot = kwargs.get('robot', None)
        self.speed = kwargs.get('speed', 0.001) # 1 mm/s
        self.turningspeed = kwargs.get('turningspeed', 1) # 1 degrees/s
        assert isinstance(self.robot, WorldObject)
        assert self.robot.objecttype == ObjectType.ROBOT

        # need to set long term goal
        # this should be a copy of a scan result so that we can
        # use inertial tracking in case it doesn't show up in the next scan
        self.goal = None
        # need to initalise movement history

        # anything we're transporting
        self.holding = []

    def move(self, sensorinformation):
        # find something to move towards
        goal = self.findGoal(sensorinformation)

        # if we didn't find anything, were we already moving towards something?
#        if goal != None and self.goal == None:
#            print("setting new long term goal")
#            self.goal = copy.copy(goal)
        if goal == None and self.goal == None:
            print("no goals, doing nothing")
            return

        # for now operate off the short term goal only

        # rotate so we are facing the target
        headingoffset = goal.heading - self.robot.angle
        if abs(headingoffset) > 15 :
            if headingoffset > 0:
                self.robot.angle += self.turningspeed
            else:
                self.robot.angle -= self.turningspeed
        else: # we're already facing so move
            self.robot.x += self.speed * math.sin(math.radians(goal.heading))
            self.robot.y += self.speed * math.cos(math.radians(goal.heading))

    def findGoal(self, sensorinformation):
        """find the closest TARET or ZONE"""
        closest = None
        closestdistance = 9e99
        for x in sensorinformation:
            # only look for a target if we're holding nothing
            if x.objecttype == ObjectType.TARGET and len(self.holding) == 0:
                if x.distance < closestdistance:
                    closest = x
                    closestdistance = x.distance
            # otherwise find the zone that matches the target colour

        return closest
