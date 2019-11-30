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

        # are we within grabbing distance of the goal?
        tr = self.robot.radius + self.heldRadius()
        if goal.objecttype == ObjectType.ZONE:
            pass
        else:
            tr += goal.radius
        if goal.distance < tr:
            if goal.objecttype == ObjectType.TARGET:
                print("grabbing goal!")
                self.holding.append(goal.parent)
            if goal.objecttype == ObjectType.ZONE and len(self.holding) > 0:
                self.holding[0].ignore = 1
                print("dropping off target!")
                self.holding.pop(0)
                # move backwards a little after dropping off target
                self.executeMove(-self.speed, goal.heading)
            return


        # for now operate off the short term goal only

        # rotate so we are facing the target
        headingoffset = goal.heading - self.robot.angle
        # NOTE: sometimes we rotate in the wrong direction
        # this is probably an x - 360 type issue?
        if abs(headingoffset) > self.turningspeed/2:
            if headingoffset > 0:
                self.executeRotate(self.turningspeed)
            else:
                self.executeRotate(-self.turningspeed)
        else: # we're already facing so move
            self.executeMove(self.speed, goal.heading)

    def executeRotate(self, amount):
        self.robot.angle += amount

        # rotate objects we're holding about us
        for obj in self.holding:
            obj.angle += amount

            dx = math.cos(math.radians(-amount)) * (obj.x - self.robot.x) - math.sin(math.radians(-amount)) * (obj.y - self.robot.y)
            dy = math.sin(math.radians(-amount)) * (obj.x - self.robot.x) + math.cos(math.radians(-amount)) * (obj.y - self.robot.y)

            obj.x = self.robot.x + dx
            obj.y = self.robot.y + dy

    def executeMove(self, dist, heading):
        x = dist * math.sin(math.radians(heading))
        y = dist * math.cos(math.radians(heading))
        self.robot.x += x
        self.robot.y += y

        for obj in self.holding:
            obj.x += x
            obj.y += y

    def findGoal(self, sensorinformation):
        """find the closest TARET or ZONE"""
        closest = None
        closestdistance = 9e99
        for obj in sensorinformation:
            # only look for a target if we're holding nothing
            if obj.objecttype == ObjectType.TARGET and len(self.holding) == 0:
                if obj.distance < closestdistance:
                    closest = obj
                    closestdistance = obj.distance
            # otherwise find the zone that matches the target colour
            elif len(self.holding) > 0 and obj.objecttype == ObjectType.ZONE and obj.color == self.holding[0].color:
                return obj

        return closest

    def heldRadius(self):
        """Roughly increase in radius due to items being held"""
        r = 0
        for obj in self.holding:
            dist = math.sqrt(math.pow(obj.x - self.robot.x, 2) + math.pow(obj.y - self.robot.y, 2))
            tr = dist - self.robot.radius + obj.radius
            if tr > r:
                r = tr

        return r
