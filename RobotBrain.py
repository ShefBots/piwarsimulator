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
        self.turning_speed = kwargs.get('turning_speed', 1) # 1 degrees/s
        assert isinstance(self.robot, WorldObject)
        assert self.robot.object_type == ObjectType.ROBOT

        # need to set long term goal
        # this should be a copy of a scan result so that we can
        # use inertial tracking in case it doesn't show up in the next scan
        self.goal = None
        # need to initalise movement history

        # anything we're transporting
        self.holding = []

        # the set of directions we're currently following
        self.movement_queue = []

    def move(self, sensor_information):
        # find something to move towards
        goal = self.find_goal(sensor_information)

        # if we didn't find anything, were we already moving towards something?
#        if goal != None and self.goal == None:
#            print("setting new long term goal")
#            self.goal = copy.copy(goal)
        if goal == None and self.goal == None:
            print("no goals, doing nothing")
            return

        # are we within grabbing distance of the goal?
        tr = self.robot.radius + self.held_radius()
        if goal.object_type == ObjectType.ZONE:
            pass
        else:
            tr += goal.radius
        if goal.distance < tr:
            if goal.object_type == ObjectType.TARGET:
                print("grabbing goal!")
                self.holding.append(goal.parent)
            if goal.object_type == ObjectType.ZONE and len(self.holding) > 0:
                self.holding[0].ignore = True
                print("dropping off target!")
                self.holding.pop(0)
                # move backwards a little after dropping off target
                self.execute_move(-self.speed, goal.heading)
            return


        # for now operate off the short term goal only

        # rotate so we are facing the target
        heading_offset = goal.heading - self.robot.angle
        # NOTE: sometimes we rotate in the wrong direction
        # this is probably an x - 360 type issue?
        if abs(heading_offset) > self.turning_speed / 2:
            if heading_offset > 0:
                self.execute_rotate(self.turning_speed)
            else:
                self.execute_rotate(-self.turning_speed)
        else: # we're already facing so move
            self.execute_move(self.speed, goal.heading)

    def execute_rotate(self, amount):
        self.robot.angle += amount

        # rotate objects we're holding about us
        for obj in self.holding:
            obj.angle += amount

            dx = math.cos(math.radians(-amount)) * (obj.x - self.robot.x) - math.sin(math.radians(-amount)) * (obj.y - self.robot.y)
            dy = math.sin(math.radians(-amount)) * (obj.x - self.robot.x) + math.cos(math.radians(-amount)) * (obj.y - self.robot.y)

            obj.x = self.robot.x + dx
            obj.y = self.robot.y + dy

    def execute_move(self, dist, heading):
        x = dist * math.sin(math.radians(heading))
        y = dist * math.cos(math.radians(heading))
        self.robot.x += x
        self.robot.y += y

        for obj in self.holding:
            obj.x += x
            obj.y += y

    def find_goal(self, sensor_information):
        """find the closest TARET or ZONE"""
        closest = None
        closest_distance = 9e99
        for obj in sensor_information:
            # only look for a target if we're holding nothing
            if obj.object_type == ObjectType.TARGET and len(self.holding) == 0:
                if obj.distance < closest_distance:
                    closest = obj
                    closest_distance = obj.distance
            # otherwise find the zone that matches the target colour
            elif len(self.holding) > 0 and obj.object_type == ObjectType.ZONE and obj.color == self.holding[0].color:
                return obj

        return closest

    def held_radius(self):
        """Roughly increase in radius due to items being held"""
        r = 0
        for obj in self.holding:
            dist = math.sqrt(math.pow(obj.x - self.robot.x, 2) + math.pow(obj.y - self.robot.y, 2))
            tr = dist - self.robot.radius + obj.radius
            if tr > r:
                r = tr

        return r
