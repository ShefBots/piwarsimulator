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
        # in the format of [x, y] where
        #   x is 1 or 2 to indicate move, turn
        #   y is the distance to go/amount to turn
        self.movement_queue = [[1, 0.1]]

    def process(self, sensor_information):
        pass

    def find_goal(self, sensor_information):
        pass

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

    def simulate(self, dt):
        """When simulating the robot wouldn't otherwise move itself..."""
        if len(self.movement_queue) != 0:
            movement_type = self.movement_queue[0][0]
            movement_left = self.movement_queue[0][1]

            if movement_type == 1:
                movement_amount = self.speed * dt
                movement_function = lambda x: self.execute_move(x, self.robot.angle)
            elif movement_type == 2:
                movement_amount = self.turning_speed * dt
                movement_function = lambda x: self.execute_rotate(x)

            movement_left -= movement_amount
            if movement_left > 0:
                movement_function(movement_amount)
                self.movement_queue[0][1] -= movement_amount
            else:
                movement_function(self.movement_queue[0][1])
                self.movement_queue.pop(0)

    def held_radius(self):
        """Roughly increase in radius due to items being held"""
        r = 0
        for obj in self.holding:
            dist = math.sqrt(math.pow(obj.x - self.robot.x, 2) + math.pow(obj.y - self.robot.y, 2))
            tr = dist - self.robot.radius + obj.radius
            if tr > r:
                r = tr

        return r
