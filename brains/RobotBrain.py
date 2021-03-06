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

        # the long term goal - we get this from a scan result and
        # use it to cross verify with subsequent scan results
        self.goal = None

        # anything we're transporting
        self.holding = []

        # the set of directions we're currently following
        # in the format of [x, y] where
        #   x is 1 or 2 to indicate move, turn
        #   y is the distance to go/amount to turn
        #   if x is 3, the gripper with y = 1 to open and y = 2 to close?
        self.movement_queue = []

    def process(self, sensor_information):
        self.check_for_collision(sensor_information)

    def find_goal(self, sensor_information):
        pass

    def check_for_collision(self, sensor_information, ignore=None):
        tr = self.robot.radius + self.held_radius()
        for obj in sensor_information:
            # the code for the real robot should probably treat the ignore slightly differently...
            # TODO collisions in a circle that's the robot and holding isn't effective,
            # replace this with something that checks the radius of both independently
            if obj.distance - tr - obj.radius < 0.05 and not obj.parent in ignore and not obj.parent in self.holding:
                print("yikes! that's a bit close in'it?")
                self.movement_queue = []
                return True
                # this isn't working because the same move just gets repeated I think?
        return False

    def execute_rotate(self, amount):
        """Communicate to the robot to rotate a given amount..."""
        self.robot.angle += amount

        # rotate objects we're holding about us
        for obj in self.holding:
            obj.angle += amount

            dx = math.cos(math.radians(-amount)) * (obj.x - self.robot.x) - math.sin(math.radians(-amount)) * (obj.y - self.robot.y)
            dy = math.sin(math.radians(-amount)) * (obj.x - self.robot.x) + math.cos(math.radians(-amount)) * (obj.y - self.robot.y)

            obj.x = self.robot.x + dx
            obj.y = self.robot.y + dy

    def execute_move(self, dist, heading):
        """Communicate to the robot to move a given amount in a given direction..."""
        x = dist * math.sin(math.radians(heading))
        y = dist * math.cos(math.radians(heading))

        self.robot.x += x
        self.robot.y += y
        # perhaps here we can check, would this move collide with something
        # if so don't do it

        for obj in self.holding:
            obj.x += x
            obj.y += y

    # this may be named incorrectly, it executes the movement_queue
    # in time to the timestep so the robot animates correctly, but with
    # real hardware we need something that does this same thing, but just
    # sends the command to the motors
    def simulate(self, dt):
        """When simulating the robot wouldn't otherwise move itself..."""
        if self.movement_queue:
            movement_type = self.movement_queue[0][0]
            movement_left = self.movement_queue[0][1]

            if movement_type == 1:
                movement_amount = self.speed * dt
                movement_function = lambda x: self.execute_move(x, self.robot.angle)
            elif movement_type == 2:
                movement_amount = self.turning_speed * dt
                movement_function = lambda x: self.execute_rotate(x)

            # handle turning counter-clockwise and reversing
            if movement_left < 0:
                movement_amount = -movement_amount
            if movement_amount > movement_left and movement_amount > 0:
                movement_amount = movement_left
            elif movement_amount < movement_left and movement_amount < 0:
                movement_amount = movement_left

            movement_left -= movement_amount
            if abs(movement_left) > 1e-10:
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
