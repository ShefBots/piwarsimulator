#!/usr/bin/env python3
import math
import copy
from world.WorldObject import *
from world.ObjectType import *

# note: at some point we can probably use child classes to define
# what type of AI challenge we're dealing with

class RobotBrain():
    """the basics of every brain"""

    def __init__(self, **kwargs):
        self.robot = kwargs.get('robot', None)
        self.controller = kwargs.get('controller', None)
        self.speed = kwargs.get('speed', 0.001) # 1 mm/s
        self.turning_speed = kwargs.get('turning_speed', 1) # 1 degrees/s
        assert isinstance(self.robot, WorldObject)
        assert self.robot.object_type == ObjectType.ROBOT

        # the long term goal - we get this from a scan result and
        # use it to cross verify with subsequent scan results
        self.goal = None

        # anything we're transporting (exterior world objects when simulating)
        self.holding = []

        # the set of directions we're currently following
        # in the format of [x, y] where
        #   x is 1 or 2 to indicate move, turn
        #   y is the distance to go/amount to turn
        #   if x is 3, the gripper with y = 1 to open and y = 2 to close?
        self.movement_queue = []

        # for sensor output
        self.sensors = []
        self.TheWorld = []

    def add_sensor(self, sensor):
        self.sensors.append(sensor);
    
    def poll_sensors(self):
        """poll all sensors attached to the robot and check for collisions"""
        # add in ourself to start with
        self.TheWorld = [WorldObject(object_type=ObjectType.ROBOT, x=0, y=0, radius=self.robot.radius, angle=self.robot.angle)]
        for s in self.sensors:
            self.TheWorld += s.do_scan()

    def process(self):
        """basic logic is to just not hit anything"""
        self.poll_sensors();
        if self.check_for_collision():
            self.controller.stop()

    def find_goal(self):
        """default brain has no goal"""
        pass

    def check_for_collision(self):
        tr = self.radius()
        for obj in self.TheWorld[1:]: # ignore the robot in 0
            # the code for the real robot should probably treat the ignore slightly differently...
            # TODO collisions in a circle that's the robot and holding isn't effective,
            # replace this with something that checks the radius of both independently
            if obj.distance() - tr - obj.radius < 0.05 and not obj.exterior in self.holding:
                print(obj)
                print("yikes! that's a bit close in'it?")
                return True
        return False

    def radius(self):
        """estimated radius including anything being held"""
        r = 0
        for obj in self.holding:
            tr = obj.distance(self.robot) + obj.radius
            if tr > r:
                r = tr
        
        # if holding nothing we're at least as big as ther robot itself
        if r <= self.robot.radius:
            r = self.robot.radius

        return r
