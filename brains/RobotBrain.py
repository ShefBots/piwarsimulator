#!/usr/bin/env python3
import math
import copy
from world.WorldObject import *
from world.ObjectType import *

# note: at some point we can probably use child classes to define
# what type of AI challenge we're dealing with


class RobotBrain:
    """the basics of every brain"""

    def __init__(self, **kwargs):
        self.robot = kwargs.get("robot", None)
        self.controller = kwargs.get("controller", None)
        self.speed = kwargs.get("speed", 0.001)  # 1 mm/s
        self.turning_speed = kwargs.get("turning_speed", 1)  # 1 degrees/s
        assert isinstance(self.robot, WorldObject)
        assert self.robot.object_type == ObjectType.ROBOT

        # the long term goal - we get this from a scan result and
        # use it to cross verify with subsequent scan results
        self.goal = None

        # anything we're transporting (exterior world objects when simulating)
        self.holding = []

        self.collision_tolerance = kwargs.get("collision_tolerance", 0.01)  # m

        # for sensor output
        self.sensors = []  # any sensors
        self.TheWorld = []  # any objects returned by sensors
        self.sensor_measurements = {
            "manual_control": False
        }  # direct sensor measurements

    def add_sensor(self, sensor):
        self.sensors.append(sensor)

    def poll_sensors(self):
        """poll all sensors attached to the robot and check for collisions"""
        # add in ourself to start with
        self.TheWorld = [
            # scanned coordinates should be relative to robot north
            WorldObject(
                object_type=ObjectType.ROBOT,
                x=0,
                y=0,
                w=self.robot.width,
                h=self.robot.height,
                angle=0,
            )
        ]
        for s in self.sensors:
            objects, readings = s.do_scan()
            self.TheWorld += objects
            for k, v in readings.items():
                self.sensor_measurements[k] = v

    def process(self):
        """basic logic is to just not hit anything & respond to control input"""
        self.poll_sensors()
        if self.check_for_collision():
            self.controller.stop()

        # print(self.sensor_measurements)
        if self.sensor_measurements["manual_control"]:
            self.controller.set_plane_velocity(
                [
                    self.sensor_measurements["sideways_vel"],
                    self.sensor_measurements["forward_vel"],
                ]
            )
            self.controller.set_angular_velocity(
                self.sensor_measurements["angular_vel"]
            )

    def find_goal(self):
        """default brain has no goal"""
        pass

    def check_for_collision(self):
        for obj in self.TheWorld[1:]:  # ignore the robot in 0
            # the code for the real robot should probably treat the ignore slightly differently...
            # TODO collisions in a circle that's the robot and holding isn't effective,
            # replace this with something that checks the radius of both independently
            if (
                self.TheWorld[0].get_distance(obj) < self.collision_tolerance
                and not obj.exterior in self.holding
                and not (
                    # can't collide with flat objects
                    obj.object_type == ObjectType.MINE
                    or obj.object_type == ObjectType.LINE
                )
            ):
                print(obj)
                print("Yikes! Something's a bit close!")
                return True
        return False
