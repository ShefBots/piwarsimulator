#!/usr/bin/env python3
import math
import numpy as np
from time import time
from brains.ExecutionState import ExecutionState
from world.WorldObject import *
from world.ObjectType import *


class RobotBrain:
    """
    the basics of every brain
    includes some common logic
    brains (logic) for different challenges inherit this
    """

    COLLISION_TOLERANCE = 0.02  # m, how close is too close
    # precision when trying to match objects based on location
    HOLDING_TOLERANCE = 0.01  # m

    # where time of flight sensors are installed pointing
    SENSOR_HEADINGS = [-90, 0, 90]

    # seconds to move when doing square up procedure
    SQUARE_UP_DURATION = 1

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

        # what we're doing (roughly)
        self.state = ExecutionState.NO_CONTROL

        # for squaring up
        self.square_up_heading = 0  # direction we're aligning to
        self.square_time = 0  # where we are in alignment process
        self.square_distance = 0  # distance away at start of alignment
        self.square_rotate_time = 0  # how much time to rotate for to get into alignment
        self.square_up_pass = 0  # do it twice for best results

        # for sensor output
        self.sensors = []  # any sensors
        self.TheWorld = []  # any objects returned by sensors
        self.sensor_measurements = {
            "manual_control": False
        }  # direct sensor measurements

        # are we close to colliding with something?
        self.collision = None

        # distances to the nearest wall in dirction of TOF sensors
        self.distances = [None] * len(self.SENSOR_HEADINGS)

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
        self.check_for_collision()
        self.find_distances()

        # TODO do we always want to try and stop on collisions? return?
        if not self.collision is None:
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

        if self.state == ExecutionState.SQUARING_UP:
            self.square_up()

    def find_goal(self):
        """default brain has no goal"""
        pass

    def check_for_collision(self):
        for obj in self.TheWorld[1:]:  # ignore the robot in 0
            if (
                self.TheWorld[0].get_distance(obj) < self.COLLISION_TOLERANCE
                and not self.is_holding(obj)
                and not (
                    # can't collide with flat objects
                    obj.object_type == ObjectType.MINE
                    or obj.object_type == ObjectType.LINE
                )
            ):
                print(f"WARNING: {obj} is a bit close")
                self.collision = obj
                return

    @staticmethod
    def match_objects(obj1, obj2):
        """try and determine if obj1 and obj2 are the same object based on coordinates"""
        if (
            obj1.object_type == obj2.object_type
            and obj1.color == obj2.color
            and obj1.get_distance(obj2, relative_to="center")
            < RobotBrain.HOLDING_TOLERANCE
        ):
            # assume that if they are sufficiently close they're the same
            return True
        else:
            # they aren't the same
            return False

    def is_holding(self, obj2):
        """check if obj is something we're holding"""
        for obj1 in self.holding:
            if RobotBrain.match_objects(obj1, obj2):
                return True
        return False

    def find_closest(self, object_type, color="", exclude=[]):
        """find the closest object_type"""
        closest = None
        closest_distance = 9e99
        for obj in self.TheWorld[1:]:  # skip the robot and check everything else
            dist = self.TheWorld[0].get_distance(obj)
            if (
                obj.object_type == object_type
                and (color == "" or obj.color == color)
                and not self.is_holding(obj)
                and not any(RobotBrain.match_objects(obj, obj2) for obj2 in exclude)
            ):
                if dist < closest_distance:
                    closest = obj
                    closest_distance = dist

        return (closest, closest_distance)

    def find_distances(self):
        """find the walls reported by sensors in each heading direction"""

        # reset distances
        self.distances = [None] * len(self.SENSOR_HEADINGS)

        # for each sensor find if there's a wall in that direction
        for k, _ in enumerate(self.SENSOR_HEADINGS):
            for obj in self.TheWorld[1:]:
                if obj.object_type != ObjectType.WALL:
                    continue
                if obj.heading == self.SENSOR_HEADINGS[k]:
                    self.distances[k] = self.TheWorld[0].get_distance(obj)
                    break

    def distance_forward(self):
        """return the distance to ahead wall"""
        return self.distances[self.SENSOR_HEADINGS.index(0)]

    def distance_left(self):
        """return the distance to left wall"""
        return self.distances[self.SENSOR_HEADINGS.index(-90)]

    def distance_right(self):
        """return the distance to left wall"""
        return self.distances[self.SENSOR_HEADINGS.index(90)]

    def square_up(self):
        """
        make sure we're parallel to a wall in a direction
        move side to side checking forward and side sensors distance change
        use angle of that to rotate a specified amount
        """

        dist = self.distances[self.SENSOR_HEADINGS.index(self.square_up_heading)]

        # direction of moment for alignment
        speed = -self.speed / 4
        if self.square_up_heading == 0:
            alignment_vector = np.array([speed, 0])
        elif abs(self.square_up_heading) == 90:
            # note, not super tested aligning to side walls
            alignment_vector = np.array([0, speed])
        else:
            dist = None  # trigger a failure unknown heading

        if self.square_up_pass == 1:
            speed = -speed
            alignment_vector = -alignment_vector

        # if no reading in alignment direction cancel and return
        if dist is None:
            print("No distance reading, canceling squaring")
            self.square_up_cancel()
            return

        time_at = time() - self.square_time

        if self.square_time == 0:
            # routine start
            self.square_time = time()
            self.square_distance = dist
            self.square_rotate_time = 0
            self.controller.set_plane_velocity(alignment_vector)

        elif time_at >= self.SQUARE_UP_DURATION and self.square_rotate_time == 0:
            # finished moving left, check and align
            self.controller.stop()

            base = speed * self.SQUARE_UP_DURATION
            height = self.square_distance - dist

            # needs a magic factor, function of distance and angle... iterate instead
            angle_to_wall = math.degrees(math.atan(height / base))

            print(f"Square across {base} rise {height} angle {angle_to_wall}")

            if math.fabs(angle_to_wall) < 2 or math.fabs(height) < 0.003:
                # already pretty aligned
                print("Already aligned, canceling squaring")
                self.square_up_cancel()
                return

            self.square_rotate_time = math.fabs(
                angle_to_wall / (self.turning_speed / 4)
            )
            if angle_to_wall < 0:
                self.controller.set_angular_velocity(-self.turning_speed / 4)
            else:
                self.controller.set_angular_velocity(self.turning_speed / 4)

        elif (
            time_at >= self.SQUARE_UP_DURATION + self.square_rotate_time
            and not self.controller.theta_vel == 0
        ):
            # finished rotating to align
            self.controller.stop()
            self.controller.set_plane_velocity(-alignment_vector)

        elif time_at >= self.SQUARE_UP_DURATION * 2 + self.square_rotate_time:
            # finished moving back
            # return control to the brain
            print(f"Done squaring up ({self.square_up_pass})")
            if self.square_up_pass == 5:
                print("Pass limit reached, canceling squaring")
                self.square_up_cancel()
                return
            else:
                self.square_up_pass += 1
                self.square_time = 0

    def square_up_cancel(self):
        self.controller.stop()
        self.square_time = 0
        self.square_rotate_time = 0
        self.square_up_pass = 0
        self.state = ExecutionState.PROGRAM_CONTROL
