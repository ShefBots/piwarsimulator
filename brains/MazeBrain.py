#!/usr/bin/env python3
from time import time
from world.WorldObject import *
from world.ObjectType import *
from brains.RobotBrain import RobotBrain


class MazeBrain(RobotBrain):
    """logic for the escape route challenge"""

    # how close to get to walls before moving to next program stage
    WALL_STOP_DISTANCE = 0.15

    # things the robot could be doing
    ALIGNMENT = 0
    MOVE_LEFT = 1
    MOVE_FORWARD = 2
    MOVE_RIGHT = 3
    STOPPED = 4

    CURRENT_MODE = ALIGNMENT

    def __init__(self, **kwargs):
        super(MazeBrain, self).__init__(**kwargs)
        self.last_reading = time()

    def process(self):
        """do the basic brain stuff then do specific escape route things"""

        # check sensors and stop if collision is imminent
        super().process()
        # don't do anything if the manual override is triggered
        if self.sensor_measurements["manual_control"]:
            return

        # do nothing if halted
        if self.CURRENT_MODE == self.STOPPED:
            return

        # make sure we're parallel to a wall
        if self.CURRENT_MODE == self.ALIGNMENT:
            # TODO wall alignment routine
            # move side to side checking forward and side sensors distance change
            # use angle of that to rotate a specified amount
            # then move backwards some to ensure distance to walls > WALL_STOP_DISTANCE
            self.CURRENT_MODE = self.MOVE_LEFT
            self.controller.set_plane_velocity([-self.speed, 0])

        # check we're in range of some wall still
        if any(d is not None for d in self.distances):
            self.last_reading = time()

        # move in directions with space after getting close-ish to a wall
        if self.CURRENT_MODE == self.MOVE_LEFT:
            if (
                not self.distance_left() is None
                and self.distance_left() < self.WALL_STOP_DISTANCE
            ):
                self.controller.set_plane_velocity([0, self.speed])
                self.CURRENT_MODE = self.MOVE_FORWARD

        elif self.CURRENT_MODE == self.MOVE_FORWARD:
            if (
                not self.distance_forward() is None
                and self.distance_forward() < self.WALL_STOP_DISTANCE
            ):
                if (
                    self.distance_right() is None
                    or self.distance_right() > self.WALL_STOP_DISTANCE
                ):
                    # there's space on the right, go that way
                    self.controller.set_plane_velocity([self.speed, 0])
                    self.CURRENT_MODE = self.MOVE_RIGHT
                elif (
                    self.distance_left() is None
                    or self.distance_left() > self.WALL_STOP_DISTANCE
                ):
                    # there's space on the left, go that way
                    self.controller.set_plane_velocity([-self.speed, 0])
                    self.CURRENT_MODE = self.MOVE_LEFT
                else:
                    # possibly not square anymore? reenter alignment
                    self.controller.stop()
                    self.CURRENT_MODE == self.ALIGNMENT

            if time() - self.last_reading > (0.03 / self.speed):
                # stop if no walls have been seen for X seconds (30 cm distance equiv)
                self.controller.stop()
                self.CURRENT_MODE = self.STOPPED

        elif self.CURRENT_MODE == self.MOVE_RIGHT:
            if (
                not self.distance_right() is None
                and self.distance_right() < self.WALL_STOP_DISTANCE
            ):
                self.controller.set_plane_velocity([0, self.speed])
                self.CURRENT_MODE = self.MOVE_FORWARD
