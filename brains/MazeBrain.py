#!/usr/bin/env python3
from time import monotonic
from brains.ExecutionState import ExecutionState
from brains.RobotBrain import RobotBrain
from world.WorldObject import *
from world.ObjectType import *


class MazeBrain(RobotBrain):
    """logic for the escape route challenge"""

    # how close to get to walls before moving to next program stage
    WALL_STOP_DISTANCE = 0.15

    def __init__(self, **kwargs):
        super(MazeBrain, self).__init__(**kwargs)
        # start by squaring up (making sure we're aligned with the walls)
        self.state = ExecutionState.SQUARING_UP
        self.square_up_heading = 0  # align to wall in front
        self.last_reading = monotonic()

    def process(self):
        """do the basic brain stuff then do specific escape route things"""

        # check sensors and stop if collision is imminent
        super().process()
        # don't do anything if the manual override is triggered
        if self.sensor_measurements["manual_control"]:
            return

        # do nothing if halted
        if self.state == ExecutionState.STOPPED:
            return

        # make sure we're parallel to a wall
        if self.state == ExecutionState.PROGRAM_CONTROL:
            # entry into solving routine
            self.state = ExecutionState.MOVE_LEFT
            self.controller.set_plane_velocity([-self.speed, 0])

        # check we're in range of some wall still
        if any(d is not None for d in self.distances):
            self.last_reading = monotonic()

        # move in directions with space after getting close-ish to a wall
        if self.state == ExecutionState.MOVE_LEFT:
            if (
                not self.distance_left() is None
                and self.distance_left() < self.WALL_STOP_DISTANCE
            ):
                self.controller.set_plane_velocity([0, self.speed])
                self.state = ExecutionState.MOVE_FORWARD

        elif self.state == ExecutionState.MOVE_FORWARD:
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
                    self.state = ExecutionState.MOVE_RIGHT
                elif (
                    self.distance_left() is None
                    or self.distance_left() > self.WALL_STOP_DISTANCE
                ):
                    # there's space on the left, go that way
                    self.controller.set_plane_velocity([-self.speed, 0])
                    self.state = ExecutionState.MOVE_LEFT
                else:
                    # possibly not square anymore? reenter alignment
                    self.controller.stop()
                    self.state == ExecutionState.SQUARING_UP

            if monotonic() - self.last_reading > (0.03 / self.speed):
                # stop if no walls have been seen for X seconds (30 cm distance equiv)
                self.controller.stop()
                self.state = ExecutionState.STOPPED

        elif self.state == ExecutionState.MOVE_RIGHT:
            if (
                not self.distance_right() is None
                and self.distance_right() < self.WALL_STOP_DISTANCE
            ):
                self.controller.set_plane_velocity([0, self.speed])
                self.state = ExecutionState.MOVE_FORWARD
