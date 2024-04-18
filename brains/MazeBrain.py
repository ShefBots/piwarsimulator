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
        # self.state = ExecutionState.SQUARING_UP  # IF WE CAN PLACE THE ROBOT
        self.state = ExecutionState.FIND_WALL  # IF WE CAN'T PLACE THE ROBOT
        self.square_up_heading = 0  # align to wall in front
        self.last_reading = monotonic()
        self.move_count = 0  # when this hits like... 20 we should definetely be done
        self.found_gap = False

    def process(self):
        """do the basic brain stuff then do specific escape route things"""

        # check sensors and stop if collision is imminent
        if not super().process():
            # parent suggested something dangerous was up, don't continue
            return

        # don't do anything if the manual override is triggered
        if self.sensor_measurements["manual_control"]:
            return

        # do nothing if halted
        if self.state == ExecutionState.PROGRAM_COMPLETE:
            return

        if self.move_count == 20:
            self.state = ExecutionState.PROGRAM_COMPLETE

        if self.state == ExecutionState.FIND_WALL:
            # we might not be put down facing the wall correctly
            if self.found_gap == False:
                # rotate until we find a big gap
                self.set_angular_velocity(self.turning_speed / 2)
                if self.distance_forward() > 0.5:
                    print("Found the gap")
                    self.found_gap = True
                    # we need more space to the wall behind the gap
                    self.set_plane_velocity([0, 0.05])
            else:
                # we've passed the gap, next time we see a wall we're in the correct position
                if self.distance_forward() < 0.3:
                    print("Roughly aligned, square up")
                    self.set_angular_velocity(0)
                    self.set_plane_velocity([0, 0])
                    self.state = ExecutionState.SQUARING_UP

        # make sure we're parallel to a wall
        if self.state == ExecutionState.PROGRAM_CONTROL:
            # entry into solving routine
            self.state = ExecutionState.MOVE_LEFT
            self.move_count += 1
            print("Moving left")
            self.set_plane_velocity([-self.speed, 0])

        # check we're in range of some wall still
        if any(d is not None and not d == 9e99 for d in self.distances):
            self.last_reading = monotonic()

        # move in directions with space after getting close-ish to a wall
        if self.state == ExecutionState.MOVE_LEFT:
            if (
                not self.distance_left() is None
                and self.distance_left() < self.WALL_STOP_DISTANCE
            ):
                self.set_plane_velocity([0, self.speed])
                self.state = ExecutionState.MOVE_FORWARD
                self.move_count += 1
                print("Moving forward")

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
                    self.set_plane_velocity([self.speed, 0])
                    self.state = ExecutionState.MOVE_RIGHT
                    self.move_count += 1
                    print("Moving right")
                elif (
                    self.distance_left() is None
                    or self.distance_left() > self.WALL_STOP_DISTANCE
                ):
                    # there's space on the left, go that way
                    self.set_plane_velocity([-self.speed, 0])
                    self.state = ExecutionState.MOVE_LEFT
                    self.move_count += 1
                    print("Moving left")
                else:
                    # possibly not square anymore? reenter alignment
                    self.controller_stop()
                    self.state == ExecutionState.SQUARING_UP

            if monotonic() - self.last_reading > (0.03 / self.speed):
                # stop if no walls have been seen for X seconds (30 cm distance equiv)
                # this won't work, there are will always be something detected in reality
                # TODO end based on a long distance to rear wall?
                self.controller_stop()
                self.state = ExecutionState.PROGRAM_COMPLETE

        elif self.state == ExecutionState.MOVE_RIGHT:
            if (
                not self.distance_right() is None
                and self.distance_right() < self.WALL_STOP_DISTANCE
            ):
                self.set_plane_velocity([0, self.speed])
                self.state = ExecutionState.MOVE_FORWARD
                self.move_count += 1
                print("Moving forward")
