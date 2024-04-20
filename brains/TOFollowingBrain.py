#!/usr/bin/env python3
from time import monotonic
from brains.ExecutionState import ExecutionState
from brains.RobotBrain import RobotBrain
from world.WorldObject import *
from world.ObjectType import *


class TOFollowingBrain(RobotBrain):
    """logic for the line following challenge using TOFs only"""

    NEAR_WALL = 0.22  # turn
    SLOW_WALL = 0.3  # slow down to approach

    CENTER_TOLERANCE = 0.05  # try to be in the center-ish

    def __init__(self, **kwargs):
        super(TOFollowingBrain, self).__init__(**kwargs)
        self.state = ExecutionState.PROGRAM_INIT
        # Don't use side TOFs to start, unlock after something is detected ahead
        self.side_tof_lockout = True

        # on the first process w/out parking break go forward
        self.started = False

    def process(self):
        """do the basic brain stuff then do specific line following things"""

        # check sensors and stop if collision is imminent
        if not super().process():
            # parent suggested something dangerous was up, don't continue
            return

        print(
            f"forward {self.distance_forward()} left {self.distance_left()} right {self.distance_right()}"
        )

        # don't do anything if the manual override is triggered
        if self.sensor_measurements["manual_control"]:
            return

        if (
            not self.parking_break
            and not self.started
            and self.state == ExecutionState.PROGRAM_INIT
        ):
            print("Initial launch")
            self.state = ExecutionState.MOVE_FORWARD
            self.set_plane_velocity([0, self.speed])
            self.started = True

        if self.state == ExecutionState.PROGRAM_CONTROL:
            self.state = ExecutionState.SCUTTLE

        if self.state == ExecutionState.SCUTTLE:
            offset = abs(self.distance_right() - self.distance_left())
            if offset < self.CENTER_TOLERANCE:
                self.set_plane_velocity([0, self.speed])
                self.state = ExecutionState.MOVE_FORWARD
            else:
                if self.distance_right() < self.distance_left():
                    self.set_plane_velocity([self.speed / 6, 0])
                elif self.distance_right() > self.distance_left():
                    self.set_plane_velocity([-self.speed / 6, 0])

        if self.state == ExecutionState.MOVE_FORWARD:
            if self.distance_forward() < self.NEAR_WALL:
                # we're probablt at a corner
                # square up against whichever wall is closest
                self.controller_stop()
                if self.distance_right() < self.distance_left():
                    self.square_up_invert = -1
                    self.square_up_heading = 90
                else:
                    self.square_up_heading = 270
                self.state = ExecutionState.SQUARING_UP
            elif self.distance_forward() < self.SLOW_WALL:
                # if a wall is ahead do everything more slowly
                self.set_plane_velocity([0, self.speed / 2])
