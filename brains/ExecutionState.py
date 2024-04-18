#!/usr/bin/env python3
from enum import Enum


class ExecutionState(Enum):
    """Different things the robot could be doing"""

    STOPPED = 0  # finished program
    NO_CONTROL = 1  # nothing smart started
    SQUARING_UP = 2  # aligning to wall
    PROGRAM_INIT = 3  # something to take of before control
    PROGRAM_CONTROL = 4  # do something smart
    PROGRAM_COMPLETE = 5  # done
    MANUAL_CONTROL = 6

    # states for other brains
    # maze
    MOVE_LEFT = 10
    MOVE_FORWARD = 11
    MOVE_RIGHT = 12
    FIND_WALL = 13

    # eco disaster
    MOVE_TO_BARREL = 20
    MOVE_TO_ZONE = 21
    DROP_OFF_BARREL = 22

    # line following
    GO_SLOW = 30

    def __str__(self):
        """Return a string representation of the ExecutionState"""

        return {
            self.STOPPED: "ES Stopped",
            self.NO_CONTROL: "ES No Control",
            self.SQUARING_UP: "ES Squaring Up",
            self.PROGRAM_INIT: "ES Program Initialisation",
            self.PROGRAM_CONTROL: "ES Program Control",
            self.PROGRAM_COMPLETE: "ES Program Complete",
            self.MANUAL_CONTROL: "ES Manual Control",
            self.MOVE_LEFT: "ES Moving Left",
            self.MOVE_FORWARD: "ES Moving Forward",
            self.MOVE_RIGHT: "ES Moving Right",
            self.FIND_WALL: "ES Find Wall",
            self.MOVE_TO_BARREL: "ES Moving to Barrel",
            self.MOVE_TO_ZONE: "ES Moving to Zone",
            self.DROP_OFF_BARREL: "ES Dropping off Barrel",
            self.GO_SLOW: "ES Going slow",
        }[self]
