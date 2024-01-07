#!/usr/bin/env python3
from enum import Enum


class ExecutionState(Enum):
    """Different things the robot could be doing"""

    STOPPED = 0  # finished program
    NO_CONTROL = 1  # nothing smart started
    SQUARING_UP = 2  # aligning to wall
    PROGRAM_INIT = 3  # something to take of before control
    PROGRAM_CONTROL = 4  # do something smart

    # states for other brains
    MOVE_LEFT = 10
    MOVE_FORWARD = 11
    MOVE_RIGHT = 12

    def __str__(self):
        """Return a string representation of the ExecutionState"""

        return {
            self.STOPPED: "Stopped",
            self.NO_CONTROL: "No Control",
            self.SQUARING_UP: "Squaring Up",
            self.PROGRAM_CONTROL: "Program Control",
            self.MOVE_LEFT: "Moving Left",
            self.MOVE_FORWARD: "Moving Forward",
            self.MOVE_RIGHT: "Moving Right",
        }[self]
