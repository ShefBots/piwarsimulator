#!/usr/bin/env python3
from enum import Enum


class ExecutionState(Enum):
    """Different things the robot could be doing"""

    STOPPED = 0
    NO_CONTROL = 1
    SQUARING = 2
    PROGRAM_CONTROL = 3
    MOVE_LEFT = 5
    MOVE_FORWARD = 6
    MOVE_RIGHT = 7

    def __str__(self):
        """Return a string representation of the ExecutionState"""

        return {
            self.STOPPED: "Stopped",
            self.NO_CONTROL: "No Control",
            self.SQUARING: "Squaring Up",
            self.PROGRAM_CONTROL: "Program Control",
            self.MOVE_LEFT: "Moving Left",
            self.MOVE_FORWARD: "Moving Forward",
            self.MOVE_RIGHT: "Moving Right",
        }[self]
