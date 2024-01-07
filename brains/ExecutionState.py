#!/usr/bin/env python3
from enum import Enum


class ExecutionState(Enum):
    """Different things the robot could be doing"""

    STOPPED = 0  # finished program
    NO_CONTROL = 1  # nothing smart started
    SQUARING_UP = 2  # aligning to wall
    PROGRAM_CONTROL = 3  # generic something smart is happening

    # states for other brains
    MOVE_LEFT = 5
    MOVE_FORWARD = 6
    MOVE_RIGHT = 7

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
