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

    # states for other brains
    MOVE_LEFT = 10
    MOVE_FORWARD = 11
    MOVE_RIGHT = 12

    MOVE_TO_BARREL = 20
    MOVE_TO_ZONE = 21
    DROP_OFF_BARREL = 22

    def __str__(self):
        """Return a string representation of the ExecutionState"""

        return {
            self.STOPPED: "Stopped",
            self.NO_CONTROL: "No Control",
            self.SQUARING_UP: "Squaring Up",
            self.PROGRAM_INIT: "Program Initialisation",
            self.PROGRAM_CONTROL: "Program Control",
            self.PROGRAM_COMPLETE: "Program Complete",
            self.MOVE_LEFT: "Moving Left",
            self.MOVE_FORWARD: "Moving Forward",
            self.MOVE_RIGHT: "Moving Right",
            self.MOVE_TO_BARREL: "Moving to Barrel",
            self.MOVE_TO_ZONE: "Moving to Zone",
            self.DROP_OFF_BARREL: "Dropping off Barrel",
        }[self]
