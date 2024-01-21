#!/usr/bin/env python3
from enum import Enum


class ObjectType(Enum):
    """Anything that could be in the world"""

    UNKNOWN = 0
    ROBOT = 1
    ZONE = 2
    BARREL = 4
    LINE = 5
    WALL = 6
    MINE = 7

    def __str__(self):
        """Return a string representation of the ObjectType"""

        return {
            self.UNKNOWN: "U",
            self.ROBOT: "R",
            self.ZONE: "Z",
            self.BARREL: "B",
            self.LINE: "L",
            self.WALL: "W",
            self.MINE: "M",
        }[self]
