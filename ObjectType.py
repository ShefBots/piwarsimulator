#!/usr/bin/env python3
from enum import Enum

class ObjectType(Enum):
    UNKNOWN = 0
    ROBOT = 1
    ZONE = 2
    OBSTACLE = 3
    TARGET = 4
    LINE = 5
    WALL = 6

    def __str__(self):
        """Return a string representation of the ObjectType"""
        return {
            self.UNKNOWN: 'U',
            self.ROBOT: 'R',
            self.ZONE: 'Z',
            self.OBSTACLE: 'O',
            self.TARGET: 'T',
            self.LINE: 'L',
            self.WALL: 'W'
        }[self]
