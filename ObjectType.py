#!/usr/bin/env python3
from enum import Enum

class ObjectType(Enum):
    UNKNOWN = 0
    ROBOT = 1
    ZONE = 2
    OBSTACLE = 3
    TARGET = 4
    LINE = 5
