#!/usr/bin/env python3
import math
import numpy as np
from pygame import Color
from world.ObjectType import *


class WorldObject:
    """Anything that exists in the world"""

    def __init__(self, **kwargs):
        self.pos = np.array([kwargs.get("x", 0),  kwargs.get("y", 0)])
        self.angle = kwargs.get("angle", 0)
        self.radius = kwargs.get("radius", 0.1)
        self.object_type = kwargs.get("object_type", 0)
        self.color = Color(kwargs.get("color", "white"))
        self.ignore = kwargs.get("ignore", False)
        self.is_held = kwargs.get("is_held", False)
        self.exterior = None  # exterior world version when simulation

    def distance(self, obj=None):
        if obj == None:
            return np.linalg.norm(self.pos)
        else:
            assert isinstance(obj, WorldObject)
            return np.linalg.norm(self.pos - obj.pos)

    def __str__(self):
        return "%s located at %0.3f, %0.3f rotated %0.3f degrees" % (
            self.object_type,
            self.pos[0],
            self.pos[1],
            self.angle,
        )
