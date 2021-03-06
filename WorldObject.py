#!/usr/bin/env python3
from ObjectType import *
from pygame import Color

class WorldObject:
    """Anything that exists in the world"""

    def __init__(self, *args, **kwargs):
        self.x = kwargs.get('x', 0)
        self.y = kwargs.get('y', 0)
        self.angle = kwargs.get('angle', 0)
        self.radius = kwargs.get('radius', 0.1)
        self.object_type = kwargs.get('object_type', 0)
        self.color = Color(kwargs.get('color', 'white'))
        self.ignore = kwargs.get('ignore', False)

    # def __str__(self):
    #     return "Located at %0.3f, %0.3f with rotated %0.3f degrees" % (self.x, self.y, self.angle)
    #
    # def __repr__(self):
    #     return "\nI am a %s with radius %0.3f - %s" % (self.object_type, self.radius, self.__str__())

    def __repr__(self):
        return "X"
