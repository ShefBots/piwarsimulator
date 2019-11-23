#!/usr/bin/env python3

class WorldObject:
    """Anything that exists in the world"""
    x = 0
    y = 0
    angle = 0

    def __init__(self, *args, **kwargs):
        self.x = kwargs.get('x', 0)
        self.y = kwargs.get('y', 0)
        self.angle = kwargs.get('angle', 0)

    def __str__(self):
        return "Located at %0.3f, %0.3f with a heading of %0.3f" % (self.x, self.y, self.angle)
