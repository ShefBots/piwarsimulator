#!/usr/bin/env python3
import copy
import math
from WorldObject import *

class ScanObject(WorldObject):
    """Something the robot sees"""

    def __init__(self, *args, **kwargs):
        super().__init__(self, args, kwargs)
        self.distance = kwargs.get('distance', 0)

    def __str__(self):
        return "Located at %0.3f away with a heading of %0.3f degrees" % (self.distance, self.heading)

    # https://stackoverflow.com/questions/15404256/changing-the-class-of-a-python-object-casting
    # https://stackoverflow.com/questions/18020074/convert-a-baseclass-object-into-a-subclass-object-idiomatically/18020180
    # probably super un-pythonic, but that's my java/cpp background for you
    @classmethod
    def DoScan(cls, wo: WorldObject, robot):
        """Copy an WorldObject into a ScanObject,
           then work out where it is in relation to the robot (i.e. scan)"""
        assert isinstance(wo, WorldObject)
        so = copy.deepcopy(wo)
        so.__class__ = cls
        assert isinstance(so, ScanObject)

        # calculate distance and angle
        so.distance = math.sqrt(math.pow(so.x - robot.x, 2) + math.pow(so.y - robot.y, 2))
        if abs(so.x - robot.x) < 1e-1and so.y < robot.y: # special case for tan
            so.heading = -180
        else:
            so.heading = math.degrees(math.atan2(so.x - robot.x, so.y - robot.y))
        so.parent = wo

        return so
