#!/usr/bin/env python3
from WorldObject import *

class ScanObject(WorldObject):
    """Something the robot sees"""

    def __init__(self, *args, **kwargs):
        WorldObject.__init__(self, args, kwargs)
        self.distance = kwargs.get('distance', 0)
        pass
