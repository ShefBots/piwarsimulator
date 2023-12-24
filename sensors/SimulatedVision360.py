#!/usr/bin/env python3
import copy
import math
from world.ObjectType import *


class SimulatedVision360:
    """simulated output from the 360 vision system"""

    def __init__(self, ExteriorTheWorld):
        self.ExteriorTheWorld = ExteriorTheWorld
        assert(ExteriorTheWorld[0].object_type == ObjectType.ROBOT)

    def do_scan(self):
        """return just the barrels from TheExteriorWorld"""

        scan_result = []

        for obj in self.ExteriorTheWorld[1:]: # ignore the robot in 0
            if not obj.object_type == ObjectType.BARREL or obj.is_held:
                continue  # skip non-barrels and things being held

            # copy the barrel and then change its coordinate to something relative to the robot
            barrel = copy.deepcopy(obj)
            barrel.x -= self.ExteriorTheWorld[0].x
            barrel.y -= self.ExteriorTheWorld[0].y
            barrel.angle = math.degrees(math.atan2(barrel.x, barrel.y))

            scan_result.append(barrel)

        return scan_result
