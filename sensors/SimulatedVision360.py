#!/usr/bin/env python3
import copy
import math
from pygame import Color
from sensors.Sensor import Sensor
from world.ObjectType import *
from util import rotate_by


class SimulatedVision360(Sensor):
    """simulated output from the 360 vision system"""

    def __init__(self, ExteriorTheWorld):
        self.ExteriorTheWorld = ExteriorTheWorld
        assert ExteriorTheWorld[0].object_type == ObjectType.ROBOT
        print("Activating simulated 360 degree vision sensor")

    def do_scan(self):
        """return just the barrels from TheExteriorWorld"""

        scan_result = []

        for obj in self.ExteriorTheWorld[1:]:  # ignore the robot in 0
            if (
                not (
                    obj.object_type == ObjectType.BARREL
                    or obj.object_type == ObjectType.ZONE
                    or (
                        obj.object_type == ObjectType.MINE and obj.color == Color("red")
                    )
                )
                or obj.is_held
            ):
                continue  # skip non-barrels and things being held

            # copy the barrel and then change its coordinate to something relative to the robot
            scanned_obj = copy.deepcopy(obj)
            # barrel.center -= self.ExteriorTheWorld[0].center DOES NOT WORK
            scanned_obj.center = scanned_obj.center - self.ExteriorTheWorld[0].center
            scanned_obj.center = rotate_by(
                scanned_obj.center, self.ExteriorTheWorld[0].angle
            )

            # the direction (angular heading) of the barrel relative to the robot's direction
            # the latter is taken care of by the rotate_by above
            # note y anx x are reversed in the atan2 because of our reversed rotation system
            scanned_obj.heading = math.degrees(
                math.atan2(scanned_obj.center[0], scanned_obj.center[1])
            )

            # rotation of the object relative to the rotation of the robot
            scanned_obj.angle = (
                scanned_obj.angle - self.ExteriorTheWorld[0].angle
            )  # make relative to heading of robot

            scanned_obj.exterior = obj

            scan_result.append(scanned_obj)

        return scan_result, {}
