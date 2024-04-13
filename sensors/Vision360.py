#!/usr/bin/env python3
import math
from sensors.Sensor import Sensor
from world.ObjectType import *
from world.WorldObject import *


class Vision360(Sensor):
    """class for interfacing with the 360 vision system"""

    def __init__(self):
        super().__init__()
        print("Activating connection to the 360 degree vision system")

        ### INTIALISATION CODE HERE ###

    def do_scan(self):
        """
        return list of barrels, zones, and red mines, and the nearest white line ahead as world objects
        """

        ### COMMUNICATE WITH SYSTEM HERE ###

        scan_result = []

        # create objects if they were detected
        if True:

            # create a dummy barrel
            scanned_obj = WorldObject(
                object_type=ObjectType.BARREL,
                x=0.2,
                y=0.2,
                radius=0.025,
                color="darkgreen",
            )

            # angle relative to the robot
            scanned_obj.heading = math.degrees(
                math.atan2(scanned_obj.center[0], scanned_obj.center[1])
            )

            # add it to the list of objects found
            scan_result.append(scanned_obj)

        # return the objects found
        return scan_result, {}
