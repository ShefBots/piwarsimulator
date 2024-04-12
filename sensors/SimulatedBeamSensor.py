#!/usr/bin/env python3
import copy
import math
from shapely.geometry import LineString
from shapely.affinity import rotate
from sensors.Sensor import Sensor
from world.ObjectType import *
from world.WorldObject import *
from util import fast_translate


class SimulatedBeamSensor(Sensor):
    """check if something crosses the beam"""

    def __init__(self, ExteriorTheWorld):
        super().__init__()
        self.ExteriorTheWorld = ExteriorTheWorld
        assert ExteriorTheWorld[0].object_type == ObjectType.ROBOT
        print(f"Activating simulated gripper line sensor'")

        # the line is just in front of the robot and just a little bit in from its width
        # first distance to check edge for (larger than width or height to make sure it goes beyond)
        robot = self.ExteriorTheWorld[0]
        self.outline = LineString(
            [
                (
                    -robot.width / 2 + 0.02,
                    robot.height / 2 + 0.03,
                ),
                (
                    robot.width / 2 - 0.02,
                    robot.height / 2 + 0.03,
                ),
            ]
        )

    def do_scan(self):
        """return the nearest barrel or wall from TheExteriorWorld within the field of view"""

        scan_result = []

        # translate the outline to the ExteriorWorld location of the robot for drawing on screen
        self.fov = rotate(self.outline, -self.ExteriorTheWorld[0].angle, origin=(0, 0))
        self.fov = fast_translate(
            self.fov,
            self.ExteriorTheWorld[0].center[0],
            self.ExteriorTheWorld[0].center[1],
        )

        # Does the beam hit a barrel?
        for obj in self.ExteriorTheWorld[1:]:  # ignore the robot in 0
            if obj.object_type == ObjectType.BARREL:
                if self.fov.intersects(obj.outline):
                    print("Beam hit something!")
                    return [], {"beam": True}

        return [], {"beam": False}
