#!/usr/bin/env python3
import copy
import math
from pygame import Color
from shapely.geometry import Polygon
from shapely.affinity import rotate
from sensors.Sensor import Sensor
from world.ObjectType import *
from world.WorldObject import *
from util import rotate_by,fast_translate


class SimulatedVision360(Sensor):
    """simulated output from the 360 vision system"""

    # how far around the robot to scan for lines
    # this excludes the robot dimension, so make sure it's big enough
    LINE_DECTION_DISTANCE = 0.2

    def __init__(self, ExteriorTheWorld, brain):
        super().__init__()
        self.ExteriorTheWorld = ExteriorTheWorld
        self.robot_brain = brain  # needed for holding
        assert ExteriorTheWorld[0].object_type == ObjectType.ROBOT
        print("Activating simulated 360 degree vision sensor")

        # guess using previous readings when no new update
        self.safe_to_guess = True

        # detect lines pretty similar to how simulated line of sight works
        # construct a box ahead of the robot where a line may be detected
        # find the closest line in the box, and then return that
        self.outline = rotate(
            Polygon(
                [
                    (
                        self.ExteriorTheWorld[0].center[0] - self.LINE_DECTION_DISTANCE,
                        self.ExteriorTheWorld[0].center[1],
                    ),
                    (
                        self.ExteriorTheWorld[0].center[0] + self.LINE_DECTION_DISTANCE,
                        self.ExteriorTheWorld[0].center[1],
                    ),
                    (
                        self.ExteriorTheWorld[0].center[0] + self.LINE_DECTION_DISTANCE,
                        self.ExteriorTheWorld[0].center[1] + self.LINE_DECTION_DISTANCE,
                    ),
                    (
                        self.ExteriorTheWorld[0].center[0] - self.LINE_DECTION_DISTANCE,
                        self.ExteriorTheWorld[0].center[1] + self.LINE_DECTION_DISTANCE,
                    ),
                ]
            ),
            -ExteriorTheWorld[0].angle,
            origin=(
                self.ExteriorTheWorld[0].center[0],
                self.ExteriorTheWorld[0].center[1],
            ),
        )
        # subtract the outline of the robot
        self.outline = self.outline.difference(ExteriorTheWorld[0].outline)
        self.outline = rotate(
            self.outline,
            ExteriorTheWorld[0].angle,
            origin=(
                self.ExteriorTheWorld[0].center[0],
                self.ExteriorTheWorld[0].center[1],
            ),
        )
        # make relative to a robot at 0,0 pointing north
        self.outline = fast_translate(
            self.outline,
            -self.ExteriorTheWorld[0].center[0],
            -self.ExteriorTheWorld[0].center[1],
        )
        assert self.outline.geom_type == "Polygon"  # LINE_DECTION_DISTANCE not small

    def do_scan(self):
        """
        return barrels, zones, and red mines from TheExteriorWorld
        also returns the closest white line
        """

        # from SimulatedLineOfSight
        self.fov = rotate(self.outline, -self.ExteriorTheWorld[0].angle, origin=(0, 0))
        self.fov = fast_translate(
            self.fov,
            self.ExteriorTheWorld[0].center[0],
            self.ExteriorTheWorld[0].center[1],
        )

        scan_result = []

        # from SimulatedLineOfSight
        closest = None
        closest_distance = 9e99
        closest_line = None

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
                if (
                    obj.object_type == ObjectType.LINE
                    and obj.color == Color("white")
                    and self.fov.intersects(obj.outline)
                ):
                    # from SimulatedLineOfSight
                    u = self.fov.intersection(obj.outline)
                    if u.geom_type == "LineString":
                        dist = self.ExteriorTheWorld[0].outline.distance(u)
                        if dist < closest_distance:
                            closest = obj
                            closest_distance = dist
                            closest_line = rotate(
                                fast_translate(
                                    u,
                                    -self.ExteriorTheWorld[0].center[0],  # inversed
                                    -self.ExteriorTheWorld[0].center[1],  # inversed
                                ),
                                self.ExteriorTheWorld[0].angle,  # inversed
                                origin=(0, 0),
                            )
                    elif u.geom_type == "MultiLineString":
                        # line is being bisected into two or more parts by the robot
                        # find the first part head of the robot and use that
                        # easiest way to do this is to put the lines back in the robot's
                        # frame of reference and pick something straight ahead
                        u2 = rotate(
                            fast_translate(
                                u,
                                -self.ExteriorTheWorld[0].center[0],  # inversed
                                -self.ExteriorTheWorld[0].center[1],  # inversed
                            ),
                            self.ExteriorTheWorld[0].angle,  # inversed
                            origin=(0, 0),
                        )
                        for u in u2.geoms:
                            if (
                                u.coords[0][1]
                                >= (self.ExteriorTheWorld[0].height / 2 - 0.01)
                                and u.coords[1][1]
                                >= self.ExteriorTheWorld[0].height / 2
                            ):
                                dist = self.ExteriorTheWorld[0].outline.distance(u)
                                if dist < closest_distance:
                                    closest = obj
                                    closest_distance = dist
                                    closest_line = u

                continue  # skip non-barrels, non-zones, non-red mines, non-lines and things being held

            # copy the scanned object and then change its coordinate to something relative to the robot
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

        # a line was detected
        # from SimulatedLineOfSight
        if closest_line != None:
            x1, y1 = closest_line.coords[0]
            x2, y2 = closest_line.coords[1]

            scanned_obj = WorldObject(
                object_type=ObjectType.LINE,
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
                color="white",
            )

            scanned_obj._center = np.array(
                [scanned_obj.outline.centroid.x, scanned_obj.outline.centroid.y]
            )

            # let's have the heading as the centroid of the detected line, probably best compromise
            scanned_obj.heading = math.degrees(
                math.atan2(scanned_obj.center[0], scanned_obj.center[1])
            )

            scanned_obj.exterior = obj
            scan_result.append(scanned_obj)

        return scan_result, {}
