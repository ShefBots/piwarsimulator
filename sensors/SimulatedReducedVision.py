#!/usr/bin/env python3
import copy
import math
from pygame import Color
from shapely.geometry import LineString, Polygon
from shapely.affinity import rotate
from sensors.Sensor import Sensor
from world.ObjectType import *
from world.WorldObject import *
from util import rotate_by, fast_translate


class SimulatedReducedVision(Sensor):
    """simulated output from a reduced capability vision system"""

    # the range of what the system sees looking forward
    FIELD_OF_VIEW = 90  # degrees, centered on 0' forwards
    MAX_RANGE = 1.3  # metres

    def __init__(self, ExteriorTheWorld, brain, fov=None):
        super().__init__()
        self.ExteriorTheWorld = ExteriorTheWorld
        self.robot_brain = brain  # needed for holding
        assert ExteriorTheWorld[0].object_type == ObjectType.ROBOT
        print("Activating simulated reduced capability vision sensor")

        self.fov_angle = self.FIELD_OF_VIEW if fov is None else fov
        assert self.fov_angle <= 180

        # need to get mounting position on the chassis
        # first distance to check edge for (larger than width or height to make sure it goes beyond)
        l = max(self.ExteriorTheWorld[0].width, self.ExteriorTheWorld[0].height) * 2
        # find edge
        t = LineString(
            [
                (
                    self.ExteriorTheWorld[0].center[0],
                    self.ExteriorTheWorld[0].center[1],
                ),
                (
                    self.ExteriorTheWorld[0].center[0],
                    self.ExteriorTheWorld[0].center[1] + l,
                ),
            ]
        ).intersection(self.ExteriorTheWorld[0].outline)
        # intersection of sensor and robot outline, this is where the sensor is "mounted"
        x0, y0 = t.coords[1]

        self.outline = Polygon(
            [
                (
                    x0,
                    y0,
                ),
                (
                    x0 - self.MAX_RANGE * math.tan(math.radians(self.fov_angle / 2)),
                    y0 + self.MAX_RANGE,
                ),
                (
                    x0 + self.MAX_RANGE * math.tan(math.radians(self.fov_angle / 2)),
                    y0 + self.MAX_RANGE,
                ),
            ]
        )
        # make relative to a robot at 0,0 pointing north
        self.outline = fast_translate(
            self.outline,
            -self.ExteriorTheWorld[0].center[0],
            -self.ExteriorTheWorld[0].center[1],
        )
        # sensor moutning location for 0,0 pointing north
        self.x0 = self.outline.exterior.coords[0][0]
        self.y0 = self.outline.exterior.coords[0][1]
        assert self.outline.geom_type == "Polygon"  # need to look in a volume

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
            # only inside fov and not something held
            if not self.fov.intersects(obj.outline) or obj.is_held:
                continue
            if (
                # only register barrels, red mines, or white-lines
                obj.object_type == ObjectType.BARREL
                or (obj.object_type == ObjectType.MINE and obj.color == Color("red"))
                or (obj.object_type == ObjectType.LINE and obj.color == Color("white"))
            ):
                if obj.object_type == ObjectType.LINE:
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
                else:
                    dist = self.ExteriorTheWorld[0].get_distance(
                        obj, relative_to=(self.x0, self.y0)
                    )
                    if dist < closest_distance:
                        closest = obj
                        closest_distance = dist

        if not closest is None and closest_line is None:

            # copy the scanned object and then change its coordinate to something relative to the robot
            scanned_obj = copy.deepcopy(closest)
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

        elif not closest_line is None:
            # a line was detected
            # from SimulatedLineOfSight
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
