#!/usr/bin/env python3
import copy
import math
from shapely.geometry import LineString, Polygon
from shapely.affinity import rotate
from shapely.affinity import translate
from sensors.Sensor import Sensor
from world.ObjectType import *
from world.WorldObject import *
from util import rotate_by


class SimulatedLineOfSight(Sensor):
    """simulated output from a line of sight (time of flight) sensor"""

    # from VL53L4CD datasheet
    FIELD_OF_VIEW = 18  # degrees
    MAX_RANGE = 1.3  # metres

    def __init__(self, ExteriorTheWorld, angle):
        self.ExteriorTheWorld = ExteriorTheWorld
        assert ExteriorTheWorld[0].object_type == ObjectType.ROBOT
        print(f"Activating simulated time of flight sensor, pointing at {angle}'")

        # construct a triangle reprsenting the sensor field of view
        # this is relative to the robot's center
        self.angle = angle

        # need to get mounting position on the chassis
        # first distance to check edge for (larger than width or height to make sure it goes beyond)
        l = max(self.ExteriorTheWorld[0].width, self.ExteriorTheWorld[0].height) * 2
        # find edge
        t = rotate(
            LineString(
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
            ),
            -self.angle, # make scan in direction of sensor
            origin=(
                self.ExteriorTheWorld[0].center[0],
                self.ExteriorTheWorld[0].center[1],
            ),
        ).intersection(self.ExteriorTheWorld[0].outline)
        x0, y0 = t.coords[1] # intersection of sensor and robot outline
        # the intersection point is where the sensor is "mounted"
        self.dist_to_hull = max(x0, y0)

        self.outline = Polygon(
            [
                (
                    x0,
                    y0,
                ),
                (
                    x0
                    - self.MAX_RANGE * math.tan(math.radians(self.FIELD_OF_VIEW / 2)),
                    y0 + self.MAX_RANGE,
                ),
                (
                    x0
                    + self.MAX_RANGE * math.tan(math.radians(self.FIELD_OF_VIEW / 2)),
                    y0 + self.MAX_RANGE,
                ),
            ]
        )
        self.outline = rotate(
            self.outline,
            -angle,
            origin=(
                self.ExteriorTheWorld[0].center[0],
                self.ExteriorTheWorld[0].center[1],
            ),
        )

    def do_scan(self):
        """return the nearest barrel or wall from TheExteriorWorld within the field of view"""

        # need to move the field of view outline to the robots locations and rotation
        # rotate first to take advantage of center (-ve because coordiante system)
        fov = rotate(self.outline, -self.ExteriorTheWorld[0].angle, origin=(0, 0))
        fov = translate(
            fov, self.ExteriorTheWorld[0].center[0], self.ExteriorTheWorld[0].center[1]
        )

        scan_result = []

        closest = None
        closest_distance = 9e99

        for obj in self.ExteriorTheWorld[1:]:  # ignore the robot in 0
            if (
                not (
                    obj.object_type == ObjectType.BARREL
                    or obj.object_type == ObjectType.WALL
                )
                or obj.is_held
            ):
                continue  # skip non-barrels, non-walls and things being held

            if fov.intersects(obj.outline):
                u = fov.intersection(obj.outline)
                dist = self.ExteriorTheWorld[0].outline.distance(u)
                if dist < closest_distance:
                    closest = obj
                    closest_distance = dist

        closest_distance += self.dist_to_hull

        # construct the wall the scanned object could be
        if closest != None:
            scanned_obj = WorldObject(
                object_type=ObjectType.WALL,
                x1=-closest_distance * math.tan(math.radians(self.FIELD_OF_VIEW / 2)),
                y1=closest_distance,
                x2=closest_distance * math.tan(math.radians(self.FIELD_OF_VIEW / 2)),
                y2=closest_distance,
                color="lightgray",
            )

            # align it relative to sensor & readjust center
            scanned_obj._angle = self.angle
            scanned_obj.outline = rotate(
                scanned_obj.outline,
                -self.angle,
                origin=(0, 0),  # in TheWorld robot is always 0,0
            )
            scanned_obj._center = np.array(
                [scanned_obj.outline.centroid.x, scanned_obj.outline.centroid.y]
            )
            # is there a nicer way to do this using the property setters?

            # because the sensor is always pointing one direction we know it's always has that heading
            scanned_obj.heading = self.angle

            scanned_obj.exterior = obj
            scan_result.append(scanned_obj)

        return scan_result, {}
