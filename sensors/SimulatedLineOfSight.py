#!/usr/bin/env python3
import copy
import math
from shapely.geometry import LineString, Polygon
from shapely.affinity import rotate
from sensors.Sensor import Sensor
from world.ObjectType import *
from world.WorldObject import *
from util import fast_translate


class SimulatedLineOfSight(Sensor):
    """simulated output from a line of sight (time of flight) sensor"""

    # from VL53L4CD datasheet
    FIELD_OF_VIEW = 18  # degrees
    MAX_RANGE = 1.3  # metres
    # field of view is halved to either side from centreline, but things to
    # that side read further away than they actually are

    def __init__(self, ExteriorTheWorld, brain, angle):
        super().__init__()
        self.ExteriorTheWorld = ExteriorTheWorld
        self.robot_brain = brain  # needed for holding
        assert ExteriorTheWorld[0].object_type == ObjectType.ROBOT
        print(f"Activating simulated time of flight sensor, pointing at {angle}'")

        # guess using previous readings when no new update
        # self.safe_to_guess = True

        # construct a triangle reprsenting the sensor field of view
        # this is relative to the robot's center
        assert angle == 0 or angle == 90 or angle == 180 or angle == 270
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
            -self.angle,  # make scan in direction of sensor
            origin=(
                self.ExteriorTheWorld[0].center[0],
                self.ExteriorTheWorld[0].center[1],
            ),
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
        # make relative to a robot at 0,0 pointing north
        self.outline = rotate(self.outline, -self.angle, origin=(x0, y0))
        self.outline = fast_translate(
            self.outline,
            -self.ExteriorTheWorld[0].center[0],
            -self.ExteriorTheWorld[0].center[1],
        )
        # sensor moutning location for 0,0 pointing north
        self.x0 = self.outline.exterior.coords[0][0]
        self.y0 = self.outline.exterior.coords[0][1]

    def do_scan(self):
        """return the nearest barrel or wall from TheExteriorWorld within the field of view"""

        # need to move the field of view outline to the robots locations and rotation
        # rotate first to take advantage of center (-ve because coordiante system)
        self.fov = rotate(self.outline, -self.ExteriorTheWorld[0].angle, origin=(0, 0))
        self.fov = fast_translate(
            self.fov,
            self.ExteriorTheWorld[0].center[0],
            self.ExteriorTheWorld[0].center[1],
        )

        # # if holding something a forward facing sensor won't see anything
        # if len(self.robot_brain.holding) > 0 and self.angle == 0:
        #     # could return unknown object type instead?
        #     return scan_result, {}

        closest = None
        closest_distance = 9e99

        for obj in self.ExteriorTheWorld[1:]:  # ignore the robot in 0
            if (
                not (
                    obj.object_type
                    == ObjectType.WALL
                    # we'll see over the tops of barrels
                    # or obj.object_type == ObjectType.BARREL
                )
                or obj.is_held
            ):
                continue  # skip non-barrels, non-walls and things being held

            if self.fov.intersects(obj.outline):
                u = self.fov.intersection(obj.outline)
                dist = self.ExteriorTheWorld[0].outline.distance(u)
                if dist < closest_distance:
                    closest = obj
                    closest_distance = dist

        # construct the wall the scanned object could be
        scanned_obj = None
        if closest != None:
            # how long half of the wall segment is
            wall_segment_length = closest_distance * math.tan(
                math.radians(self.FIELD_OF_VIEW / 2)
            )
            # it is a problem if it is < the width of the robot so make it have a minimum
            m = max(self.ExteriorTheWorld[0].width, self.ExteriorTheWorld[0].height)
            if wall_segment_length < m / 2:
                wall_segment_length = m / 2

            scanned_obj = WorldObject(
                object_type=ObjectType.WALL,
                x1=self.x0 - wall_segment_length,
                y1=self.y0 + closest_distance,
                x2=self.x0 + wall_segment_length,
                y2=self.y0 + closest_distance,
                color="lightgray",
            )

            # align it relative to sensor & readjust center
            scanned_obj._angle = self.angle
            scanned_obj.outline = rotate(
                scanned_obj.outline,
                -self.angle,
                origin=(self.x0, self.y0),  # in TheWorld robot is always 0,0
            )
            scanned_obj._center = np.array(
                [scanned_obj.outline.centroid.x, scanned_obj.outline.centroid.y]
            )
            # is there a nicer way to do this using the property setters?

            # because the sensor is always pointing one direction we know it's always has that heading
            scanned_obj.heading = self.angle

            scanned_obj.exterior = closest

        return [scanned_obj], {"tof_" + str(self.angle): closest_distance}
