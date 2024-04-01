#!/usr/bin/env python3
import copy
import math
from shapely.geometry import LineString, Polygon
from shapely.affinity import rotate
from sensors.Sensor import Sensor
from world.ObjectType import *
from world.WorldObject import *
from util import fast_translate

from devices import IOController


class DistanceSensor(Sensor):
    """distance from a time of flight sensor turned into a wall"""

    # from VL53L4CD datasheet
    FIELD_OF_VIEW = 18  # degrees
    MAX_RANGE = 1.3  # metres
    # field of view is halved to either side from centreline, but things to
    # that side read further away than they actually are

    def __init__(self, serial_instances, robot, angle):
        super().__init__()
        assert robot.object_type == ObjectType.ROBOT
        print(f"Activating time of flight sensor, pointing at {angle}'")

        # construct a triangle reprsenting the sensor field of view
        # this is relative to the robot's center
        assert angle == 0 or angle == 90 or angle == 180 or angle == 270
        self.angle = angle

        self.io_controller = None

        if IOController.EXPECTED_ID in serial_instances.keys():
            self.io_controller = IOController(serial_instances[IOController.EXPECTED_ID])
        else:
            raise Exception("Could not find IO controller hardware")

        # need to get mounting position on the chassis
        # first distance to check edge for (larger than width or height to make sure it goes beyond)
        l = max(self.robot.width, self.robot.height) * 2
        # find edge
        t = rotate(
            LineString(
                [
                    (
                        self.robot.center[0],
                        self.robot.center[1],
                    ),
                    (
                        self.robot.center[0],
                        self.robot.center[1] + l,
                    ),
                ]
            ),
            -self.angle,  # make scan in direction of sensor
            origin=(
                self.robot.center[0],
                self.robot.center[1],
            ),
        ).intersection(self.robot.outline)
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
            -self.robot.center[0],
            -self.robot.center[1],
        )
        # sensor moutning location for 0,0 pointing north
        self.x0 = self.outline.exterior.coords[0][0]
        self.y0 = self.outline.exterior.coords[0][1]

    def do_scan(self):
        """return the nearest barrel or wall from TheExteriorWorld within the field of view"""

        scan_result = []

        # need to move the field of view outline to the robots locations and rotation
        # rotate first to take advantage of center (-ve because coordiante system)
        self.fov = rotate(self.outline, -self.robot.angle, origin=(0, 0))
        self.fov = fast_translate(
            self.fov,
            self.robot.center[0],
            self.robot.center[1],
        )

        # # if holding something a forward facing sensor won't see anything
        # if len(self.robot_brain.holding) > 0 and self.angle == 0:
        #     # could return unknown object type instead?
        #     return scan_result, {}

        # TODO query the correct sensor
        closest_distance = self.io_controller.read_tof() / 100 # convert cm to m

        # construct the wall the scanned object could be
        if closest_distance > 0:
            scanned_obj = WorldObject(
                object_type=ObjectType.WALL,
                x1=self.x0
                - closest_distance * math.tan(math.radians(self.FIELD_OF_VIEW / 2)),
                y1=self.y0 + closest_distance,
                x2=self.x0
                + closest_distance * math.tan(math.radians(self.FIELD_OF_VIEW / 2)),
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
            
            scan_result.append(scanned_obj)

        return scan_result, {}
