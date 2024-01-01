#!/usr/bin/env python3
import math
import numpy as np
from pygame import Color
from shapely.geometry import LineString, Point, Polygon
from shapely.affinity import rotate
from shapely.affinity import translate
from world.ObjectType import ObjectType


class WorldObject:
    """
    Anything that exists in the world
    Contains a shapely outline for collisions
    DO NOT += -= on @properties
    """

    def __init__(self, **kwargs):
        self.object_type = kwargs.get("object_type", 0)
        self.color = Color(kwargs.get("color", "white"))

        if (
            self.object_type == ObjectType.UNKNOWN
            or self.object_type == ObjectType.BARREL
        ):
            self._center = np.array([kwargs.get("x", 0), kwargs.get("y", 0)])
            self._angle = 0  # construct the objects vertically then rotate

            self.radius = kwargs.get("radius", 0.1)
            # circular objects are points with a buffer (turning them into a polygon)
            self.outline = Point(self._center).buffer(self.radius)

            self.angle = kwargs.get("angle", 0)
        elif (
            self.object_type == ObjectType.ROBOT
            or self.object_type == ObjectType.ZONE
            or self.object_type == ObjectType.MINE
        ):
            self._center = np.array([kwargs.get("x", 0), kwargs.get("y", 0)])
            self._angle = 0  # construct the objects vertically then rotate

            self.width = kwargs.get("w", 0.1)  # width
            self.height = kwargs.get("h", 0.1)  # height
            self.outline = Polygon(
                [
                    (
                        self._center[0] - self.width / 2,
                        self._center[1] - self.height / 2,
                    ),
                    (
                        self._center[0] + self.width / 2,
                        self._center[1] - self.height / 2,
                    ),
                    (
                        self._center[0] + self.width / 2,
                        self._center[1] + self.height / 2,
                    ),
                    (
                        self._center[0] - self.width / 2,
                        self._center[1] + self.height / 2,
                    ),
                ]
            )

            self.angle = kwargs.get("angle", 0)
        elif self.object_type == ObjectType.LINE or self.object_type == ObjectType.WALL:
            if "x1" in kwargs and "y1" in kwargs and "x2" in kwargs and "y2" in kwargs:
                # set wall/line by coordinates

                self.outline = LineString(
                    [(kwargs["x1"], kwargs["y1"]), (kwargs["x2"], kwargs["y2"])]
                )
                self.length = self.outline.length
                self._center = np.array(
                    [self.outline.centroid.x, self.outline.centroid.y]
                )
                self._angle = 90 - math.degrees(
                    math.atan2(kwargs["y2"] - kwargs["y1"], kwargs["x2"] - kwargs["x1"])
                )

            else:
                self._center = np.array([kwargs.get("x", 0), kwargs.get("y", 0)])
                self._angle = 0  # construct the objects vertically then rotate

                self.length = kwargs.get("l", 1)  # length
                self.outline = LineString(
                    [
                        (
                            self._center[0],
                            self._center[1] - self.length / 2,
                        ),
                        (
                            self._center[0],
                            self._center[1] + self.length / 2,
                        ),
                    ]
                )

                self.angle = kwargs.get("angle", 0)

        else:
            raise Exception("unknown object type")

        self.is_held = kwargs.get("is_held", False)
        self.exterior = None  # exterior world version when simulation
        self.heading = 0  # angle relative to the robot (for sensor output)

    @property
    def center(self):
        return self._center

    @center.setter
    def center(self, pos):
        d_pos = pos - self._center
        self.outline = translate(self.outline, d_pos[0], d_pos[1])
        self._center = pos

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, ang):
        d_ang = ang - self._angle
        # angle is negative because we're using clockwise as +ve (as opposed to -ve convention)
        self.outline = rotate(self.outline, -d_ang)
        self._angle = ang

    def angle_to(self):
        """angle between this object and another object"""
        pass

    def get_distance(self, obj=None, relative_to="outline"):
        """distance between this object and 0,0 or another object"""
        if relative_to == "center":
            if obj == None:
                return np.linalg.norm(self._center)
            else:
                assert isinstance(obj, WorldObject)
                return np.linalg.norm(self._center - obj._center)
        elif relative_to == "outline":
            if obj == None:
                print("WARNING: was this supposed to take into account the robot???")
                return self.outline.distance(Point(0, 0))
            else:
                return self.outline.distance(obj.outline)
        else:
            raise Exception("unknown distance comparison method")

    def xy(self):
        """return the xy coordinates of the outline"""
        if isinstance(self.outline, LineString):
            return self.outline.xy
        else:
            return self.outline.exterior.xy

    def __str__(self):
        return "%s centered at %0.3f, %0.3f rotated %0.3f degrees" % (
            self.object_type,
            self._center[0],
            self._center[1],
            self._angle,
        )
