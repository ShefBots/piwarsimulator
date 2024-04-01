#!/usr/bin/env python3
import copy
import math
from shapely.affinity import scale
from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely import lib as shapely_lib
from time import monotonic as time
from brains.ExecutionState import ExecutionState
from brains.RobotBrain import RobotBrain
from algorithms.pathfinding_astar import AStar as Pathfinding
from util import fast_translate
from world.WorldObject import *
from world.ObjectType import *


# TODO use is_held and add held barrel to TheWorld
# TODO pre-programmed target drop-off points
# TODO move to zone & drop off


class CheesedEcoDisasterBrain(RobotBrain):
    """
    logic for the ecodisaster challenge
    find a barrel
    find its zone
    move the barrel to the zone
    rinse and repeat
    """

    # if barrel is this colour aim for that colour zone
    GOAL_MAPPING = {"darkgreen": Color("blue"), "red": Color("yellow")}

    # how close to be to target before activating gripper
    GRIPPER_ANGLE_TOLERANCE = 2  # degree
    GRIPPER_TOLERANCE = 0.025  # m

    GRIPPER_CLOSED = 0
    GRIPPER_OPEN = 1
    GRIPPER_MOVING = 2

    # how close to get to walls before stopping
    LEFT_WALL_TARGET = 0.02
    RIGHT_WALL_TARGET = 0.02
    # how much further than the target is OK
    WALL_MARGIN = 0.02

    def __init__(self, **kwargs):
        super(CheesedEcoDisasterBrain, self).__init__(**kwargs)
        self.state = ExecutionState.PROGRAM_INIT
        self.do_collision_detection = False  # plow through barrels

        half_gripper_closed = Polygon(
            [
                (
                    -self.robot.width / 2,
                    self.robot.height / 2 - 0.002,  # minus a bit so geometry sticks
                ),
                (
                    -0.002,  # minus a bit so that there's always a hole
                    self.robot.height / 2 + 0.1,
                ),
                (
                    -0.002,
                    self.robot.height / 2 + 0.08,
                ),
                (
                    -self.robot.width / 2 + 0.02,
                    self.robot.height / 2 - 0.002,
                ),
            ]
        )

        half_gripper_open = rotate(
            half_gripper_closed,
            45,
            origin=[
                -self.robot.width / 2,
                self.robot.height / 2,
            ],
        )

        self.gripper_closed_outline = half_gripper_closed.union(
            scale(half_gripper_closed, xfact=-1, origin=(0, 0))
        )
        self.gripper_open_outline = half_gripper_open.union(
            scale(half_gripper_open, xfact=-1, origin=(0, 0))
        )
        self.gripper_state = self.GRIPPER_CLOSED
        self.attachment_outline = self.gripper_closed_outline
        # self.close_gripper()
        # self.open_gripper()
        # TODO real hardware init

        # how close to get to rear wall
        # we will start to increment this at some point
        self.rear_wall_target = 0.02
        # currently don't know where a barrel is
        self.found_barrel = 0

    def process(self):
        """do the basic brain stuff then do specific ecodisaster things"""

        # check sensors and stop if collision is imminent
        super().process()

        # some additional sensor processing

        # use time of flight sensor readings to infer drop of zone locations
        # where are we relative to (0,0) if that was the center of the arena?
        tof_front = self.sensor_measurements["tof_0"]
        tof_rear = self.sensor_measurements["tof_180"]
        tof_right = self.sensor_measurements["tof_90"]
        tof_left = self.sensor_measurements["tof_270"]

        yf = self.estimate_y_front(tof_front)
        yr = self.estimate_y_rear(tof_rear)

        xr = self.estimate_x_right(tof_right)
        xl = self.estimate_x_left(tof_left)

        # rely on closest reading of side sensors when close to the other side's wall
        # this avoids issue where false readings from wall edge of wall mess up distances
        x = None
        y = None

        if not yr is None and tof_rear < 0.15:  # and xr > 0.15 and xl > 0.15:
            y = yr
            if not xr is None and not xl is None:
                if tof_left <= tof_right:
                    x = xl
                else:
                    x = xr
            elif not xr is None:
                x = xr
            elif not xl is None:
                x = xl
        elif not yf is None and tof_front < 0.15:  # and xr > 0.15 and xl > 0.15:
            y = yf
            if not xr is None and not xl is None:
                if tof_left <= tof_right:
                    x = xl
                else:
                    x = xr
            elif not xr is None:
                x = xr
            elif not xl is None:
                x = xl

        if not xr is None and tof_right < 0.15:
            x = xr
            if not yf is None and not yr is None:
                if tof_front <= tof_rear:
                    y = yf
                else:
                    y = yr
            elif not yf is None:
                y = yf
            elif not yr is None:
                y = yr
        elif not xl is None and tof_left < 0.15:
            x = xl
            if not yf is None and not yr is None:
                if tof_front <= tof_rear:
                    y = yf
                else:
                    y = yr
            elif not yf is None:
                y = yf
            elif not yr is None:
                y = yr

        if y is None and (not yf is None or not yr is None):
            if not yf is None:
                y = yf
            elif not yr is None:
                y = yr
            else:
                y = (yf + yr) / 2

        if x is None and (not xr is None or not xl is None):
            if not xr is None:
                x = xr
            elif not xl is None:
                x = xl
            else:
                x = (xl + xr) / 2

        if y is None:
            print("Y Localisation failed!")
        if x is None:
            print("X Localisation failed!")

        # print([x, y])

        # put in estimated target zone locations based on sensor readings
        self.TheWorld.append(
            WorldObject(
                object_type=ObjectType.ZONE,
                x=-0.4 - x,
                y=1.0 - y,
                w=0.6,
                h=0.2,
                color="blue",
            )
        )
        self.TheWorld.append(
            WorldObject(
                object_type=ObjectType.ZONE,
                x=0.4 - x,
                y=1.0 - y,
                w=0.6,
                h=0.2,
                color="yellow",
            )
        )

        # don't do anything if the manual override is triggered
        if self.sensor_measurements["manual_control"]:
            return

        if self.state == ExecutionState.PROGRAM_COMPLETE:
            return
        elif self.state == ExecutionState.PROGRAM_INIT:
            # TODO squaring up?
            self.state = ExecutionState.PROGRAM_CONTROL
        elif self.state == ExecutionState.SQUARING_UP:
            # nothing smart to do while squaring up
            return

        # TODO add in distance mesuarement margins?

        if self.state == ExecutionState.PROGRAM_CONTROL:
            print("Homing...")
            # move to back left corner
            if tof_rear < self.rear_wall_target - self.WALL_MARGIN:
                self.controller.set_plane_velocity([0, self.speed])
            elif tof_rear > self.rear_wall_target:
                # and not tof_rear > self.rear_wall_target + self.WALL_MARGIN: # for margin?
                self.controller.set_plane_velocity([0, -self.speed])
            elif tof_left > self.LEFT_WALL_TARGET:
                self.controller.set_plane_velocity([-self.speed, 0])
            else:
                self.controller.stop()
                if self.found_barrel == 0:
                    self.state = ExecutionState.MOVE_TO_BARREL
                elif self.found_barrel == 1:
                    self.state = ExecutionState.MOVE_TO_ZONE

        elif self.state == ExecutionState.MOVE_TO_BARREL:
            # find a barrel to move towards
            if self.found_barrel == 0:
                print("Locating barrel...")
                if tof_right < self.RIGHT_WALL_TARGET:
                    # go forwards and do another scan
                    self.rear_wall_target += 0.05
                    self.controller.stop()
                    self.state = ExecutionState.PROGRAM_CONTROL
                    print("Drop it back down and reverse it...")

                self.controller.set_plane_velocity([self.speed, 0])

            goal, goal_distance = self.find_goal()
            if goal is not None:
                self.found_barrel = 1
                print("Barrel found...")
                if abs(goal.heading) < self.GRIPPER_ANGLE_TOLERANCE:
                    self.controller.set_plane_velocity([0, self.speed])

            if self.found_barrel == 1:
                print(goal_distance)
                if goal_distance < self.GRIPPER_TOLERANCE:
                    # barrel is right in front of us (shorter distance goes first)
                    self.controller.stop()
                    self.close_gripper()
                    if not self.gripper_state == self.GRIPPER_CLOSED:
                        return
                    self.holding.append(goal)
                    # goal.exterior.is_held = True
                    self.state = (
                        ExecutionState.PROGRAM_CONTROL
                    )  # home, then we move to zone
                elif goal_distance < 0.15:
                    # barrel is closer to in front of us
                    self.controller.set_plane_velocity([0, self.speed / 4])
                    self.open_gripper()  # follw up open call to trigger gripper animation
                elif goal_distance < 0.2:
                    # barrel is close to in front of us
                    self.controller.set_plane_velocity([0, self.speed / 2])
                    self.open_gripper()

        elif self.state == ExecutionState.MOVE_TO_ZONE:
            print("Heading to zone...")
            if not self.gripper_state == self.GRIPPER_CLOSED:
                return
            pass

        elif self.state == ExecutionState.DROP_OFF_BARREL:
            pass

    def estimate_y_front(self, tof):
        # base off front sensor
        if not tof == 9e99:
            return 1.1 - (tof + self.robot.height / 2)
        else:
            return None

    def estimate_y_rear(self, tof):
        # base off rear sensor
        if not tof == 9e99:
            return -1.1 + (tof + self.robot.height / 2)
        else:
            return None

    def estimate_x_right(self, tof):
        # base off right sensor
        if not tof == 9e99:
            return 1.1 - (tof + self.robot.width / 2)
        else:
            return None

    def estimate_x_left(self, tof):
        # base off left sensor
        if not tof == 9e99:
            return -1.1 + (tof + self.robot.width / 2)
        else:
            return None

    ### TODO OLD CODE BELOW HERE, WILL NEED OPTIMISATION/SIMPLIFICATION ###

    def find_in_front(self, object_type, color="", exclude=[], relative_to="outline"):
        """
        find something somewhat in front of the robot
        based on find_closest
        note color should be a pygame.color.Color
        exclude a list of WorldObjects
        if relative_to is a tuple or list, treat that as an offset from the center of the robot
        """
        closest = None
        closest_distance = 9e99
        for obj in self.TheWorld[1:]:  # skip the robot and check everything else
            dist = self.TheWorld[0].get_distance(obj, relative_to=relative_to)
            t = obj.center[1]
            if isinstance(relative_to, (tuple, list, np.ndarray)):
                t = obj.center - relative_to
            else:
                raise Exception("can only find closest points in front")
            if (
                obj.object_type == object_type
                and (color == "" or obj.color == color)
                and not self.is_holding(obj)
                and not any(RobotBrain.match_objects(obj, obj2) for obj2 in exclude)
                and t[1] > 0  # this should filter for objects in front
                and abs(t[0]) < 0.25  # only straight ahead
            ):
                if dist < closest_distance:
                    # I'd like to dist > self.TheWorld[0].height/2 but then it looses tracking
                    closest = obj
                    closest_distance = dist

        return (closest, closest_distance)

    def find_goal(self):
        """find the closest TARGET or ZONE depending on execution state"""

        if len(self.holding) > 0:
            barrel = self.holding[0]
            if barrel.color == Color("darkgreen"):
                holding_color = "darkgreen"
            elif barrel.color == Color("red"):
                holding_color = "red"
            else:
                print("yikes shouldn't get here")
            goal_color = self.GOAL_MAPPING[holding_color]
        else:
            goal_color = None
        # treat the middle of the gripper as the center
        gripper_center = (0, self.robot.height / 2 + 0.025)

        if self.state == ExecutionState.MOVE_TO_ZONE:
            # TODO need to ignore barrels in/near zones
            return self.find_closest(ObjectType.ZONE, color=goal_color)

        elif self.state == ExecutionState.MOVE_TO_BARREL:
            # treat the middle of the gripper as the center, so find barrel closest
            # to the front of the robot, 0.025 = barrel radius
            # finds the closest barrel that's in a straight line
            return self.find_in_front(ObjectType.BARREL, relative_to=gripper_center)
            # TODO fall back to flosest if there's nothing in front?
            # return self.find_closest(ObjectType.BARREL, relative_to=gripper_center)

        elif self.state == ExecutionState.DROP_OFF_BARREL:
            # we have a barrel and we're near the zone, find the bit of the zone
            # closet to where we are to put the barrel

            # the zone
            zone, _ = self.find_closest(ObjectType.ZONE, color=goal_color)

            barrel_center_point = Point(barrel.center)
            # the closest part of the zone to the barrel
            zone_intersection_point = zone.outline.exterior.interpolate(
                zone.outline.exterior.project(barrel_center_point)
            )

            # if we're not inside the zone, find a point inside it to aim towards

            # the line between the barrel and edge of zone
            line_to_zone = LineString([barrel_center_point, zone_intersection_point])

            if not zone.outline.intersects(barrel_center_point):
                # barrel is not inside the zone
                # scaled by *2 because scaling both ends
                fact = (barrel.radius * 3 + line_to_zone.length) / line_to_zone.length
                scaled_line = scale(line_to_zone, xfact=fact, yfact=fact)
                drop_point = scaled_line.coords[1]
            elif zone.outline.contains(barrel.outline):
                # the barrel is entirely in the zone, we're done
                return (barrel, 0)
            else:
                # slightly inside the zone
                # trying to find some point slightly inside the zone still
                # but more likely just some point slightly ahead of where we are
                drop_point = line_to_zone.interpolate(
                    line_to_zone.length - barrel.radius * 3
                )
                drop_point = (drop_point.x, drop_point.y)
                # if we're just inside the zone then the interpolated location along
                # the line ends up being the end of the line instead of the correct barrel location
                # so we need to extrapolate instead
                if all(barrel.center == drop_point):
                    fact = (barrel.radius * 3) / line_to_zone.length
                    scaled_line = scale(line_to_zone, xfact=fact, yfact=fact)
                    drop_point = scaled_line.coords[0]

            # temporary world object to move towards
            barrel_drop = WorldObject(
                object_type=ObjectType.DROP_SPOT,
                x=drop_point[0],
                y=drop_point[1],
                radius=barrel.radius,
                color="gray",
            )
            # work out the heading
            # possibly more useful to use the heading between the barrel and
            # drop off point instead of the robot and drop off point?
            offset_drop = drop_point - barrel.center
            barrel_drop.heading = math.degrees(
                # math.atan2(barrel_drop.center[0], barrel_drop.center[1])
                math.atan2(offset_drop[0], offset_drop[1])
            )

            self.TheWorld.append(barrel_drop)
            return (
                barrel_drop,
                self.TheWorld[0].get_distance(
                    barrel_drop,
                    relative_to=gripper_center,
                ),
            )
        else:
            return (None, 9e99)

    def open_gripper(self):
        # TODO real hardware
        if self.gripper_state == self.GRIPPER_CLOSED:
            print("Opening gripper")
            self.gripper_changed = time()
            self.gripper_state = self.GRIPPER_MOVING
        if (
            self.gripper_state == self.GRIPPER_MOVING
            and time() - self.gripper_changed > 1
        ):
            print("Gripper open")
            self.gripper_state = self.GRIPPER_OPEN
            self.attachment_outline = self.gripper_open_outline

    def close_gripper(self):
        # TODO real hardware
        if self.gripper_state == self.GRIPPER_OPEN:
            print("Closing gripper")
            self.gripper_changed = time()
            self.gripper_state = self.GRIPPER_MOVING
        if (
            self.gripper_state == self.GRIPPER_MOVING
            and time() - self.gripper_changed > 1
        ):
            print("Gripper closed")
            self.gripper_state = self.GRIPPER_CLOSED
            self.attachment_outline = self.gripper_closed_outline