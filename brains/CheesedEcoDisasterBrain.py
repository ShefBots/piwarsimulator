#!/usr/bin/env python3
import math
from shapely.affinity import scale
from shapely.geometry import Point
from brains.ExecutionState import ExecutionState
from brains.RobotBrain import RobotBrain
from world.WorldObject import *
from world.ObjectType import *


# TODO use is_held and add held barrel to TheWorld
# TODO split drop off code from move to zone code
# TODO verify zone colour on drop off?


class CheesedEcoDisasterBrain(RobotBrain):
    """
    logic for the ecodisaster challenge
    find a barrel
    find its zone
    move the barrel to the zone
    rinse and repeat
    """

    # if barrel is this colour aim for that colour zone
    GOAL_MAPPING = {"darkgreen": "blue", "red": "yellow"}

    # how close to be to target before activating gripper
    GRIPPER_ANGLE_TOLERANCE = 2  # degree
    GRIPPER_TOLERANCE = 0.04  # m, if this isn't big bad things happen

    # how close to get to walls before stopping
    LEFT_WALL_TARGET = 0.1  # puts robot on edge of zone where no barrels may be
    RIGHT_WALL_TARGET = 0.1
    FRONT_WALL_TARGET = 0.1
    # how much further than the target is OK
    WALL_MARGIN = 0.02

    # zones for barrel dropping off
    # note these are the zone mid-points
    DROP_ZONES = [(-0.4, "blue"), (0.4, "yellow")]
    # size of the zones
    ZONE_WIDTH = 0.6
    ZONE_HEIGHT = 0.2

    def __init__(self, **kwargs):
        super(CheesedEcoDisasterBrain, self).__init__(**kwargs)
        self.state = ExecutionState.PROGRAM_INIT
        self.do_collision_detection = False  # plow through barrels

        # how close to get to rear wall
        # we will start to increment this at some point
        self.rear_wall_target = 0.04
        # currently don't know where a barrel is
        self.found_barrel = 0

        # is there a barrel dropped off at a specific location?
        self.barrel_positions = np.zeros([2, 6])
        # place to drop off the current barrel
        self.drop_off_x = None
        self.drop_off_y = None
        self.drop_idx = (0, 0)

    def process(self):
        """do the basic brain stuff then do specific ecodisaster things"""

        # check sensors and stop if collision is imminent
        super().process()

        # we're done if all barrel holding positions are filled
        if np.sum(self.barrel_positions) == 12:
            self.controller.stop()
            return

        # some additional sensor processing

        # use time of flight sensor readings to infer drop of zone locations
        # where are we relative to (0,0) if that was the center of the arena?
        tof_front = self.distance_forward()
        tof_rear = self.distance_back()
        tof_right = self.distance_right()
        tof_left = self.distance_left()

        # compatability with old tof measurement system
        if tof_front is None:
            tof_front = 9e99
        if tof_rear is None:
            tof_rear = 9e99
        if tof_right is None:
            tof_right = 9e99
        if tof_left is None:
            tof_left = 9e99

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
            print("Y Localisation failed! (reusing old value, yikes)")
            y = self.y
        else:
            self.y = y
        if x is None:
            print("X Localisation failed! (reusing old value, yikes)")
            x = self.x
        else:
            self.x = x

        # print([x, y])

        # put in estimated target zone locations based on sensor readings
        for xoff, color in self.DROP_ZONES:
            self.TheWorld.append(
                WorldObject(
                    object_type=ObjectType.ZONE,
                    x=xoff - x,
                    y=1.0 - y,
                    w=self.ZONE_WIDTH,
                    h=self.ZONE_HEIGHT,
                    color=color,
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
            elif tof_left > self.LEFT_WALL_TARGET and self.found_barrel == 0:
                # go to bottom left when not holding a barrel
                self.controller.set_plane_velocity([-self.speed, 0])
            elif tof_right > self.RIGHT_WALL_TARGET and self.found_barrel == 1:
                # and bottom right when holding a barrel
                # (so that when dropping off barrels barrels could get pushed left and detected earlier)
                self.controller.set_plane_velocity([self.speed, 0])
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
                    self.rear_wall_target += 0.25
                    self.controller.stop()
                    self.state = ExecutionState.PROGRAM_CONTROL
                    print("Drop it back down and reverse it...")

                self.controller.set_plane_velocity([self.speed, 0])

            goal, goal_distance = self.find_goal()
            if goal is not None:
                self.found_barrel = 1
                print("Barrel found...")
                # if abs(goal.heading) < self.GRIPPER_ANGLE_TOLERANCE:
                if abs(goal.center[0]) < self.GRIPPER_TOLERANCE / 2:
                    self.controller.set_plane_velocity([0, self.speed])

            if self.found_barrel == 1:
                print(goal_distance)
                if goal_distance < self.GRIPPER_TOLERANCE:
                    # barrel is right in front of us (shorter distance goes first)
                    self.controller.stop()
                    self.attachment_controller.close_gripper()
                    if (
                        not self.attachment_controller.gripper_state
                        == self.attachment_controller.GRIPPER_CLOSED
                    ):
                        return
                    self.holding.append(goal)
                    # goal.exterior.is_held = True
                    self.state = (
                        ExecutionState.PROGRAM_CONTROL
                    )  # home, then we move to zone
                elif goal_distance < 0.15:
                    # barrel is closer to in front of us
                    self.controller.set_plane_velocity([0, self.speed / 4])
                    # follw up open call to trigger gripper animation
                    self.attachment_controller.open_gripper()
                elif goal_distance < 0.2:
                    # barrel is close to in front of us
                    self.controller.set_plane_velocity([0, self.speed / 2])
                    self.attachment_controller.open_gripper()

        elif self.state == ExecutionState.MOVE_TO_ZONE:
            if (
                not self.attachment_controller.gripper_state
                == self.attachment_controller.GRIPPER_CLOSED
                and len(self.holding) == 1
            ):
                return
            print("Heading to zone...")
            if (
                tof_front > self.ZONE_HEIGHT + self.FRONT_WALL_TARGET
                and len(self.holding) == 1
            ):  # + gripper size
                # 1) approach up the right hand side wall
                self.controller.set_plane_velocity([0, self.speed])
            elif len(self.holding) == 0:
                if tof_front < self.ZONE_HEIGHT + self.FRONT_WALL_TARGET:
                    if (
                        not self.attachment_controller.gripper_state
                        == self.attachment_controller.GRIPPER_OPEN
                    ):
                        return
                    # 5) no longer holding, go back a bit to back off
                    self.controller.set_plane_velocity([0, -self.speed / 4])
                elif (
                    not self.attachment_controller.gripper_state
                    == self.attachment_controller.GRIPPER_CLOSED
                ):
                    self.controller.stop()
                    self.attachment_controller.close_gripper()
                else:
                    # 6) return to the right
                    if tof_right > self.RIGHT_WALL_TARGET:
                        self.controller.set_plane_velocity([self.speed, 0])
                    else:
                        # 7) done, we should now be able to go back to homing and restart the process
                        self.controller.set_plane_velocity([0, -self.speed])
                        # but we need to clean up state variables first
                        self.drop_off_x = None
                        self.drop_off_y = None
                        self.found_barrel = 0
                        self.state = ExecutionState.PROGRAM_CONTROL
                        return
            else:
                self.find_next_free_barrel_slot(self.get_barrel_color(self.holding[0]))
                if x > self.drop_off_x:
                    # 2) scuttle left to the drop off location
                    self.controller.set_plane_velocity([-self.speed, 0])
                elif y < self.drop_off_y:
                    # 3) at location go forwards
                    self.controller.set_plane_velocity([0, self.speed / 4])
                elif len(self.holding) == 1:
                    # 4) we're there, drop things off
                    self.controller.stop()
                    self.attachment_controller.open_gripper()
                    self.holding.pop(0)
                    self.barrel_positions[self.drop_idx[0], self.drop_idx[1]] = 1

            # TODO split the relevant part of above out into DROP_OFF_BARREL

        elif self.state == ExecutionState.DROP_OFF_BARREL:
            pass

    def estimate_y_front(self, tof):
        # base off front sensor
        if not tof is None and not tof == 9e99:
            # return 1.1 - (tof + self.robot.height / 2)
            # distance forward includes gripper size, need to account for this
            b = self.TheWorld[0].outline.bounds
            return 1.1 - (tof + b[3])
        else:
            return None

    def estimate_y_rear(self, tof):
        # base off rear sensor
        if not tof is None and not tof == 9e99:
            return -1.1 + (tof + self.robot.height / 2)
        else:
            return None

    def estimate_x_right(self, tof):
        # base off right sensor
        if not tof is None and not tof == 9e99:
            return 1.1 - (tof + self.robot.width / 2)
        else:
            return None

    def estimate_x_left(self, tof):
        # base off left sensor
        if not tof is None and not tof == 9e99:
            return -1.1 + (tof + self.robot.width / 2)
        else:
            return None

    def get_barrel_color(self, barrel):
        if barrel.color == Color("darkgreen"):
            return "darkgreen"
        elif barrel.color == Color("red"):
            return "red"

    def find_next_free_barrel_slot(self, barrel_color):
        target_zone_color = self.GOAL_MAPPING[barrel_color]
        for k, tpl in enumerate(self.DROP_ZONES):
            if tpl[1] == target_zone_color:
                break
        print(f"Looking for the {target_zone_color} zone (index {k})")
        _, l = np.shape(self.barrel_positions)
        for ii in range(0, l):
            if self.barrel_positions[k, ii] == 0:
                break
        print(f"Next available slot is {ii}")

        # calculate offset from edge of zone (across & up)
        # (offset into the zone)
        zone_xoff = (ii + 1) * self.ZONE_WIDTH / 7.0
        self.drop_idx = (k, ii)

        # find actual location of zone (check TheWorld)
        goal, _ = self.find_goal()
        if (
            not goal is None
            and goal.object_type == ObjectType.ZONE
            and goal.color == Color(target_zone_color)
        ):
            # t = goal.center - np.array([self.ZONE_WIDTH/2, self.ZONE_HEIGHT/2])
            # set cordinates relative to robot as local variable
            # we need to compare x,y location to these, not to tof readings!
            self.drop_off_x = tpl[0] - self.ZONE_WIDTH / 2 + zone_xoff
            # center of the zone minus half the gripper size (for barrel position) - robot edge to center
            self.drop_off_y = (
                1.1 - self.ZONE_HEIGHT / 2 - 0.1 / 2 - self.TheWorld[0].height / 2
            )
            print(f"Aiming to drop off at {self.drop_off_x}, {self.drop_off_y}")

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
            goal_color = Color(self.GOAL_MAPPING[holding_color])
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
            # return self.find_in_front(ObjectType.BARREL, relative_to=gripper_center)
            # TODO fall back to flosest if there's nothing in front?
            return self.find_closest(ObjectType.BARREL, relative_to=gripper_center)

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
