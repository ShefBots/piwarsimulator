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


# TODO: back away after letting go of barrel and hunt for the next one
#       excluding barrels in zones
# TODO: what should happen with walls? use them to align and move orthogonal?
# TODO: occaisionally gets stuck
# TODO: sometimes drives over a barrel dropping off


class EcoDisasterBrain(RobotBrain):
    """
    logic for the ecodisaster challenge
    find a barrel
    find its zone
    move the barrel to the zone
    rinse and repeat
    """

    # if barrel is this colour aim for that colour zone
    GOAL_MAPPING = {"darkgreen": Color("blue"), "red": Color("yellow")}

    GRIPPER_ANGLE_TOLERANCE = 2  # degree
    GRIPPER_TOLERANCE = 0.01  # m

    GRIPPER_CLOSED = 0
    GRIPPER_OPEN = 1

    # how aligned to be when heading towards zone
    ZONE_ANGLE_TOLERANCE = 0.5

    # TODO make these constants?
    # pathfinding grid geometry
    PFGRID_SCALE_FACTOR = 0.08  # each grid space size (in metres)
    # how far out in distance to plan for in grid spaces, ideally this is big enough to always get to goal
    PFGRID_SIZE_HALF = 25
    # over all the grid size MUST BE ODD (centered around 0)
    PFGRID_SIZE = PFGRID_SIZE_HALF * 2 + 1

    def __init__(self, **kwargs):
        super(EcoDisasterBrain, self).__init__(**kwargs)
        self.state = ExecutionState.MOVE_TO_BARREL
        self.do_collision_detection = False  # let the logic here handle it

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
        # self.close_gripper()
        self.open_gripper()

        # for pathfinding, we need to find out if a potential location we move
        # the robot to would intersect with any objects. to do this, we need to
        # translate the location of the robot to each grid location and then to
        # check the intersection with every near-by world object that sensors
        # have deteted. nearby being +- 1 grid spacing distance or so.

        # for each possible spot on the grid, pre-compute the robot translation with both the closed and open gripper
        self.pfgrid_geometry_closed = np.zeros(
            (self.PFGRID_SIZE, self.PFGRID_SIZE), dtype=object
        )
        self.pfgrid_geometry_open = np.zeros(
            (self.PFGRID_SIZE, self.PFGRID_SIZE), dtype=object
        )
        temp_robot = WorldObject(
            object_type=ObjectType.ROBOT,
            x=0,
            y=0,
            w=self.robot.width,
            h=self.robot.height,
        )  # temporary robot at 0,0 for pre-computation
        temp_robot_outline_clsd = temp_robot.outline.union(self.gripper_closed_outline)
        temp_robot_outline_open = temp_robot.outline.union(self.gripper_open_outline)
        # store the center coordinates of each grid location
        self.pfgrid_centers = np.zeros_like(self.pfgrid_geometry_closed, dtype=object)
        # translate the robot for each grid location
        for ii in range(-self.PFGRID_SIZE_HALF, self.PFGRID_SIZE_HALF + 1):
            for jj in range(-self.PFGRID_SIZE_HALF, self.PFGRID_SIZE_HALF + 1):
                center = (ii * self.PFGRID_SCALE_FACTOR, jj * self.PFGRID_SCALE_FACTOR)
                ii2 = ii + self.PFGRID_SIZE_HALF
                jj2 = jj + self.PFGRID_SIZE_HALF
                self.pfgrid_centers[ii2, jj2] = center
                self.pfgrid_geometry_closed[ii2, jj2] = fast_translate(
                    temp_robot_outline_clsd, center[0], center[1]
                )
                self.pfgrid_geometry_open[ii2, jj2] = fast_translate(
                    temp_robot_outline_open, center[0], center[1]
                )

        # in order to turn on spot to reach barrel need to have a minimum clearance to it
        # we'll use the distance to the farthest point of the outline of the robot
        b1 = temp_robot_outline_clsd.bounds
        b2 = temp_robot_outline_open.bounds
        self.min_distance_to_edge = max(
            [
                (b1[0] ** 2 + b1[1] ** 2) ** 0.5,
                (b1[2] ** 2 + b1[3] ** 2) ** 0.5,
                (b2[0] ** 2 + b2[1] ** 2) ** 0.5,
                (b2[2] ** 2 + b2[3] ** 2) ** 0.5,
            ]
        )

        # last calculated pathfinding path
        self.last_path = []
        self.last_path_refresh = 0  # time last path was calculated

        # some timers to switch behaivours if we're stuck
        self.time_trying_to_get_to_zone = 0  # time since grabbed barrel
        self.time_since_good_move = 0  # time since last good move

    def process(self):
        """do the basic brain stuff then do specific ecodisaster things"""

        # check sensors and stop if collision is imminent
        super().process()
        # don't do anything if the manual override is triggered
        if self.sensor_measurements["manual_control"]:
            return

        if self.state == ExecutionState.PROGRAM_COMPLETE:
            # done
            return
        elif self.state == ExecutionState.PROGRAM_CONTROL:
            # this should only happen after squaring up
            self.state = ExecutionState.DROP_OFF_BARREL
        elif self.state == ExecutionState.SQUARING_UP:
            # nothing smart to do while squaring up
            return

        # could try plotting routes to both zones here in order to pick barrel
        # with easiest route back to zone

        # find something to move towards
        (goal, goal_distance) = self.find_goal()
        if goal is None:
            return

        if self.state == ExecutionState.MOVE_TO_BARREL:
            # try to get to the nearest barrel and grab it
            # this assumes there's nothing between the robot and the barrel

            if (
                goal_distance < 0.2
                and math.fabs(goal.heading) < self.GRIPPER_ANGLE_TOLERANCE
            ):
                self.open_gripper()

            # if in range of barrel
            if goal_distance < self.GRIPPER_TOLERANCE:
                print("Grabbing barrel")
                self.controller.stop()
                if (
                    not self.gripper_state == self.GRIPPER_OPEN
                    and len(self.holding) == 0
                ):
                    # gripper needs to be open before we can grab it
                    return
                self.close_gripper()
                self.holding.append(goal)
                if not self.gripper_state == self.GRIPPER_CLOSED:
                    return
                self.state = ExecutionState.MOVE_TO_ZONE
                self.time_trying_to_get_to_zone = time()

            else:
                # strafe towards barrel
                if goal.heading > self.GRIPPER_ANGLE_TOLERANCE:
                    self.controller.set_plane_velocity([self.speed, 0])
                elif goal.heading < -self.GRIPPER_ANGLE_TOLERANCE:
                    self.controller.set_plane_velocity([-self.speed, 0])
                else:

                    if (
                        goal_distance < self.min_distance_to_edge
                        and math.fabs(goal.heading) > 10
                    ):
                        # we're too close to fit in the gripper, backup
                        self.controller.set_plane_velocity([0, -self.speed / 4])
                    elif math.fabs(goal.heading) > 10:
                        # so far off we probably need to just turn in place
                        # self.controller.set_plane_velocity([0, 0])
                        self.controller.set_plane_velocity(
                            [0, self.speed / 10]
                        )  # too easy to dead lock otherwise?
                    else:
                        # move towards goal
                        self.controller.set_plane_velocity([0, self.speed])

            # TODO if the angles don't match, backup rotate, try to grab again

        elif self.state == ExecutionState.MOVE_TO_ZONE:
            # wait for the gripper to close
            if not self.gripper_state == self.GRIPPER_CLOSED:
                return

            # if we're close switch to a homing mode to drop off the barrel
            if goal_distance < 0.1:
                print("Close to target drop off zone, switching to drop off mode")
                self.controller.stop()
                self.state = ExecutionState.DROP_OFF_BARREL
                return

            # turn towards zone after a few seconds in case we're stuck
            if (
                time() - self.time_trying_to_get_to_zone > 8
                or math.fabs(goal.heading) > 40
            ):
                # 0.5*30 = 15 degrees?
                if goal.heading > self.ZONE_ANGLE_TOLERANCE * 30:
                    self.controller.set_angular_velocity(self.turning_speed / 4)
                elif goal.heading < -self.ZONE_ANGLE_TOLERANCE * 30:
                    self.controller.set_angular_velocity(-self.turning_speed / 4)
                else:
                    self.controller.set_angular_velocity(0)
            else:
                self.controller.set_angular_velocity(0)

            # PLAN ROUTE BACK TO ZONE
            # general scheme is to break the world into grid (done in init)
            # for each grid space would we colide if we were in it
            # pass onto path finding library to determine a route to the goal
            # try to follow that route

            # only recalculate a few times a second as it's very expensive
            if time() - self.last_path_refresh < 0.1:
                print("skipping new path")
                return

            # things we're trying to avoid and where they are located
            world_obstacles = []
            for obj in self.TheWorld[1:]:
                if (
                    obj.object_type == ObjectType.BARREL
                    or obj.object_type == ObjectType.WALL
                    # stay out of not destination zone
                    or (obj.object_type == ObjectType.ZONE and not obj is goal)
                ) and not self.is_holding(obj):
                    # could add a buffer around objets here, obj.outline.buffer(0.02)
                    world_obstacles.append((obj.outline.buffer(0.02), obj.center))

            # the pre-computed robot locations that we could be moving to on our way to the goal
            if self.gripper_state == self.GRIPPER_CLOSED:
                pfgrid_geometry = self.pfgrid_geometry_closed
            else:
                pfgrid_geometry = self.pfgrid_geometry_open
            # slightly faster to work from local scope
            pathfinding_centers = self.pfgrid_centers

            # map of invalid locations to move to
            obstacle_map = np.zeros_like(pfgrid_geometry)
            # check for obstacles within this distance of the grid location
            # (i.e. with this distance of a spot we're looking at moving to)
            # check_distance = (self.pfgrid_scale_factor * 2) ** 2
            # the robot's outline ends up quite far away from its center, we
            # need to check that entire distance :(
            # this costs like 2 fps to do properly
            check_distance = (
                math.ceil(self.min_distance_to_edge / self.PFGRID_SCALE_FACTOR)
                * self.PFGRID_SCALE_FACTOR
            ) ** 2

            goal_check_distance = (goal.width + goal.height) ** 2
            for ii in range(0, self.PFGRID_SIZE):
                for jj in range(0, self.PFGRID_SIZE):
                    b = pathfinding_centers[ii, jj]

                    # put down obstacles first so goals can override
                    for obj, oc in world_obstacles:
                        # slightly annoying that writing it out long hand is faster than vectorisation
                        if (oc[0] - b[0]) ** 2 + (oc[1] - b[1]) ** 2 < check_distance:
                            # marginally faster to call shapely's c library directly
                            if shapely_lib.intersects(pfgrid_geometry[ii, jj], obj):
                                # ii and jj swapped for pathfinding library map output
                                obstacle_map[jj, ii] = Pathfinding.OBSTACLE
                                break

                    # doing the goal second lets us ignore already deposited barrels
                    # if goal.outline.contains(Point(b)):
                    if (goal.center[0] - b[0]) ** 2 + (
                        goal.center[1] - b[1]
                    ) ** 2 < goal_check_distance:
                        if shapely_lib.intersects(goal.outline, Point(b)):
                            obstacle_map[jj, ii] = Pathfinding.GOAL

            # map holding our current location, it will have the found path added to it
            map = np.zeros_like(obstacle_map)
            # starting location
            map[self.PFGRID_SIZE_HALF, self.PFGRID_SIZE_HALF] = Pathfinding.ME

            # do the pathfinding and store the route in newmap
            pf = Pathfinding(obstacle_map, one_goal=False)
            _, state = pf.execute(map)
            # newmap, state = pf.execute(map)
            # pf.print_map(newmap)
            self.last_path_refresh = time()

            if state == Pathfinding.ARRIVED:

                # on first time store for oscillation detection
                if len(self.last_path) == 0:
                    self.last_path = copy.deepcopy(pf.move_record)

                last_move = self.last_path[-1]  # last move
                next_move = pf.move_record[-1]  # next move
                if len(pf.move_record) > 1:
                    # the move after that (for diagonals)
                    move_after = pf.move_record[-2]
                else:
                    move_after = None

                # check for oscillating movement and don't
                do_next_move = True
                if not last_move == next_move:
                    if last_move == Pathfinding.UP and next_move == Pathfinding.DOWN:
                        do_next_move = False
                    elif last_move == Pathfinding.DOWN and next_move == Pathfinding.UP:
                        do_next_move = False
                    elif (
                        last_move == Pathfinding.LEFT and next_move == Pathfinding.RIGHT
                    ):
                        do_next_move = False
                    elif (
                        last_move == Pathfinding.RIGHT and next_move == Pathfinding.LEFT
                    ):
                        do_next_move = False

                if do_next_move == True:
                    self.time_since_good_move = time()
                    if next_move == Pathfinding.UP:
                        if move_after == Pathfinding.LEFT:
                            self.controller.set_plane_velocity(
                                [-self.speed * 0.8, self.speed * 0.8]
                            )
                        elif move_after == Pathfinding.RIGHT:
                            self.controller.set_plane_velocity(
                                [self.speed * 0.8, self.speed * 0.8]
                            )
                        else:
                            # print("Trying to move up")
                            self.controller.set_plane_velocity([0, self.speed])
                    elif next_move == Pathfinding.DOWN:
                        if move_after == Pathfinding.LEFT:
                            self.controller.set_plane_velocity(
                                [-self.speed * 0.8, -self.speed * 0.8]
                            )
                        elif move_after == Pathfinding.RIGHT:
                            self.controller.set_plane_velocity(
                                [self.speed * 0.8, -self.speed * 0.8]
                            )
                        else:
                            # print("Trying to move down")
                            self.controller.set_plane_velocity([0, -self.speed])
                    elif next_move == Pathfinding.LEFT:
                        if move_after == Pathfinding.UP:
                            self.controller.set_plane_velocity(
                                [-self.speed * 0.8, self.speed * 0.8]
                            )
                        elif move_after == Pathfinding.DOWN:
                            self.controller.set_plane_velocity(
                                [-self.speed * 0.8, -self.speed * 0.8]
                            )
                        else:
                            # print("Trying to move left")
                            self.controller.set_plane_velocity([-self.speed, 0])
                    elif next_move == Pathfinding.RIGHT:
                        if move_after == Pathfinding.UP:
                            self.controller.set_plane_velocity(
                                [self.speed * 0.8, self.speed * 0.8]
                            )
                        elif move_after == Pathfinding.DOWN:
                            self.controller.set_plane_velocity(
                                [self.speed * 0.8, -self.speed * 0.8]
                            )
                        else:
                            # print("Trying to move right")
                            self.controller.set_plane_velocity([self.speed, 0])

                else:
                    print("NO NEW MOVE!!!")

                self.last_path = copy.deepcopy(pf.move_record)
            else:
                print("Pathfinding failed!!! :(")

            # time since last side to side when moving front to back?

        elif self.state == ExecutionState.DROP_OFF_BARREL:
            # at this point should be fairly close to the zone and in an area
            # where there aren't going to be any barrels near by
            # 1) rotate to minimise distance between barrel and zone
            # 2) square up against left wall for left zone and right wall for right zone (not done)
            # 3) slowly move towards the zone and when the barrel is entirely in the zone drop it off
            #    this might not work well depending on the 360 system bing able to tell where the zone is
            # 4) back away (not done)
            # 5) go for next barrel (not done)

            # are we in the goal? it counts as long as some part of the barrel is touching the zone
            if goal_distance < 0.005:
                self.controller.stop()
                # wait for the gripper to open
                self.open_gripper()
                if not self.gripper_state == self.GRIPPER_OPEN:
                    return
                self.holding.pop(0)
                self.state = ExecutionState.PROGRAM_COMPLETE
                return

            # strafe to be in line with the zone

            if goal.heading > self.ZONE_ANGLE_TOLERANCE:
                v = [self.speed / 4, self.speed / 8]
            elif goal.heading < -self.ZONE_ANGLE_TOLERANCE:
                v = [-self.speed / 4, self.speed / 8]
            else:
                v = [0, self.speed / 4]

            if self.distance_forward() < 0.10:
                # about to hit the wall with the gripper, don't move forward anymore!!!
                v[1] = 0
            self.controller.set_plane_velocity(v)

            pass

            # if a zone drop the barrel off
            # might be better to square up on it?
            # self.square_up_heading = 90
            # self.state = ExecutionState.SQUARING_UP

            # if goal.object_type == ObjectType.ZONE:
            # self.controller.stop()
            # print("Dropping off barrel")
            # self.holding.pop(0)

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
        self.gripper_state = self.GRIPPER_OPEN
        self.attachment_outline = self.gripper_open_outline

    def close_gripper(self):
        # TODO real hardware
        self.gripper_state = self.GRIPPER_CLOSED
        self.attachment_outline = self.gripper_closed_outline
