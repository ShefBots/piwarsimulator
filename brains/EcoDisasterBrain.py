#!/usr/bin/env python3
import copy
import math
from shapely.affinity import scale
from shapely.geometry import Point
from shapely import lib as shapely_lib
from time import monotonic as time
import algorithms.ecodisaster_find as ecodisaster_find
from brains.ExecutionState import ExecutionState
from brains.RobotBrain import RobotBrain
from algorithms.pathfinding_astar import AStar as Pathfinding
from util import fast_translate
from world.WorldObject import *
from world.ObjectType import *


# TODO: back away after letting go of barrel and hunt for the next one
#       excluding barrels in zones
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
    GOAL_MAPPING = {"darkgreen": "blue", "red": "yellow"}

    GRIPPER_ANGLE_TOLERANCE = 2  # degree
    GRIPPER_TOLERANCE = 0.01  # m

    # how aligned to be when heading towards zone
    ZONE_ANGLE_TOLERANCE = 0.5

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
        temp_robot_outline_clsd = temp_robot.outline.union(
            self.attachment_controller.generate_outline(
                angle=self.attachment_controller.GRIPPER_ANGLE_CLOSED
            )
        )
        temp_robot_outline_open = temp_robot.outline.union(
            self.attachment_controller.generate_outline(
                angle=self.attachment_controller.GRIPPER_ANGLE_OPEN
            )
        )
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

        # if there's no beam, set the sensor measurement to false as if there was one not detecting anything
        if not "beam" in self.sensor_measurements.keys():
            # This only gets hit once as the dictionary isn't reset
            self.sensor_measurements["beam"] = False

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
                self.attachment_controller.open_gripper()

            if (
                self.attachment_controller.gripper_state
                == self.attachment_controller.GRIPPER_OPEN
                or self.attachment_controller.gripper_state
                == self.attachment_controller.GRIPPER_OPENING
            ):
                speed_multiplier = 0.25
            else:
                speed_multiplier = 1

            # if in range of barrel
            if (
                goal_distance < self.GRIPPER_TOLERANCE
                or self.sensor_measurements["beam"]
            ):
                print("Grabbing barrel")
                self.controller_stop()
                if (
                    not self.attachment_controller.gripper_state
                    == self.attachment_controller.GRIPPER_OPEN
                    and len(self.holding) == 0
                ):
                    # gripper needs to be open before we can grab it
                    self.controller_stop()
                    return
                self.attachment_controller.close_gripper()
                if not goal.exterior.is_held == True:
                    print("adding barrel to held")
                    self.holding.append(goal)
                self.state = ExecutionState.MOVE_TO_ZONE
                self.time_trying_to_get_to_zone = time()

            else:
                # strafe towards barrel
                if goal.heading > self.GRIPPER_ANGLE_TOLERANCE:
                    self.set_plane_velocity([self.speed * speed_multiplier, 0])
                elif goal.heading < -self.GRIPPER_ANGLE_TOLERANCE:
                    self.set_plane_velocity([-self.speed * speed_multiplier, 0])
                else:

                    if (
                        goal_distance < self.min_distance_to_edge
                        and math.fabs(goal.heading) > 10
                    ):
                        # we're too close to fit in the gripper, backup
                        self.set_plane_velocity([0, -self.speed / 4])
                    elif math.fabs(goal.heading) > 10:
                        # so far off we probably need to just turn in place
                        # self.set_plane_velocity([0, 0])
                        self.set_plane_velocity(
                            [0, self.speed / 10]
                        )  # too easy to dead lock otherwise?
                    else:
                        # move towards goal
                        self.set_plane_velocity([0, self.speed * speed_multiplier])

            # TODO if the angles don't match, backup rotate, try to grab again

        elif self.state == ExecutionState.MOVE_TO_ZONE:
            # wait for the gripper to close
            if (
                not self.attachment_controller.gripper_state
                == self.attachment_controller.GRIPPER_CLOSED
            ):
                self.controller_stop()
                return

            # if we're close switch to a homing mode to drop off the barrel
            if goal_distance < 0.1:
                print("Close to target drop off zone, switching to drop off mode")
                self.controller_stop()
                self.state = ExecutionState.DROP_OFF_BARREL
                return

            # turn towards zone after a few seconds in case we're stuck
            if (
                time() - self.time_trying_to_get_to_zone > 8
                or math.fabs(goal.heading) > 40
            ):
                # 0.5*30 = 15 degrees?
                if goal.heading > self.ZONE_ANGLE_TOLERANCE * 30:
                    self.set_angular_velocity(self.turning_speed / 4)
                elif goal.heading < -self.ZONE_ANGLE_TOLERANCE * 30:
                    self.set_angular_velocity(-self.turning_speed / 4)
                else:
                    self.set_angular_velocity(0)
            else:
                self.set_angular_velocity(0)

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
            if (
                self.attachment_controller.gripper_state
                == self.attachment_controller.GRIPPER_CLOSED
            ):
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

            goal_found = False
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
                            goal_found = True

            if goal_found == False:
                print("ERROR NO GOAL, SKIPPING")
                return

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
                            self.set_plane_velocity(
                                [-self.speed * 0.8, self.speed * 0.8]
                            )
                        elif move_after == Pathfinding.RIGHT:
                            self.set_plane_velocity(
                                [self.speed * 0.8, self.speed * 0.8]
                            )
                        else:
                            # print("Trying to move up")
                            self.set_plane_velocity([0, self.speed])
                    elif next_move == Pathfinding.DOWN:
                        if move_after == Pathfinding.LEFT:
                            self.set_plane_velocity(
                                [-self.speed * 0.8, -self.speed * 0.8]
                            )
                        elif move_after == Pathfinding.RIGHT:
                            self.set_plane_velocity(
                                [self.speed * 0.8, -self.speed * 0.8]
                            )
                        else:
                            # print("Trying to move down")
                            self.set_plane_velocity([0, -self.speed])
                    elif next_move == Pathfinding.LEFT:
                        if move_after == Pathfinding.UP:
                            self.set_plane_velocity(
                                [-self.speed * 0.8, self.speed * 0.8]
                            )
                        elif move_after == Pathfinding.DOWN:
                            self.set_plane_velocity(
                                [-self.speed * 0.8, -self.speed * 0.8]
                            )
                        else:
                            # print("Trying to move left")
                            self.set_plane_velocity([-self.speed, 0])
                    elif next_move == Pathfinding.RIGHT:
                        if move_after == Pathfinding.UP:
                            self.set_plane_velocity(
                                [self.speed * 0.8, self.speed * 0.8]
                            )
                        elif move_after == Pathfinding.DOWN:
                            self.set_plane_velocity(
                                [self.speed * 0.8, -self.speed * 0.8]
                            )
                        else:
                            # print("Trying to move right")
                            self.set_plane_velocity([self.speed, 0])

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
                self.controller_stop()
                # wait for the gripper to open
                self.attachment_controller.open_gripper()
                if (
                    not self.attachment_controller.gripper_state
                    == self.attachment_controller.GRIPPER_OPEN
                ):
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
            self.set_plane_velocity(v)

            pass

            # if a zone drop the barrel off
            # might be better to square up on it?
            # self.square_up_heading = 90
            # self.state = ExecutionState.SQUARING_UP

            # if goal.object_type == ObjectType.ZONE:
            # self.controller_stop()
            # print("Dropping off barrel")
            # self.holding.pop(0)

    def find_goal(self):
        return ecodisaster_find.find_goal(self, use_front=True)
