#!/usr/bin/env python3
import copy
import math
from shapely.affinity import scale
from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely import lib as shapely_lib
from brains.ExecutionState import ExecutionState
from brains.RobotBrain import RobotBrain
from algorithms.pathfinding_astar import AStar as Pathfinding
from util import fast_translate
from world.WorldObject import *
from world.ObjectType import *


# TODO: let go of barrel
# TODO: what REALLY happens with walls?
# TODO: only ever move orthogonal?
# TODO: sometimes drives through barrels
# TODO: won't stop to collect initial close by barrel
# TODO: sometimes gets stuck


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
        self.close_gripper()

        # for pathfinding, we need to find out if a potential location we move
        # the robot to would intersect with any objects. to do this, we need to
        # translate the location of the robot to each grid location and then to
        # check the intersection with every near-by world object that sensors
        # have deteted. nearby being +- 1 grid spacing distance or so.

        # TODO make these constants?
        # pathfinding grid geometry
        self.pfgrid_scale_factor = 0.08  # each grid space size (in metres)
        # how far out in distance to plan for in grid spaces, ideally this is big enough to always get to goal
        self.pfgrid_size_half = 25
        # over all the grid size MUST BE ODD (centered around 0)
        self.pfgrid_size = self.pfgrid_size_half * 2 + 1

        # for each possible spot on the grid, pre-compute the robot translation with both the closed and open gripper
        self.pfgrid_geometry_closed = np.zeros(
            (self.pfgrid_size, self.pfgrid_size), dtype=object
        )
        self.pfgrid_geometry_open = np.zeros(
            (self.pfgrid_size, self.pfgrid_size), dtype=object
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
        for ii in range(-self.pfgrid_size_half, self.pfgrid_size_half + 1):
            for jj in range(-self.pfgrid_size_half, self.pfgrid_size_half + 1):
                center = (ii * self.pfgrid_scale_factor, jj * self.pfgrid_scale_factor)
                ii2 = ii + self.pfgrid_size_half
                jj2 = jj + self.pfgrid_size_half
                self.pfgrid_centers[ii2, jj2] = center
                self.pfgrid_geometry_closed[ii2, jj2] = fast_translate(
                    temp_robot_outline_clsd, center[0], center[1]
                )
                self.pfgrid_geometry_open[ii2, jj2] = fast_translate(
                    temp_robot_outline_open, center[0], center[1]
                )

        # last calculated pathfinding path
        self.last_path = []
        self.path_refresh = 0

    def process(self):
        """do the basic brain stuff then do specific ecodisaster things"""

        # check sensors and stop if collision is imminent
        super().process()
        # don't do anything if the manual override is triggered
        if self.sensor_measurements["manual_control"]:
            return

        if self.state == ExecutionState.PROGRAM_CONTROL:
            # this should only happen after squaring up
            self.state = ExecutionState.MOVE_TO_ZONE
        elif self.state == ExecutionState.SQUARING_UP:
            # nothing smart to do while squaring up
            return

        # find something to move towards
        (goal, goal_distance) = self.find_goal()
        if goal is None:
            return

        if self.state == ExecutionState.MOVE_TO_BARREL:
            # try to get to the nearest barrel and grab it
            # this assumes there's nothing between the robot and the barrel

            if goal_distance < 0.2:
                self.open_gripper()

            # if in range of barrel
            if goal_distance < self.GRIPPER_TOLERANCE:
                print("Grabbing barrel")
                self.controller.stop()
                self.close_gripper()
                self.holding.append(goal)
                self.state = ExecutionState.MOVE_TO_ZONE

            else:
                # turn towards barrel
                if goal.heading > self.GRIPPER_ANGLE_TOLERANCE:
                    self.controller.set_angular_velocity(self.turning_speed)
                elif goal.heading < -self.GRIPPER_ANGLE_TOLERANCE:
                    self.controller.set_angular_velocity(-self.turning_speed)
                else:
                    self.controller.set_angular_velocity(0)

                if math.fabs(goal.heading) > 10:
                    # so far off we probably need to just turn in place
                    self.controller.set_plane_velocity([0, 0])
                else:
                    # move towards goal
                    self.controller.set_plane_velocity([0, self.speed])

            # TODO if the angles don't match, backup rotate, try to grab again

        elif self.state == ExecutionState.MOVE_TO_ZONE:
            # wait for the gripper to close
            if self.gripper_state == self.GRIPPER_OPEN:
                return

            # if we're close switch to a homing mode to drop off the barrel
            if goal_distance < 0.1:
                print("Close to target drop off zone, switching to drop off mode")
                self.controller.stop()
                self.state = ExecutionState.DROP_OFF_BARREL
                return

            # turn towards zone
            # if goal.heading > self.ZONE_ANGLE_TOLERANCE:
            #     self.controller.set_angular_velocity(self.turning_speed)
            # elif goal.heading < -self.ZONE_ANGLE_TOLERANCE:
            #     self.controller.set_angular_velocity(-self.turning_speed)
            # else:
            #     self.controller.set_angular_velocity(0)

            # PLAN ROUTE BACK TO ZONE
            # general scheme is to break the world into grid (done in init)
            # for each grid space would we colide if we were in it
            # pass onto path finding library to determine a route to the goal
            # try to follow that route

            # things we're trying to avoid and where they are located
            world_obstacles = []
            for obj in self.TheWorld[1:]:
                if (
                    obj.object_type == ObjectType.BARREL
                    or obj.object_type == ObjectType.WALL
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
            check_distance = (self.pfgrid_scale_factor * 2) ** 2
            goal_check_distance = (goal.width + goal.height) ** 2
            for ii in range(0, self.pfgrid_size):
                for jj in range(0, self.pfgrid_size):
                    b = pathfinding_centers[ii, jj]

                    # check if this grid point sits within the goal first so that
                    # obstacles can overwrite if need be
                    # if goal.outline.contains(Point(b)):
                    if (goal.center[0] - b[0]) ** 2 + (
                        goal.center[1] - b[1]
                    ) ** 2 < goal_check_distance:
                        if shapely_lib.intersects(goal.outline, Point(b)):
                            obstacle_map[jj, ii] = Pathfinding.GOAL

                    for obj, oc in world_obstacles:
                        # slightly annoying that writing it out long hand is faster than vectorisation
                        if (oc[0] - b[0]) ** 2 + (oc[1] - b[1]) ** 2 < check_distance:
                            # marginally faster to call shapely's c library directly
                            if shapely_lib.intersects(pfgrid_geometry[ii, jj], obj):
                                # ii and jj swapped for pathfinding library map output
                                obstacle_map[jj, ii] = Pathfinding.OBSTACLE
                                break

            # map holding our current location, it will have the found path added to it
            map = np.zeros_like(obstacle_map)
            # starting location
            map[self.pfgrid_size_half, self.pfgrid_size_half] = Pathfinding.ME

            # do the pathfinding and store the route in newmap
            pf = Pathfinding(obstacle_map, one_goal=False)
            _, state = pf.execute(map)
            print(f"STATE: {state}")

            if state == Pathfinding.ARRIVED:
                print("Pathfinding worked, trying to follow...")

                # on first time store for oscillation detection
                if len(self.last_path) == 0:
                    self.last_path = copy.deepcopy(pf.move_record)

                l_move = self.last_path[-1]  # last move
                n_move = pf.move_record[-1]  # next move

                # check for oscillating movement and don't
                do_next_move = True
                if l_move == n_move:
                    print("Move agreement...")
                else:
                    if l_move == Pathfinding.UP and n_move == Pathfinding.DOWN:
                        do_next_move = False
                    elif l_move == Pathfinding.DOWN and n_move == Pathfinding.UP:
                        do_next_move = False
                    elif l_move == Pathfinding.LEFT and n_move == Pathfinding.RIGHT:
                        do_next_move = False
                    elif l_move == Pathfinding.RIGHT and n_move == Pathfinding.LEFT:
                        do_next_move = False

                if do_next_move == True:
                    if n_move == Pathfinding.UP:
                        print("Trying to move up")
                        self.controller.set_plane_velocity([0, self.speed])
                    elif n_move == Pathfinding.DOWN:
                        print("Trying to move down")
                        self.controller.set_plane_velocity([0, -self.speed])
                    elif n_move == Pathfinding.LEFT:
                        print("Trying to move left")
                        self.controller.set_plane_velocity([-self.speed, 0])
                    elif n_move == Pathfinding.RIGHT:
                        print("Trying to move right")
                        self.controller.set_plane_velocity([self.speed, 0])
                else:
                    print("NO NEW MOVE!!!")

                self.last_path = copy.deepcopy(pf.move_record)

        elif self.state == ExecutionState.DROP_OFF_BARREL:
            pass

            # if a zone drop the barrel off
            # might be better to square up on it?
            # self.square_up_heading = 90
            # self.state = ExecutionState.SQUARING_UP

            # if goal.object_type == ObjectType.ZONE:
            # self.controller.stop()
            # print("Dropping off barrel")
            # self.holding.pop(0)

    def find_goal(self):
        """find the closest TARGET or ZONE depending on execution state"""

        if self.state == ExecutionState.MOVE_TO_ZONE:
            # TODO need to ignore barrels in/near zones
            if self.holding[0].color == Color("darkgreen"):
                holding_color = "darkgreen"
            elif self.holding[0].color == Color("red"):
                holding_color = "red"
            return self.find_closest(
                ObjectType.ZONE, color=self.GOAL_MAPPING[holding_color]
            )
        elif self.state == ExecutionState.MOVE_TO_BARREL:
            # find barrel closest to the front of the robot, 0.025 = barrel radius
            return self.find_closest(
                ObjectType.BARREL, relative_to=(0, self.robot.height / 2 + 0.025)
            )
            # find the closest barrel that's in a straight unobstructed line?
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
