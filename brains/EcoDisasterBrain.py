#!/usr/bin/env python3
import copy
import math
from shapely.affinity import scale
from shapely.geometry import Polygon
from brains.ExecutionState import ExecutionState
from brains.RobotBrain import RobotBrain
from algorithms.pathfinding_astar import AStar as Pathfinding
from util import fast_translate
from world.WorldObject import *
from world.ObjectType import *


# TODO: logic...
class EcoDisasterBrain(RobotBrain):
    """
    logic for the ecodisaster challenge
    find a barrel
    find its zone
    move the barrel to the zone
    rinse and repeat
    """

    # will need to account for gripper in collisions, this'll probably need a constructor
    # will need to account for zones in collisions

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

            # turn towards zone
            # if goal.heading > self.ZONE_ANGLE_TOLERANCE:
            #     self.controller.set_angular_velocity(self.turning_speed)
            # elif goal.heading < -self.ZONE_ANGLE_TOLERANCE:
            #     self.controller.set_angular_velocity(-self.turning_speed)
            # else:
            #     self.controller.set_angular_velocity(0)

            # PLAN ROUTE BACK TO ZONE
            # general scheme is to break the world into grid
            # for each grid space would we colide if we were in it
            # pass onto path finding library to determine a route to the goal
            # try to follow that route

            # if self.path_refresh == 0: ## only update the path every X loops/time interval?

            # thing we're trying to move
            robot_outline = self.TheWorld[0].outline.union(self.attachment_outline)
            # things we're trying to avoid
            obstacle_outline = Polygon()
            for obj in self.TheWorld[1:]:
                if (
                    obj.object_type == ObjectType.BARREL
                    or obj.object_type == ObjectType.WALL
                ) and not self.is_holding(obj):
                    # could add a buffer around objets here
                    obstacle_outline = obstacle_outline.union(obj.outline)

            # create grid (2d matrix of possible/impossible movement locations) for pathfinding
            scale_factor = 0.1  # each grid space size (in metres)
            grid_half_size = 15  # how far out in distance to plan for in grid spaces, ideally this is big enough to always get to goal
            grid_size = grid_half_size * 2 + 1  # MUST BE ODD (centered around 0)
            obstacle_map = np.zeros((grid_size, grid_size))
            # dummy walls to keep pathfinding from breaking
            obstacle_map[0, :] = Pathfinding.OBSTACLE  # top wall
            obstacle_map[-1, :] = Pathfinding.OBSTACLE  # bottom wall
            obstacle_map[:, 0] = Pathfinding.OBSTACLE  # left wall
            obstacle_map[:, -1] = Pathfinding.OBSTACLE  # right wall

            # for each spot in the grid would the robot hit anything
            for ii in np.arange(-grid_half_size, grid_half_size + 1):
                for jj in np.arange(-grid_half_size, grid_half_size + 1):
                    tro = fast_translate(robot_outline, ii * scale_factor, jj * scale_factor)
                    if tro.intersects(obstacle_outline):
                        obstacle_map[
                            jj + grid_half_size,
                            ii
                            + grid_half_size,  # ii and jj swapped for pathfinding library map output
                        ] = Pathfinding.OBSTACLE
            # where the goal is on the grid, + grid_half_size turn from (0,0) to index
            goal_iijj = (
                np.floor(goal.center / scale_factor).astype(int) + grid_half_size
            )
            obstacle_map[goal_iijj[1], goal_iijj[0]] = Pathfinding.GOAL

            # matrix holding our route to the goal (for visualisation)
            map = np.zeros_like(obstacle_map)
            map[grid_half_size, grid_half_size] = Pathfinding.ME  # starting location

            # do the pathfinding and store the route in newmap
            pf = Pathfinding(obstacle_map)
            newmap, state = pf.execute(map)
            print(f"STATE: {state}")

            if state == Pathfinding.ARRIVED:
                print("Pathfinding worked, trying to follow...")

                # on first time store for oscillation detection
                if len(self.last_path) == 0:
                    self.last_path = copy.deepcopy(pf.move_record)
                    # self.last_path.reverse()

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

            pf.print_map(newmap)
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
            return self.find_closest(ObjectType.BARREL)
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
