#!/usr/bin/env python3
from time import monotonic as time
from brains.ExecutionState import ExecutionState
from brains.RobotBrain import RobotBrain
from world.WorldObject import *
from world.ObjectType import *


class MinesweeperBrain(RobotBrain):
    """logic for the minesweeper challenge"""

    FUN_MODE = False  # do interesting looking turns?

    ANGLE_TOLERANCE = 5  # turn in at least this much towards mine
    OVERLAP_TARGET = 0.5  # fraction to overlap mine by before stopping
    WALL_TOLERANCE = 0.2  # m how close to walls to get

    MINE_TIMEOUT = 1

    def __init__(self, **kwargs):
        super(MinesweeperBrain, self).__init__(**kwargs)
        # self.state = ExecutionState.PROGRAM_INIT  # we don't get to place it
        self.state = ExecutionState.PROGRAM_CONTROL  # we do
        self.last_mine_found = time()

    def process(self):
        """do the basic brain stuff then do specific minesweeper things"""

        # check sensors and stop if collision is imminent
        if not super().process():
            # parent suggested something dangerous was up, don't continue
            return

        # don't do anything if the manual override is triggered
        if self.sensor_measurements["manual_control"]:
            return

        if self.state == ExecutionState.PROGRAM_WAIT:
            if time() - self.last_state_change_time > self.MINE_TIMEOUT:
                self.state = ExecutionState.PROGRAM_CONTROL
            return

        if self.state == ExecutionState.PROGRAM_COMPLETE:
            return
        elif self.state == ExecutionState.PROGRAM_INIT:
            self.state = ExecutionState.SQUARING_UP
            if not self.distance_forward() is None:
                self.square_up_heading = 0
            elif not self.distance_right() is None:
                self.square_up_heading = 90
            elif not self.distance_left() is None:
                self.square_up_heading = -90
            else:
                self.state = ExecutionState.PROGRAM_CONTROL
            print(f"Squaring up based on {self.square_up_heading}' time of flight")
            return
        elif self.state == ExecutionState.SQUARING_UP:
            # nothing smart to do while squaring up
            return

        # if time() - self.last_mine_found > 30:
        #     print("No mines found for 30 seconds, we're done here")
        #     # self.running = False
        #     self.state = ExecutionState.PROGRAM_COMPLETE
        #     return

        # find something to move towards
        (goal, goal_distance) = self.find_goal()
        if goal is None:
            return

        # slow down as we approach the mine
        if self.TheWorld[0].outline.intersects(goal.outline):
            overlap_outline = self.TheWorld[0].outline.intersection(goal.outline)
            overlap = overlap_outline.area / self.TheWorld[0].outline.area
            # print(f"Overlapping mine by {overlap*100:.2f}% (vs {self.OVERLAP_TARGET*100:0.2f}%)")
            speed_modifier = 0.75
        else:
            speed_modifier = 1
            overlap = 0

        if overlap > self.OVERLAP_TARGET and self._controller.moving:
            self.controller_stop()
            # self.set_plane_velocity([0, 0])
            self.state = ExecutionState.PROGRAM_WAIT
            return
        elif overlap < self.OVERLAP_TARGET:
            if self.FUN_MODE:
                # turn towards target
                if goal.heading > self.ANGLE_TOLERANCE:
                    self.set_angular_velocity(self.turning_speed)
                elif goal.heading < -self.ANGLE_TOLERANCE:
                    self.set_angular_velocity(-self.turning_speed)
                else:
                    self.set_angular_velocity(0)

                # move sideways towards
                if goal.center[0] > self.TheWorld[0].center[0]:
                    side_vel = self.speed * speed_modifier
                else:
                    side_vel = -self.speed * speed_modifier

                # move forwards towards
                if goal.center[1] > self.TheWorld[0].center[1]:
                    forward_vel = self.speed * speed_modifier
                else:
                    forward_vel = -self.speed * speed_modifier

            else:
                # go directly at angle to the target
                forward_vel = (
                    self.speed * speed_modifier * math.cos(math.radians(goal.heading))
                )
                side_vel = (
                    self.speed * speed_modifier * math.sin(math.radians(goal.heading))
                )

            # handle walls so that we zig-zag off them
            if (
                not self.distance_forward() is None
                and forward_vel > 0
                and self.distance_forward() > self.WALL_TOLERANCE - 0.1
                and self.distance_forward() < self.WALL_TOLERANCE
            ):
                forward_vel = -forward_vel
            if (
                not self.distance_left() is None
                and side_vel < 0
                and self.distance_left() > self.WALL_TOLERANCE - 0.1
                and self.distance_left() < self.WALL_TOLERANCE
            ):
                side_vel = -side_vel
            if (
                not self.distance_right() is None
                and side_vel > 0
                and self.distance_right() > self.WALL_TOLERANCE - 0.1
                and self.distance_right() < self.WALL_TOLERANCE
            ):
                side_vel = -side_vel

            self.set_plane_velocity([side_vel, forward_vel])

    def find_goal(self):
        """find the closest MINE"""
        goal = self.find_closest(ObjectType.MINE)
        if goal[0] is None:
            print("Hunting for mines!")
            self.set_plane_velocity([0, 0])
            self.set_angular_velocity(self.turning_speed)
        else:
            self.last_mine_found = time()
            self.set_angular_velocity(0)
        return goal
