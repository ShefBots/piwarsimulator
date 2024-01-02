#!/usr/bin/env python3
import math
import copy

from world.WorldObject import *
from world.ObjectType import *
from brains.RobotBrain import RobotBrain


class MinesweeperBrain(RobotBrain):
    """logic for the minesweeper challenge"""

    FUN_MODE = False  # do interesting looking turns?

    ANGLE_TOLERANCE = 5  # turn in at least this much towards mine
    OVERLAP_TARGET = 0.5  # fraction to overlap mine by before stopping

    def process(self):
        """do the basic brain stuff then do specific ecodisaster things"""

        # check sensors and stop if collision is imminent
        super().process()
        # don't do anything if the manual override is triggered
        if self.sensor_measurements["manual_control"]:
            return

        # find something to move towards
        (goal, goal_distance) = self.find_goal()
        if goal == None:
            self.controller.stop()
            return

        # slow down as we approach the mine
        if self.TheWorld[0].outline.intersects(goal.outline):
            overlap_outline = self.TheWorld[0].outline.intersection(goal.outline)
            overlap = overlap_outline.area / self.TheWorld[0].outline.area
            # print(f"Overlapping mine by {overlap*100:.2f}% (vs {self.OVERLAP_TARGET*100:0.2f}%)")
            speed_modifier = 0.5
        else:
            speed_modifier = 1
            overlap = 0

        if overlap > self.OVERLAP_TARGET and self.controller.moving:
            self.controller.stop()
        elif overlap < self.OVERLAP_TARGET:
            if self.FUN_MODE:
                # turn towards target
                if goal.heading > self.ANGLE_TOLERANCE:
                    self.controller.set_angular_velocity(self.turning_speed)
                elif goal.heading < -self.ANGLE_TOLERANCE:
                    self.controller.set_angular_velocity(-self.turning_speed)
                else:
                    self.controller.set_angular_velocity(0)

            # move sideways towards
            if goal.center[0] > self.TheWorld[0].center[0]:
                sideways_vel = self.speed * speed_modifier
            else:
                sideways_vel = -self.speed * speed_modifier

            # move forwards towards
            if goal.center[1] > self.TheWorld[0].center[1]:
                forward_vel = self.speed * speed_modifier
            else:
                forward_vel = -self.speed * speed_modifier

            self.controller.set_plane_velocity([sideways_vel, forward_vel])

    def find_goal(self):
        """find the closest MINE"""
        closest = None
        closest_distance = 9e99
        for obj in self.TheWorld[1:]:
            dist = self.TheWorld[0].get_distance(obj)
            # only look for a target if we're holding nothing
            if obj.object_type == ObjectType.MINE:
                if dist < closest_distance:
                    closest = obj
                    closest_distance = dist

        return (closest, closest_distance)