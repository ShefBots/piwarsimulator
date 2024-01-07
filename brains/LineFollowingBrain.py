#!/usr/bin/env python3
from time import time
from world.WorldObject import *
from world.ObjectType import *
from brains.RobotBrain import RobotBrain


class LineFollowingBrain(RobotBrain):
    """logic for the line following challenge"""

    ANGLE_TOLERANCE = 2  # try and be pointing towards the line
    NEAR_WALL = 0.1

    def __init__(self, **kwargs):
        super(LineFollowingBrain, self).__init__(**kwargs)
        self.last_reading = time()

    def process(self):
        """do the basic brain stuff then do specific line following things"""

        # check sensors and stop if collision is imminent
        super().process()
        # don't do anything if the manual override is triggered
        if self.sensor_measurements["manual_control"]:
            return

        # find something to move towards
        (goal, goal_distance) = self.find_goal()
        if goal is None:
            if self.controller.moving:
                self.controller.stop()
            return

        # if a wall is ahead do everything more slowly
        if self.distance_forward() is None or self.distance_forward() > 0.5:
            speed_modifier = 1
        elif self.distance_forward() < 0.3:
            speed_modifier = 0.2
        else:
            speed_modifier = 0.2 + 0.8 * (self.distance_forward() - 0.3) / 0.2
            print(
                f"dist: {self.distance_forward()} m, speed modifier: {speed_modifier}"
            )
        forward_vel = 0
        side_vel = 0

        # turn towards the line
        if goal.heading > self.ANGLE_TOLERANCE:
            self.controller.set_angular_velocity(self.turning_speed * speed_modifier)
        elif goal.heading < -self.ANGLE_TOLERANCE:
            self.controller.set_angular_velocity(-self.turning_speed * speed_modifier)
        else:
            self.controller.set_angular_velocity(0)

        # find the far end of the detected line
        far_end = Point(0, 0)
        for i in goal.outline.coords:
            i = Point(i)
            if self.TheWorld[0].get_distance(i) > self.TheWorld[0].get_distance(
                far_end
            ):
                far_end = i

        if far_end.y > self.robot.height / 2:
            forward_vel = self.speed * speed_modifier

            # if a wall is to either side move away (for if we're horribly not square)
            if not self.distance_left() is None and self.distance_left() < self.NEAR_WALL:
                side_vel = self.speed * speed_modifier
            elif not self.distance_right() is None and self.distance_right() < self.NEAR_WALL:
                side_vel = -self.speed * speed_modifier
        else:
            if far_end.x < 0:
                side_vel = self.speed * speed_modifier
            else:
                side_vel = -self.speed * speed_modifier

        self.controller.set_plane_velocity([side_vel, forward_vel])

    def find_goal(self):
        """find the closest LINE"""
        return self.find_closest(ObjectType.LINE)
