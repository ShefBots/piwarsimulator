#!/usr/bin/env python3
import math
from shapely.affinity import scale
from shapely.geometry import Polygon
from brains.ExecutionState import ExecutionState
from brains.RobotBrain import RobotBrain
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

            # move to the top left most free corner of the zone
            # check distances and pick furthest for squaring up (most likely to be real wall)
            # self.square_up_heading = 90
            # self.state = ExecutionState.SQUARING_UP

            # TODO PLAN ROUTE BACK TO ZONE

            # turn towards barrel (temporary while testing holding)
            if goal.heading > self.GRIPPER_ANGLE_TOLERANCE:
                self.controller.set_angular_velocity(self.turning_speed)
            elif goal.heading < -self.GRIPPER_ANGLE_TOLERANCE:
                self.controller.set_angular_velocity(-self.turning_speed)
            else:
                self.controller.set_angular_velocity(0)
                self.controller.set_plane_velocity([0, 0.3])

            # if a zone drop the barrel off
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
