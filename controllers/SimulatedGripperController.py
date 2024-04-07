#!/usr/bin/env python3
import time
from shapely.affinity import rotate
from shapely.affinity import scale
from shapely.geometry import Polygon
from threading import Thread
from controllers.Controller import Controller
from world.ObjectType import *


class SimulatedGripperController(Controller, Thread):
    """move the gripper in simulation"""

    UPDATE_RATE = 1 / 120.0

    # note real gripper is 50 degrees opening to parallel with robot side
    # simulated gripper doesn't have quite the same geometry, so it opens
    # to 45 degrees to also be parallel when open
    GRIPPER_ANGLE_OPEN = 45
    GRIPPER_ANGLE_CLOSED = 0
    GRIPPER_SPEED = 45  # degrees / s

    # these should match the definitions in
    # https://github.com/ShefBots/io-controller-micropython/blob/main/main.py#L61
    GRIPPER_CLOSED = 0
    GRIPPER_OPEN = 1
    GRIPPER_UNKOWN = 2
    GRIPPER_CLOSING = 3
    GRIPPER_OPENING = 4

    def __init__(self, robot, secondary_controller=None):
        if type(self).__name__ == "SimulatedGripperController":
            # avout printing out initialising when a parent class...?
            print("Initialising SimulatedGripperController...")
        # https://stackoverflow.com/questions/13380819/multiple-inheritance-along-with-threading-in-python
        super(SimulatedGripperController, self).__init__()

        assert robot.object_type == ObjectType.ROBOT
        self.robot = robot

        # assume starting with the gripper closed
        self.gripper_angle = 0
        self._gripper_state = self.GRIPPER_CLOSED

        self.half_gripper_closed = Polygon(
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

        self.mirror = secondary_controller  # for sensor_simulation mode
        self.running = False
        # self.start()

    def set_brain(self, robot_brain):
        if self.running == True:
            return
        self.robot_brain = robot_brain
        # self.running = False
        self.start()

    @property
    def gripper_state(self):
        return self._gripper_state

    def generate_outline(self, angle=None):
        if angle is None:
            angle = self.gripper_angle
        half_gripper_ = rotate(
            self.half_gripper_closed,
            angle,
            origin=[
                -self.robot.width / 2,
                self.robot.height / 2,
            ],
        )

        return half_gripper_.union(scale(half_gripper_, xfact=-1, origin=(0, 0)))

    def open_gripper(self):
        """open the gripper"""
        if (
            self._gripper_state == self.GRIPPER_OPEN
            or self._gripper_state == self.GRIPPER_OPENING
        ):
            return
        print("Opening gripper")
        self._gripper_state = self.GRIPPER_OPENING
        self.set_angular_velocity(self.GRIPPER_SPEED)
        if not self.mirror is None:
            self.mirror.open_gripper()

    def close_gripper(self):
        """close the gripper"""
        if (
            self._gripper_state == self.GRIPPER_CLOSED
            or self._gripper_state == self.GRIPPER_CLOSING
        ):
            return
        print("Closing gripper")
        self._gripper_state = self.GRIPPER_CLOSING
        self.set_angular_velocity(-self.GRIPPER_SPEED)
        if not self.mirror is None:
            self.mirror.close_gripper()

    def set_angular_velocity(self, theta):
        """set angular velocity in degrees per second"""
        # +ve to open
        # -ve to open
        assert theta == self.GRIPPER_SPEED or theta == -self.GRIPPER_SPEED
        super().set_angular_velocity(theta)
        if not self.mirror is None:
            self.mirror.set_angular_velocity(theta)

    def set_plane_velocity(self, vel):
        # do nothing
        pass

    def poke(self):
        """tell the hardware we're still using it and check if it's still there"""
        flag = super().poke()
        if not self.mirror is None:
            flag = flag and self.mirror.poke()
        return flag

    def run(self):
        print("Starting simulated gripper thread...")
        self.robot_brain.attachment_outline = self.generate_outline()
        self.running = True
        while self.running == True:
            now = time.monotonic()

            if self.moving:
                self.gripper_angle += self.theta_vel * self.UPDATE_RATE
                self.robot_brain.attachment_outline = self.generate_outline()

                if self.gripper_angle >= self.GRIPPER_ANGLE_OPEN:
                    self._gripper_state = self.GRIPPER_OPEN
                    self.stop()
                elif self.gripper_angle <= self.GRIPPER_ANGLE_CLOSED:
                    self._gripper_state = self.GRIPPER_CLOSED
                    self.stop()

            to_sleep = self.UPDATE_RATE - (time.monotonic() - now)
            if to_sleep > 0:
                time.sleep(to_sleep)

        print("Simulated gripper thread end")

    def stop(self, exiting=False):
        """stop gripper"""
        print("Stopping gripper")
        super().stop(exiting)
        if not self.mirror is None:
            self.mirror.stop()

        if exiting:
            self.running = False
