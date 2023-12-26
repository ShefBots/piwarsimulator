#!/usr/bin/env python3
import time
import numpy as np
from threading import Thread
from controllers.Controller import Controller
import util
from world.ObjectType import *


class SimulatedMovementController(Controller, Thread):
    """controllers are things that move the robot"""

    UPDATE_RATE = 1 / 120.0
    TRANSLATION_THRESHOLD = 0.0001  # m
    ROTATION_THRESHOLD = 0.001  # degrees

    def __init__(self, robot):
        # https://stackoverflow.com/questions/13380819/multiple-inheritance-along-with-threading-in-python
        super(SimulatedMovementController, self).__init__()
        assert robot.object_type == ObjectType.ROBOT
        self.robot = robot

        # default don't move
        self.vel = np.array([0, 0])
        self.theta_vel = 0  # angular velocity

        self.running = False

    def set_angular_velocity(self, theta):
        """set angular velocity in degrees per second"""
        self.theta_vel = theta
        if not self.running:
            self.start()

    def set_plane_velocity(self, vel):
        """velocity aligned to the robot"""
        self.vel = np.array(vel)
        if not self.running:
            self.start()

    def run(self):
        self.running = True
        while self.running == True:
            now = time.time()

            # TODO handle any objects the robot is holding

            rotation = self.theta_vel * self.UPDATE_RATE
            translation = self.vel * self.UPDATE_RATE

            # Need to account for when translating and rotating the x,y coordinate is no longer the centre of rotation
            # Code from @ZodiusInfuser
            # NOTE I think something is funny here, needs to be -rotation in rotate_by to work

            if abs(rotation) > self.ROTATION_THRESHOLD:
                # Has Rotation?
                if np.linalg.norm(translation) > self.TRANSLATION_THRESHOLD:
                    # Has Translation? Then calculate the instantaneous centre of rotation

                    # This takes the vector perpendicular to the translation and scales it by
                    # a factor inversely proportional to the rotation. The result is a position
                    # either to the left or the right of the translation that the robot will
                    # follow the circle of, with the translation being tangent to that circle
                    path_factor = np.degrees(1 / np.abs(rotation))
                    if rotation > 0.0:
                        centre_of_rotation = (
                            np.array([translation[1], -translation[0]]) * path_factor
                        )
                    else:
                        centre_of_rotation = (
                            np.array([-translation[1], translation[0]]) * path_factor
                        )

                    # Convert it to be in world coordinates
                    world_centre_of_rotation = (
                        util.rotate_by(centre_of_rotation, -self.robot.angle)
                        + self.robot.pos
                    )

                    # Rotate the robot position around the centre of rotation
                    relative_position = self.robot.pos - world_centre_of_rotation
                    rotated_position = util.rotate_by(relative_position, -rotation)
                    self.robot.pos = rotated_position + world_centre_of_rotation

                # Change the robot's heading
                self.robot.angle += rotation

            else:
                # Translate the robot position in its local frame
                world_translation = util.rotate_by(translation, self.robot.angle)
                self.robot.pos = world_translation + self.robot.pos

            to_sleep = self.UPDATE_RATE - (time.time() - now)
            if to_sleep > 0:
                time.sleep(to_sleep)

    def stop(self):
        """stop moving"""
        self.vel = np.array([0, 0])
        self.theta_vel = 0
        self.running = False
