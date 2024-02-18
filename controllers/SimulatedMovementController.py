#!/usr/bin/env python3
import time
import numpy as np
from shapely.affinity import rotate
from threading import Thread
import util
from controllers.Controller import Controller
from world.ObjectType import *


class SimulatedMovementController(Controller, Thread):
    """move the robot in the exterior the world"""

    UPDATE_RATE = 1 / 120.0
    TRANSLATION_THRESHOLD = 0.0001  # m
    ROTATION_THRESHOLD = 0.001  # degrees

    def __init__(self, robot, secondary_controller=None):
        print("Initialising SimulatedMovementController...")
        # https://stackoverflow.com/questions/13380819/multiple-inheritance-along-with-threading-in-python
        super(SimulatedMovementController, self).__init__()

        # special stuff for simulation
        assert robot.object_type == ObjectType.ROBOT
        self.robot = robot
        self.holding = []  # items to move along with robot
        self.mirror = secondary_controller  # for sensor_simulation mode
        self.running = False
        self.start()

    def set_angular_velocity(self, theta):
        """set angular velocity in degrees per second"""
        super().set_angular_velocity(theta)
        if not self.mirror is None:
            self.mirror.set_angular_velocity(theta)

    def set_plane_velocity(self, vel):
        """velocity aligned to the robot (sideways, forwards)"""
        super().set_plane_velocity(vel)
        if not self.mirror is None:
            self.mirror.set_plane_velocity(vel)

    def ping(self):
        """tell the hardware we're still using it and check if it's still there"""
        flag = super().ping()
        if not self.mirror is None:
            flag = flag and self.mirror.ping()
        return flag

    def run(self):
        print("Starting simulated movement thread...")
        self.running = True
        while self.running == True:
            now = time.monotonic()

            if self.moving:
                rotation = self.theta_vel * self.UPDATE_RATE
                translation = self.vel * self.UPDATE_RATE

                # Need to account for when translating and rotating the x,y coordinate is no longer the centre of rotation
                # Code from @ZodiusInfuser
                # needs to be -rotation in rotate_by to work, I think because of navigational rotation is reversed from mathematical rotation

                # Has rotation?
                if np.abs(rotation) > self.ROTATION_THRESHOLD:
                    # For held item rotation
                    relative_position = self.robot.center

                    # Has translation?
                    if np.linalg.norm(translation) > self.TRANSLATION_THRESHOLD:
                        # Then calculate the instantaneous centre of rotation

                        # This takes the vector perpendicular to the translation and scales it by
                        # a factor inversely proportional to the rotation. The result is a position
                        # either to the left or the right of the translation that the robot will
                        # follow the circle of, with the translation being tangent to that circle
                        path_factor = np.degrees(1 / np.abs(rotation))
                        if rotation > 0.0:
                            centre_of_rotation = (
                                np.array([translation[1], -translation[0]])
                                * path_factor
                            )
                        else:
                            centre_of_rotation = (
                                np.array([-translation[1], translation[0]])
                                * path_factor
                            )

                        # Convert it to be in world coordinates
                        world_centre_of_rotation = (
                            util.rotate_by(centre_of_rotation, -self.robot.angle)
                            + self.robot.center
                        )

                        # Translation step of robot rotatation around the centre of rotation
                        relative_position = self.robot.center - world_centre_of_rotation
                        rotated_position = util.rotate_by(relative_position, -rotation)
                        self.robot.center = rotated_position + world_centre_of_rotation

                        # Translation step of held item rotatation around the centre of rotation
                        for obj in self.holding:
                            relative_position = (
                                obj.exterior.center - world_centre_of_rotation
                            )
                            rotated_position = util.rotate_by(
                                relative_position, -rotation
                            )
                            obj.exterior.center = (
                                rotated_position + world_centre_of_rotation
                            )
                            # Change the held item's heading
                            obj.exterior.angle = obj.exterior.angle + rotation

                    else:
                        # Change held items rotation (rotating about robot's center of rotation)
                        for obj in self.holding:
                            obj.exterior.outline = rotate(
                                obj.exterior.outline,
                                -rotation,  # reversed rotation from coordinate system
                                # origin=relative_position.tolist(),
                                origin=self.robot.outline.centroid,
                            )
                            obj.exterior._angle = obj.exterior._angle + rotation
                            obj.exterior._center = np.array(
                                [
                                    obj.exterior.outline.centroid.x,
                                    obj.exterior.outline.centroid.y,
                                ]
                            )

                    # Change the robot's heading
                    self.robot.angle = self.robot.angle + rotation

                else:
                    # Translate the robot position in its local frame
                    world_translation = util.rotate_by(translation, -self.robot.angle)
                    self.robot.center = world_translation + self.robot.center

                    # Translate held items
                    for obj in self.holding:
                        obj.exterior.center = obj.exterior.center + world_translation

            to_sleep = self.UPDATE_RATE - (time.monotonic() - now)
            if to_sleep > 0:
                time.sleep(to_sleep)

        print("Simulated movement thread end")

    def stop(self, exiting=False):
        """stop moving"""
        print("Stopping moving")
        super().stop(exiting)
        if not self.mirror is None:
            self.mirror.stop()

        if exiting:
            self.running = False
