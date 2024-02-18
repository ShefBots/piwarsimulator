#!/usr/bin/env python3
import time
import numpy as np
import util
from controllers.Controller import Controller
from world.ObjectType import *
from devices import MotorDriver


# TODO move out common bits to Controller and call via __super__


class MovementController(Controller):
    """drive the real robot"""

    # TODO pass motor driver serial instance
    def __init__(self, serial_instances):
        print("Initialising MovementController...")
        super(MovementController, self).__init__()

        # default don't move
        self.vel = np.array([0, 0])
        self.theta_vel = 0  # angular velocity
        self.moving = False

        self.motor_driver = None

        if MotorDriver.EXPECTED_ID in serial_instances.keys():
            self.motor_driver = MotorDriver(serial_instances[MotorDriver.EXPECTED_ID])
        else:
            raise Exception("Could not find motor driver hardware")

    def set_angular_velocity(self, theta):
        """set angular velocity in degrees per second"""
        self.motor_driver.set_angular_velocity(theta)
        self.theta_vel = theta
        self.moving = True

    def set_plane_velocity(self, vel):
        """velocity aligned to the robot (sideways, forwards)"""
        self.motor_driver.set_linear_velocities(vel[0, vel[1]])
        self.vel = np.array(vel)
        self.moving = True

    def stop(self, exiting=False):
        """stop moving"""
        print("Stopping moving")
        if not self.motor_driver is None:
            # need to check is None because this will still be called on the Exception raised by having no serial
            self.motor_driver.stop_moving()
        self.vel = np.array([0, 0])
        self.theta_vel = 0
        self.moving = False

        if exiting:
            self.running = False
