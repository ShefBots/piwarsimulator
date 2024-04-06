#!/usr/bin/env python3
from controllers.Controller import Controller
from world.ObjectType import *
from devices import MotorDriver


class MovementController(Controller):
    """drive the real robot"""

    def __init__(self, serial_instances):
        print("Initialising MovementController...")
        super(MovementController, self).__init__()

        self.motor_driver = None

        if MotorDriver.EXPECTED_ID in serial_instances.keys():
            self.motor_driver = MotorDriver(serial_instances[MotorDriver.EXPECTED_ID])
        else:
            raise Exception("Could not find motor driver hardware")

    def set_angular_velocity(self, theta):
        """set angular velocity in degrees per second"""
        super().set_angular_velocity(theta)
        self.motor_driver.set_angular_velocity(theta)

    def set_plane_velocity(self, vel):
        """velocity aligned to the robot (sideways, forwards)"""
        super().set_plane_velocity(vel)
        self.motor_driver.set_linear_velocities(vel[1], vel[0])

    def poke(self):
        """tell the hardware we're still using it and check if it's still there"""
        flag = super().poke()
        if not self.motor_driver is None:
            # need to check is None because this will still be called on the Exception raised by having no serial
            self.motor_driver.poke()
        return flag

    def stop(self, exiting=False):
        """stop moving"""
        print("Stopping moving")
        super().stop(exiting)
        if not self.motor_driver is None:
            # need to check is None because this will still be called on the Exception raised by having no serial
            self.motor_driver.stop_moving()
