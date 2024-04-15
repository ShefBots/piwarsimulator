#!/usr/bin/env python3
from controllers.Controller import Controller
from world.ObjectType import *

from devices import IOController


class LauncherController(Controller):
    """move the real launcher"""

    def __init__(self, serial_instances):
        print("Initialising LauncherController...")
        super(LauncherController, self).__init__()

        self.io_controller = None

        if IOController.EXPECTED_ID in serial_instances.keys():
            self.io_controller = IOController(
                serial_instances[IOController.EXPECTED_ID]
            )
        else:
            raise Exception("Could not find IO controller hardware")

        self.last_speed = None
        self.last_angle = None
        # toggle to compare to to do firing
        self.last_fire = False

    def set_motor_speed(self, speed):
        """set the speed of the brushless motor, 0 to 1 of range"""
        if not speed == self.last_speed:
            self.last_speed = speed
            # self.io_controller.something(speed)

    def set_tilt(self, angle):
        """set the tilt angle, 0 to 1 of range"""
        if not angle == self.last_angle:
            self.last_angle = angle
            # self.io_controller.something(angle)

    def check_fire(self, fire):
        """handle the toggling of the fire switch"""
        if not fire == self.last_fire:
            self.last_fire = fire
            if fire == True:
                self.fire()
            else:
                self.advance()

    def fire(self):
        """"""
        # self.io_controller.something()
        pass

    def advance(self):
        """"""
        # self.io_controller.something()
        pass
