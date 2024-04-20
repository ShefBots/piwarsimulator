#!/usr/bin/env python3
from time import monotonic as time
from controllers.Controller import Controller
from util import get_io_controller as get_io_controller
from world.ObjectType import *


class LauncherController(Controller):
    """move the real launcher"""

    def __init__(self, serial_instances):
        print("Initialising LauncherController...")
        super(LauncherController, self).__init__()

        self.io_controller = get_io_controller(serial_instances)
        self.turret_power = False

        self.last_speed = None
        self.last_angle = None
        # toggle to compare to to do firing
        self.last_fire = False
        self.last_fire_time = time()

    # def __del__(self):
    #     # turn off launcher power
    #     self.io_controller.power_turret(0)
    #     # (this should end up being called when the software powers off
    #     # anyway from comm disconnection)

    def power_on(self):
        # turn on launcher power
        if not self.turret_power:
            self.io_controller.power_turret(1)
            self.turret_power = True

    def set_motor_speed(self, speed):
        """set the speed of the brushless motor, 0 to 1 of range"""

        # clamp range
        if speed < 0:
            speed = 0
        elif speed > 1:
            speed = 1

        if not speed == self.last_speed:
            self.power_on()  # turn power on if it's not already
            if speed == 0 and not self.last_speed == 0:
                # power down ?
                pass
            elif speed > 0 and self.last_speed == 0:
                # power up ?
                pass

            self.last_speed = speed
            # speed io_controller is expeting should be in the range of 0-10,000
            self.io_controller.turret_speed(int(speed * 10000))

    def set_tilt(self, angle):
        """set the tilt angle, 0 to 1 of range"""

        # clamp range
        if angle < 0:
            angle = 0
        elif angle > 1:
            angle = 1

        if not angle == self.last_angle:
            self.power_on()  # turn power on if it's not already
            self.last_angle = angle
            # angle io_controller is expeting should be in the range of 0-10,000
            self.io_controller.turret_tilt(int(angle * 10000))

    def check_fire(self, fire):
        """handle the toggling of the fire switch"""
        if not fire == self.last_fire:
            self.power_on()  # turn power on if it's not already
            self.last_fire = fire
            if fire == True:
                self.fire()
            else:
                self.advance()

    def fire(self):
        """fire the missles (even if you're le tired)"""
        if time() - self.last_fire_time < 10:
            # don't fire more than once every 10 seconds
            return
        self.power_on()  # turn power on if it's not already
        self.io_controller.fire()
        self.last_fire_time = time()

    def advance(self):
        """"""
        # handled by io_controller state machine
        pass
