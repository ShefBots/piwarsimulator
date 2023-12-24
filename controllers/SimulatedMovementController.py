#!/usr/bin/env python3
import time
from threading import Thread
from controllers.Controller import Controller
from world.ObjectType import *


class SimulatedMovementController(Controller, Thread):
    """controllers are things that move the robot"""

    UPDATE_RATE = 1 / 120.0

    def __init__(self, robot):
        super(SimulatedMovementController, self).__init__() # https://stackoverflow.com/questions/13380819/multiple-inheritance-along-with-threading-in-python
        assert robot.object_type == ObjectType.ROBOT
        self.robot = robot

        # default don't move
        self.x_vel = 0  # horizontal velocity
        self.y_vel = 0  # vertical velocity
        self.theta_vel = 0  # angular velocity

        self.running = False

    def set_angular_velocity(self, theta):
        """set angular velocity in degrees per second"""
        self.theta_vel = theta
        self.start()

    def set_plane_velocities(self, x, y):
        self.x_vel = x
        self.y_vel = y
        self.start()

    def run(self):
        self.running = True
        while self.running == True:
            now = time.time()

            self.robot.x += self.x_vel * self.UPDATE_RATE 
            self.robot.y += self.y_vel * self.UPDATE_RATE 
            self.robot.angle += self.theta_vel

            to_sleep = self.UPDATE_RATE - (time.time() - now)
            if to_sleep > 0:
                time.sleep(to_sleep)

    def stop(self):
        """stop moving"""
        self.x_vel = 0
        self.y_vel = 0
        self.theta_vel = 0
        self.running = False
