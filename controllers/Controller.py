#!/usr/bin/env python3
import numpy as np


class Controller:
    """controllers are things that move the robot"""

    def __init__(self):
        super(Controller, self).__init__()
        # default don't move
        self.vel = np.array([0, 0])
        self.theta_vel = 0  # angular velocity
        self.moving = False

    def set_angular_velocity(self, theta):
        """set angular velocity in degrees per second"""
        self.theta_vel = theta
        self.moving = True

    def set_plane_velocity(self, vel):
        """velocity aligned to the robot (sideways, forwards)"""
        self.vel = np.array(vel)
        assert len(self.vel) == 2, "plane velocity is always 2 components"
        self.moving = True

    def poke(self):
        """tell the hardware we're still using it and check if it's still there"""
        return True  # default assume "this is fine"

    def stop(self, exiting=False):
        """stop moving"""
        print("Stopping moving")
        self.vel = np.array([0, 0])
        self.theta_vel = 0
        self.moving = False

    def __del__(self):
        self.stop(exiting=True)
