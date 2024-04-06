#!/usr/bin/env python3
import pygame
from sensors.Sensor import Sensor


class Keyboard(Sensor):
    """keyboard inputs through pygame"""

    def __init__(self, speed, turning_speed):
        super().__init__()
        print("Activating keyboard sensor")
        self.speed = speed
        self.turning_speed = turning_speed

        self.lastg = False
        self.gripper_status = False

        self.lastspace = False
        self.manual_control = False

    def do_scan(self):
        if not pygame.display.get_init():
            return [], {}
        keys = pygame.key.get_pressed()

        forward_vel = 0
        sideways_vel = 0
        angular_vel = 0

        if keys[pygame.K_w] or keys[pygame.K_UP]:
            forward_vel = self.speed
        elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
            forward_vel = -self.speed

        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            sideways_vel = -self.speed
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            sideways_vel = self.speed

        if keys[pygame.K_q]:
            angular_vel = -self.turning_speed
        elif keys[pygame.K_e]:
            angular_vel = self.turning_speed

        if not keys[pygame.K_g] == self.lastg:
            self.lastg = keys[pygame.K_g]
            if self.lastg == True:
                self.gripper_status = not self.gripper_status

        if not keys[pygame.K_SPACE] == self.lastspace:
            self.lastspace = keys[pygame.K_SPACE]
            if self.lastspace == True:
                self.manual_control = not self.manual_control
                print(f"Manual Control: {self.manual_control}")

        if keys[pygame.K_ESCAPE]:
            do_quit = True
        else:
            do_quit = False

        inputs = {
            "forward_vel": forward_vel,
            "sideways_vel": sideways_vel,
            "angular_vel": angular_vel,
            "gripper_toggle": self.gripper_status,
            "manual_control": self.manual_control,
            "do_quit": do_quit,
        }

        return [], inputs
