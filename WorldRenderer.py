#!/usr/bin/env python3
import pygame

class WorldRenderer:
    """Render the world so we can see what the robot is doing!"""

    def __init__(self, TheWorld):
        self.TheWorld = TheWorld

        pygame.init()
        pygame.display.set_caption("piwarsimulator")

        self.screen = pygame.display.set_mode((800,600))

    def update(self):
        pygame.display.flip()

#    def __del__(self):
        

    def __str__(self):
        return repr(self.TheWorld)
