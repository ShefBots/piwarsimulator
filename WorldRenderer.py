#!/usr/bin/env python3
import pygame

class WorldRenderer:
    """Render the world so we can see what the robot is doing!"""

    def __init__(self, TheWorld):
        self.TheWorld = TheWorld
        self.running = True

        print("Initialising renderer...")

        pygame.init()
        pygame.display.set_caption("piwarsimulator")

        self.screen = pygame.display.set_mode((800,600))

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("Quit requested!")
                self.running = False

        pygame.display.flip()


#    def __del__(self):
        

    def __str__(self):
        return repr(self.TheWorld)
