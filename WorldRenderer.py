#!/usr/bin/env python3
import pygame
from pygame import Color

class WorldRenderer:
    """Render the world so we can see what the robot is doing!"""
    WorldScale = 150

    def __init__(self, TheWorld):
        self.TheWorld = TheWorld
        self.running = True

        print("Initialising renderer...")

        pygame.init()
        pygame.display.set_caption("piwarsimulator")

        self.screen = pygame.display.set_mode((800,600))

    def transformCoordinate(self, c, offset=0):
        return round(c*self.WorldScale+offset)

    # transform horizontal coordiante
    def tW(self, c):
        return self.transformCoordinate(c, self.screen.get_width()/2)

    # transform vertical coordiante
    def tV(self, c):
        return self.transformCoordinate(c, self.screen.get_height()/2)

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("Quit requested!")
                self.running = False

        # reset the canvas
        self.screen.fill(Color('black'))

        for x in self.TheWorld:
            pygame.draw.circle(self.screen, x.color, (self.tW(x.x), self.tV(x.y)), self.transformCoordinate(x.radius))

        pygame.display.flip()


#    def __del__(self):
        

    def __str__(self):
        return repr(self.TheWorld)
