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

        self.font = pygame.font.Font(None, 22) # default font

    def transformCoordinate(self, c, offset=0):
        return round(c*self.WorldScale+offset)

    # transform horizontal coordiante
    def tW(self, c):
        return self.transformCoordinate(c, self.screen.get_width()/2)

    # transform vertical coordiante
    def tV(self, c):
        return self.transformCoordinate(-c, self.screen.get_height()/2)

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("Quit requested!")
                self.running = False

        # reset the canvas
        self.screen.fill(Color('black'))

        for x in self.TheWorld:
            i = self.tW(x.x)
            j = self.tV(x.y)
            pygame.draw.circle(self.screen, x.color, (i, j), self.transformCoordinate(x.radius))

            # render a label on each item in the world
            text = self.font.render(str(x.objecttype), True, Color('orange'))
            self.screen.blit(text, (i - text.get_width() // 2, j - text.get_height() // 2 + 1)) 

        pygame.display.flip()


#    def __del__(self):
        

    def __str__(self):
        return repr(self.TheWorld)
