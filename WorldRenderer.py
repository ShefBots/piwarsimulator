#!/usr/bin/env python3
import pygame
from pygame import Color
from pygame import gfxdraw

class WorldRenderer:
    """Render the world so we can see what the robot is doing!"""
    WorldScale = 200

    def __init__(self, TheWorld):
        self.TheWorld = TheWorld
        self.running = True

        print("Initialising renderer...")

        pygame.init()
        pygame.display.set_caption("piwarsimulator")

        self.screen = pygame.display.set_mode((800, 600))

        self.font = pygame.font.Font(None, self.WorldScale // 7) # default font

    def transform_coordinate(self, c, offset=0):
        return round(c * self.WorldScale + offset)

    # transform horizontal coordiante
    def tW(self, c):
        return self.transform_coordinate(c, self.screen.get_width() / 2)

    # transform vertical coordiante
    def tV(self, c):
        return self.transform_coordinate(-c, self.screen.get_height() / 2)

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("Quit requested!")
                self.running = False

        # reset the canvas
        self.screen.fill(Color('black'))

        for obj in self.TheWorld:
            i = self.tW(obj.x)
            j = self.tV(obj.y)
            r = self.transform_coordinate(obj.radius)
            pygame.gfxdraw.filled_circle(self.screen, i, j, r, obj.color)
            pygame.gfxdraw.aacircle(self.screen, i, j, r, obj.color)

            # render a label on each item in the world
            text = self.font.render(str(obj.object_type), True, Color('orange'))
            text2 = pygame.transform.rotate(text, -obj.angle)
            self.screen.blit(text2, (i - text2.get_width() // 2 + 0.5, j - text2.get_height() // 2 + 2))

        pygame.display.flip()


#    def __del__(self):


    def __str__(self):
        return repr(self.TheWorld)
