#!/usr/bin/env python3
import pygame
import math
from ObjectType import *
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

        self.font = pygame.font.Font(None, self.WorldScale // 10) # default font

        self.frame = 0

    def transform_coordinate(self, c, offset=0):
        return round(c * self.WorldScale + offset)

    # transform horizontal coordiante
    def tW(self, c):
        return self.transform_coordinate(c, 10)

    # transform vertical coordiante
    def tV(self, c):
        return self.transform_coordinate(-c, -10 + self.screen.get_height())

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("Quit requested!")
                self.running = False

        # reset the canvas
        self.screen.fill(Color('black'))

        for row_idx, row in enumerate(self.TheWorld):
            for col_idx, col in enumerate(self.TheWorld[row_idx]):
                # print(i, j)
                obj = self.TheWorld[row_idx][col_idx]
                i = self.tW(row_idx / 10)
                j = self.tV(col_idx / 10)
                # print(i, j)
                r = self.transform_coordinate(obj.radius) // 2
                if obj.object_type == ObjectType.WALL:
                    pygame.gfxdraw.line(self.screen,
                        i - round(r*math.cos(math.radians(obj.angle))),
                        j - round(r*math.sin(math.radians(obj.angle))),
                        i + round(r*math.cos(math.radians(obj.angle))),
                        j + round(r*math.sin(math.radians(obj.angle))), obj.color)
                else:
                    pygame.gfxdraw.filled_circle(self.screen, i, j, r, obj.color)
                    pygame.gfxdraw.aacircle(self.screen, i, j, r, obj.color)

                # render a label on each item in the world
                text = self.font.render(str(obj.object_type), True, Color('orange'))
                text2 = pygame.transform.rotate(text, -obj.angle)
                self.screen.blit(text2, (i - text2.get_width() // 2 + 0.5, j - text2.get_height() // 2 + 2))

        # uncomment to save each frame to make a video
#        pygame.image.save(self.screen, "frames/image%08d.png" % self.frame)

        pygame.display.flip()
        self.frame += 1

        # TODO would be nice if we could display log output on the screen


#    def __del__(self):


    def __str__(self):
        return repr(self.TheWorld)
