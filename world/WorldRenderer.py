#!/usr/bin/env python3
import pygame
import math
from pygame import Color
from pygame import gfxdraw
from world.ObjectType import *


class WorldRenderer:
    """Render the world so we can see what the robot is doing"""

    def __init__(self, **kwargs):
        print("Initialising renderer...")

        pygame.init()
        pygame.display.set_caption("piwarsimulator")

        self.running = True
        self.x_res = int(kwargs.get("x_res", 800))
        self.y_res = int(kwargs.get("y_res", 600))
        self.x_offset = kwargs.get("x_offset", self.x_res / 2)
        self.y_offset = kwargs.get("y_offset", self.y_res / 2)
        self.screen = pygame.display.set_mode((self.x_res, self.y_res))
        self.world_scale = kwargs.get("world_scale", 200)
        self.font = pygame.font.Font(None, self.world_scale // 10)  # default font
        self.frame = 0

    def transform_coordinate(self, c, offset=0):
        """scale metres to pixels, offset by offset pixels"""
        return round(c * self.world_scale + offset)

    def transform_horizontal(self, c):
        """transform horizontal coordiant to pixels"""
        return self.transform_coordinate(c, self.x_offset)  # 10?

    def transform_vertical(self, c):
        """transform vertical coordiant to pixels"""
        return self.transform_coordinate(
            -c, -self.y_offset + self.screen.get_height()
        )  # -10?

    def update(self, *Worlds):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("Quit requested!")
                self.running = False

        # reset the canvas
        self.screen.fill(Color("black"))

        for TheWorld in Worlds:
            for obj in TheWorld:
                i = self.transform_horizontal(obj.pos[0])
                j = self.transform_vertical(obj.pos[1])
                r = self.transform_coordinate(obj.radius)
                if obj.object_type == ObjectType.WALL:
                    pygame.gfxdraw.line(
                        self.screen,
                        i - round(r * math.cos(math.radians(obj.angle))),
                        j - round(r * math.sin(math.radians(obj.angle))),
                        i + round(r * math.cos(math.radians(obj.angle))),
                        j + round(r * math.sin(math.radians(obj.angle))),
                        obj.color,
                    )
                else:
                    pygame.gfxdraw.filled_circle(self.screen, i, j, r, obj.color)
                    pygame.gfxdraw.aacircle(self.screen, i, j, r, obj.color)

                # render a label on each item in the world
                text = self.font.render(str(obj.object_type), True, Color("orange"))
                text2 = pygame.transform.rotate(text, -obj.angle)
                self.screen.blit(
                    text2,
                    (i - text2.get_width() // 2 + 0.5, j - text2.get_height() // 2 + 2),
                )

        # uncomment to save each frame to make a video
        #        pygame.image.save(self.screen, "frames/image%08d.png" % self.frame)

        pygame.display.flip()
        self.frame += 1

        # TODO would be nice if we could display log output on the screen
