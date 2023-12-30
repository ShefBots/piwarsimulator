#!/usr/bin/env python3
import pygame
import time
import numpy as np
from pygame import Color
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
        self.num_worlds = int(kwargs.get("num_worlds", 1))
        self.x_offset = kwargs.get("x_offset", self.x_res / 2)
        self.y_offset = kwargs.get("y_offset", self.y_res / 2)
        self.screen = pygame.display.set_mode(
            (self.x_res * self.num_worlds, self.y_res)
        )
        self.world_scale = kwargs.get("world_scale", 200)
        self.font = pygame.font.Font(None, self.world_scale // 10)  # default font
        self.small_font = pygame.font.Font(None, self.world_scale // 16)  # default font
        self.last_time = time.time()
        self.fps = np.ones(30) * (1 / 60)
        self.fps_at = 0
        self.frame = 0

    def transform_coordinate(self, c, offset=0):
        """scale metres to pixels, offset by offset pixels"""
        return np.round(c * self.world_scale + offset)

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

        world_at = 0
        for TheWorld in Worlds:
            for obj in reversed(TheWorld):
                # iterate through backwards so that the robot is always rendered last (on top)
                # get the coordinates of the outline (also draws line type objects)
                x, y = obj.xy()
                x = self.transform_horizontal(np.array(x)) + world_at * self.x_res
                y = self.transform_vertical(np.array(y))
                pnts = np.column_stack((x, y))

                # draw the outline/line & fill if outline
                if len(x) > 2:
                    pygame.draw.polygon(self.screen, obj.color, pnts)
                pygame.draw.aalines(self.screen, obj.color, True, pnts)
                # TODO is there a way to draw a thicker antialiased line?
                # render at a higher resolution with thick regular lines and resize to the dispaly res? (via @ZodiusInfuser)

                # coordinates of center
                i = self.transform_horizontal(obj.center[0]) + world_at * self.x_res
                j = self.transform_vertical(obj.center[1])

                # render a label on each item in the world
                font = self.font
                if obj.object_type == ObjectType.BARREL:
                    font = self.small_font
                text = font.render(str(obj.object_type), True, Color("orange"))
                text2 = pygame.transform.rotate(text, -obj.angle)
                self.screen.blit(
                    text2,
                    (i - text2.get_width() // 2 + 0.5, j - text2.get_height() // 2 + 2),
                )
            world_at += 1

        # calculate fps
        now = time.time()
        frame_time = now - self.last_time
        self.fps[self.fps_at] = 1 / frame_time
        fps = np.mean(self.fps)
        text = self.small_font.render(f"{fps:.0f} FPS", True, Color("gray"))
        self.screen.blit(text, (10, 10))
        self.last_time = now
        self.fps_at += 1
        if self.fps_at == len(self.fps):
            self.fps_at = 0

        # uncomment to save each frame to make a video
        # pygame.image.save(self.screen, "frames/image%08d.png" % self.frame)

        pygame.display.flip()
        self.frame += 1

        # TODO would be nice if we could display log output on the screen
