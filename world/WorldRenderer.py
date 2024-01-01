#!/usr/bin/env python3
import pygame
import time
import numpy as np
from pygame import Color
from world.ObjectType import *

# TODO wall detection and avoidance


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
        self.screen = pygame.display.set_mode(
            (self.x_res * self.num_worlds, self.y_res)
        )
        self.world_scale = kwargs.get("world_scale", self.y_res / 3)
        self.font = pygame.font.Font(None, self.world_scale // 10)  # default font
        self.small_font = pygame.font.Font(None, self.world_scale // 14)  # smaller font
        self.last_time = time.time()
        self.fps = np.ones(30) * (1 / 60)
        self.fps_at = 0
        self.frame = 0

    def transform_horizontal(self, scale, c):
        """transform horizontal coordiant to pixels"""
        return np.round(c * scale + self.x_res / 2)

    def transform_vertical(self, c, scale):
        """transform vertical coordiant to pixels (flipping so y = 0 is bottom )"""
        return np.round(-c * scale + self.screen.get_height() - self.y_res / 2)

    def update(self, *Worlds):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("Quit requested!")
                self.running = False

        # reset the canvas
        self.screen.fill(Color("black"))

        world_at = 0
        for TheWorld in Worlds:
            world_scale = self.world_scale

            for obj in reversed(TheWorld):
                # iterate through backwards so that the robot is always rendered last (on top)
                # get the coordinates of the outline (also draws line type objects)
                x, y = obj.xy()
                x = (
                    self.transform_horizontal(np.array(x), world_scale)
                    + world_at * self.x_res
                )
                y = self.transform_vertical(np.array(y), world_scale)
                pnts = np.column_stack((x, y))

                # draw the outline/line & fill if outline
                if len(x) > 2:
                    pygame.draw.polygon(self.screen, obj.color, pnts)
                pygame.draw.aalines(self.screen, obj.color, True, pnts)
                # TODO is there a way to draw a thicker antialiased line?
                # render at a higher resolution with thick regular lines and resize to the dispaly res? (via @ZodiusInfuser)

                # coordinates of center
                i = (
                    self.transform_horizontal(obj.center[0], world_scale)
                    + world_at * self.x_res
                )
                j = self.transform_vertical(obj.center[1], world_scale)

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
        # pygame.image.save(self.screen, "frames/image%08d.bmp" % self.frame)
        # ffmpeg -framerate 60 -i frames/image%08d.bmp -r 60 -vcodec libx265 -preset medium -crf 28 test.mp4
        # ffmpeg -framerate 60 -i frames/image%08d.bmp -r 60 -vcodec libx264 -crf 28 -pix_fmt yuv420p test.mp4

        pygame.display.flip()
        self.frame += 1

        # TODO would be nice if we could display log output on the screen
