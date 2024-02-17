#!/usr/bin/env python3
import pygame
import numpy as np
from shapely.affinity import rotate
from shapely.affinity import translate
from time import monotonic
from world.ObjectType import *
from util import outline_xy


class WorldRenderer:
    """Render the world so we can see what the robot is doing"""

    VERTICAL_VIEW = 3.0  # number of metres shown
    AA_THICK_LINES = 0  # set to 1 for smooth thick lines for a performance penalty

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
        if self.AA_THICK_LINES == 1:
            self.highres_screen = pygame.Surface(
                [self.screen.get_width() * 2, self.screen.get_height() * 2],
                pygame.SRCALPHA,
                32,
            )
        self.world_scale = kwargs.get("world_scale", self.y_res / self.VERTICAL_VIEW)
        self.auto_scale = kwargs.get("auto_scale", True)
        self.font = pygame.font.Font(None, self.y_res // 32)  # default font
        self.small_font = pygame.font.Font(None, self.y_res // 42)  # smaller font
        self.last_time = monotonic()
        self.fps = np.ones(30) * (1 / 60)
        self.fps_at = 0
        self.frame = 0

    def transform_horizontal(self, scale, c):
        """transform horizontal coordiant to pixels"""
        return np.round(c * scale + self.x_res / 2)

    def transform_vertical(self, c, scale):
        """transform vertical coordiant to pixels (flipping so y = 0 is bottom )"""
        return np.round(-c * scale + self.screen.get_height() - self.y_res / 2)

    def update(self, Worlds=[], Sensors=[]):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("Quit requested!")
                self.running = False

        # reset the canvas
        self.screen.fill(pygame.Color("black"))
        if self.AA_THICK_LINES == 1:
            self.highres_screen.fill((0, 0, 0, 0))

        world_at = 0
        for k, TheWorld in enumerate(Worlds):
            # render each world

            # the default scale works out so that about 2 m is shown
            # work out a scale that can see the whold world
            # might want to turn this off if CPU usage is high
            world_scale = self.world_scale
            zerozero_xoffset_pixels = 0
            zerozero_yoffset_pixels = 0
            if self.auto_scale:
                # thanks ChatGPT for making this a bit shorter
                xmin = min(np.min(outline_xy(obj.outline)[0]) for obj in TheWorld)
                xmax = max(np.max(outline_xy(obj.outline)[0]) for obj in TheWorld)
                ymin = min(np.min(outline_xy(obj.outline)[1]) for obj in TheWorld)
                ymax = max(np.max(outline_xy(obj.outline)[1]) for obj in TheWorld)
                m = max(xmax - xmin, ymax - ymin)
                if m > self.VERTICAL_VIEW:
                    world_scale = self.y_res / (m + 0.5)
                zerozero_xoffset_pixels = -np.round(
                    world_scale * (xmax - (xmax - xmin) / 2)
                )
                zerozero_yoffset_pixels = np.round(
                    world_scale * (ymax - (ymax - ymin) / 2)
                )

            for s in Sensors[k]:
                if not s is None:
                    # draw sensor outline
                    self.plot_outline(
                        s,
                        pygame.Color("azure"),
                        world_scale,
                        world_at,
                        zerozero_xoffset_pixels,
                        zerozero_yoffset_pixels,
                        fill=False,
                    )

            for obj in reversed(TheWorld):
                # iterate through backwards so that the robot is always rendered last (on top)

                # draw object outline
                self.plot_outline(
                    obj.outline,
                    obj.color,
                    world_scale,
                    world_at,
                    zerozero_xoffset_pixels,
                    zerozero_yoffset_pixels,
                )

                if (
                    obj.object_type == ObjectType.ROBOT
                    and hasattr(obj, "brain")
                    and hasattr(obj.brain, "attachment_outline")
                    and not obj.brain.attachment_outline.is_empty
                ):
                    ao = rotate(
                        translate(
                            obj.brain.attachment_outline, obj.center[0], obj.center[1]
                        ),
                        -obj.angle,
                        origin=(obj.center[0], obj.center[1]),
                    )

                    for o in ao.geoms:
                        self.plot_outline(
                            o,
                            obj.color,
                            world_scale,
                            world_at,
                            zerozero_xoffset_pixels,
                            zerozero_yoffset_pixels,
                        )

                # coordinates of center
                i = (
                    self.transform_horizontal(obj.center[0], world_scale)
                    + world_at * self.x_res
                    + zerozero_xoffset_pixels
                )
                j = (
                    self.transform_vertical(obj.center[1], world_scale)
                    + zerozero_yoffset_pixels
                )

                # render a label on each item in the world
                font = self.font
                if obj.object_type == ObjectType.BARREL:
                    font = self.small_font
                text = font.render(str(obj.object_type), True, pygame.Color("orange"))
                text2 = pygame.transform.rotate(text, -obj.angle)
                self.screen.blit(
                    text2,
                    (i - text2.get_width() // 2 + 0.5, j - text2.get_height() // 2 + 2),
                )
            world_at += 1

            # line to split viewports
            pygame.draw.aaline(
                self.screen,
                pygame.Color("gray80"),
                (world_at * self.x_res, 0),
                (world_at * self.x_res, self.y_res),
            )

        # calculate fps
        now = monotonic()
        frame_time = now - self.last_time
        self.fps[self.fps_at] = 1 / frame_time
        fps = np.mean(self.fps)
        text = self.small_font.render(f"{fps:.0f} FPS", True, pygame.Color("gray"))
        self.screen.blit(text, (10, 10))
        self.last_time = now
        self.fps_at += 1
        if self.fps_at == len(self.fps):
            self.fps_at = 0

        if self.AA_THICK_LINES == 1:
            self.screen.blit(
                pygame.transform.smoothscale_by(self.highres_screen, 0.5),
                (0, 0),
            )

        # uncomment to save each frame to make a video
        # pygame.image.save(self.screen, "frames/image%08d.bmp" % self.frame)
        # ffmpeg -framerate 60 -i frames/image%08d.bmp -r 60 -vcodec libx265 -preset medium -crf 28 test.mp4
        # ffmpeg -framerate 60 -i frames/image%08d.bmp -r 60 -vcodec libx264 -crf 28 -pix_fmt yuv420p test.mp4

        pygame.display.update()  # update is faster than flip
        self.frame += 1

    def plot_outline(
        self,
        outline,
        color,
        world_scale,
        world_at,
        zerozero_xoffset_pixels,
        zerozero_yoffset_pixels,
        fill=True,
    ):
        # get the coordinates of the outline (also draws line type objects)
        x, y = outline_xy(outline)
        x = (
            self.transform_horizontal(np.array(x), world_scale)
            + world_at * self.x_res
            + zerozero_xoffset_pixels
        )
        y = self.transform_vertical(np.array(y), world_scale) + zerozero_yoffset_pixels
        pnts = np.column_stack((x, y))

        # draw the outline/line & fill if outline
        if len(x) > 2 and fill == True:
            pygame.draw.polygon(self.screen, color, pnts)
            if self.AA_THICK_LINES == 1:
                pygame.draw.aalines(self.screen, color, True, pnts)
        else:
            if self.AA_THICK_LINES == 1:
                pygame.draw.lines(self.highres_screen, color, True, pnts * 2, width=4)
            else:
                pygame.draw.aalines(self.screen, color, True, pnts)
