#!/usr/bin/env python3
from world.WorldObject import *

START_LOCATION = (0, 0)


def LavaPalavaMap(ExteriorTheWorld):
    """
    estimated lava palava map
    0.55 m distance between walls
    2.2 m of straight
    1 m left turn (35 degrees)
    1 m straight
    1 m right turn (35 degrees)
    2 m straight
    walls start at -0.2 m, course starts at 0 m
    """

    for x in [-0.275, 0, 0.275]:
        type = ObjectType.WALL
        c = "gray30"
        if x == 0:
            type = ObjectType.LINE
            c = "white"
        # first section
        ExteriorTheWorld.append(
            WorldObject(object_type=type, x1=x, y1=-0.2, x2=x, y2=2, color=c)
        )
        # left hand turn (35 degrees)
        ExteriorTheWorld.append(
            WorldObject(
                object_type=type, x1=x, y1=2, x2=x - 1 * math.tan(0.61), y2=3, color=c
            )
        )
        # second straight (presumably with hump)
        ExteriorTheWorld.append(
            WorldObject(
                object_type=type,
                x1=x - 1 * math.tan(0.61),
                y1=3,
                x2=x - 1 * math.tan(0.61),
                y2=4,
                color=c,
            )
        )
        # right hand turn (35 degrees)
        ExteriorTheWorld.append(
            WorldObject(
                object_type=type, x1=x - 1 * math.tan(0.61), y1=4, x2=x, y2=5, color=c
            )
        )
        # last section
        ExteriorTheWorld.append(
            WorldObject(object_type=type, x1=x, y1=5, x2=x, y2=7, color=c)
        )

    # wall at end
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL,
            x1=-0.275,
            y1=7.5,
            x2=0.275,
            y2=7.5,
            color="gray30",
        )
    )
