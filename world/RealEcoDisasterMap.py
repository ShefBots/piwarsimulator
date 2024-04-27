#!/usr/bin/env python3
from world.WorldObject import *

START_LOCATION = (0, -0.7)


def RealEcoDisasterMap(ExteriorTheWorld):
    # for walls x and y are the center of the wall and they are extended by radius
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL, x=0, y=-1.1, angle=90, l=2.2, color="gray"
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL, x=0, y=1.1, angle=90, l=2.2, color="gray"
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL, x=-1.1, y=0, angle=0, l=2.2, color="gray"
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL, x=1.1, y=0, angle=0, l=2.2, color="gray"
        )
    )

    # tuple of x, y, color (x,y from bottom left, need to recompute)
    barrels = [
        (0.554, 1.54, "red"),
        (1.553, 1.880, "red"),
        (1.46, 1.536, "darkgreen"),
        (0.620, 1.2, "darkgreen"),
        (1.1, 1.2, "darkgreen"),
        (1.5, 1.13, "red"),
        (0.6, 0.93, "red"),
        (0.85, 0.9, "red"),
        (1.16, 0.93, "darkgreen"),
        (1.25, 0.64, "darkgreen"),
        (1.55, 0.6, "darkgreen"),
        (1.64, 0.54, "red"),
    ]

    for b in barrels:
        x = b[0] - 1.1  # + 0.07
        y = 1.1 - b[1]  # + 0.05

        ExteriorTheWorld.append(
            WorldObject(
                object_type=ObjectType.BARREL,
                x=x,
                y=y,
                radius=0.025,
                color=b[2],
            )
        )

    # some zones
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.ZONE, x=-0.4, y=1.0, w=0.6, h=0.2, color="blue"
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.ZONE, x=0.4, y=1.0, w=0.6, h=0.2, color="yellow"
        )
    )
