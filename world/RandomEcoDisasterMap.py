#!/usr/bin/env python3
from random import random
from shapely.geometry import Point
from shapely.ops import unary_union
from world.WorldObject import *

START_LOCATION = (0, -0.7)


def RandomEcoDisasterMap(ExteriorTheWorld):
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

    # some barrels - keep in mind (0,0) is the middle of the map and (-1.1,-1.1) is a corner
    numgreen = 0
    numred = 0
    for _ in range(0, 12):
        if (random() < 0.5 and numred < 6) or numgreen == 6:
            c = "red"
            numred += 1
        else:
            c = "darkgreen"
            numgreen += 1

        # need to check if we have a valid barrel position
        # start with a known bad position to kick off checks
        x = START_LOCATION[0]
        y = START_LOCATION[1]
        # regenerate barrel position if inside start box
        # (for the y limit, 2.2/2 = 1.1, 1.1 - 0.4 - 0.2 = 0.5)
        # check new barrel position doesn't overlap with any existing barrels
        # use something like 150 mm clearance in any direction from anything in ExteriorTheWorld
        while (x > -0.225 / 2 and x < 0.225 / 2 and y < -0.5) or unary_union(
            [o.outline.buffer(0.15) for o in ExteriorTheWorld]
        ).contains(Point(x, y)):
            x = random() * 1.6 - 0.8
            y = random() * 1.6 - 0.8

        ExteriorTheWorld.append(
            WorldObject(
                object_type=ObjectType.BARREL,
                x=x,
                y=y,
                radius=0.025,
                color=c,
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
