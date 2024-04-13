#!/usr/bin/env python3
from world.WorldObject import *

START_LOCATION = (0, 0)


def EmptyMap(ExteriorTheWorld):
    # Twice this is the size of each edge of the box
    box_size = 1.4

    # for walls x and y are the center of the wall and they are extended by radius
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL, x=0, y=-box_size, angle=90, l=box_size*2, color="gray"
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL, x=0, y=box_size, angle=90, l=box_size*2, color="gray"
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL, x=-box_size, y=0, angle=0, l=box_size*2, color="gray"
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL, x=box_size, y=0, angle=0, l=box_size*2, color="gray"
        )
    )
