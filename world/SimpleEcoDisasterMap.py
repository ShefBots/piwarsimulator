#!/usr/bin/env python3
from world.WorldObject import *

START_LOCATION = (0, -0.7)


def SimpleEcoDisasterMap(ExteriorTheWorld):
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
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.BARREL,
            x=-0.5,
            y=-0.5,
            radius=0.025,
            color="darkgreen",
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.BARREL, x=0.5, y=-0.5, radius=0.025, color="red"
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.BARREL, x=-0.5, y=0.5, radius=0.025, color="red"
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.BARREL, x=0.5, y=0.5, radius=0.025, color="darkgreen"
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
