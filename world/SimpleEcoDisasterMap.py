#!/usr/bin/env python3
from world.WorldObject import *


def SimpleEcoDisasterMap(ExteriorTheWorld):
    # for walls x and y are the center of the wall and they are extended by radius
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL,
            x=0,
            y=-1,
            angle=0,
            radius=1,
            color="gray",
            ignore=True,
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL,
            x=0,
            y=1,
            angle=0,
            radius=1,
            color="gray",
            ignore=True,
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL,
            x=-1,
            y=0,
            angle=90,
            radius=1,
            color="gray",
            ignore=True,
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL,
            x=1,
            y=0,
            angle=90,
            radius=1,
            color="gray",
            ignore=True,
        )
    )

    # some barrels
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.BARREL, x=0.5, y=-0.55, radius=0.04, color="red"
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.BARREL, x=0.5, y=0.5, radius=0.04, color="darkgreen"
        )
    )

    # some zones
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.ZONE, x=-0.5, y=0.9, radius=0.08, color="blue"
        )
    )
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.ZONE, x=0.5, y=0.9, radius=0.08, color="yellow"
        )
    )
