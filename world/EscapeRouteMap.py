#!/usr/bin/env python3
from world.WorldObject import *


def EscapeRouteMap(ExteriorTheWorld):
    """
    escape route map
    """

    KEY_DIM = 0.589  # unit height off map diagram
    OBSTACLE_WIDTH = KEY_DIM * 5 / 3

    WIDTH = 1.83
    HEIGHT = 2.95

    # put the robot in the middle of the start zone
    X_OFF = -WIDTH * 7 / 8
    Y_OFF = -KEY_DIM / 2

    # bottom horizontal wall
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL,
            x1=0 + X_OFF,
            y1=0 + Y_OFF,
            x2=WIDTH + X_OFF,
            y2=0 + Y_OFF,
            color="lightgray",
        )
    )
    # partial top horizontal wall
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL,
            x1=WIDTH - KEY_DIM * 5 / 3 + X_OFF,
            y1=HEIGHT + Y_OFF,
            x2=WIDTH + X_OFF,
            y2=HEIGHT + Y_OFF,
            color="lightgray",
        )
    )

    # main vertical walls
    for x in [0, WIDTH]:
        ExteriorTheWorld.append(
            WorldObject(
                object_type=ObjectType.WALL,
                x1=x + X_OFF,
                y1=0 + Y_OFF,
                x2=x + X_OFF,
                y2=HEIGHT + Y_OFF,
                color="lightgray",
            )
        )

    # blue box (horizontal walls)
    for y in [KEY_DIM, KEY_DIM * 2]:
        ExteriorTheWorld.append(
            WorldObject(
                object_type=ObjectType.WALL,
                x1=WIDTH - OBSTACLE_WIDTH + X_OFF,
                y1=y + Y_OFF,
                x2=WIDTH + X_OFF,
                y2=y + Y_OFF,
                color="blue",
            )
        )
    # blue box vertical wall
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL,
            x1=WIDTH - OBSTACLE_WIDTH + X_OFF,
            y1=KEY_DIM + Y_OFF,
            x2=WIDTH - OBSTACLE_WIDTH + X_OFF,
            y2=KEY_DIM * 2 + Y_OFF,
            color="blue",
        )
    )

    # red box (horizontal walls)
    for y in [KEY_DIM * 3, KEY_DIM * 10 / 3]:
        ExteriorTheWorld.append(
            WorldObject(
                object_type=ObjectType.WALL,
                x1=0 + X_OFF,
                y1=y + Y_OFF,
                x2=OBSTACLE_WIDTH + X_OFF,
                y2=y + Y_OFF,
                color="red",
            )
        )
    # red box vertical wall
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL,
            x1=OBSTACLE_WIDTH + X_OFF,
            y1=KEY_DIM * 3 + Y_OFF,
            x2=OBSTACLE_WIDTH + X_OFF,
            y2=KEY_DIM * 10 / 3 + Y_OFF,
            color="red",
        )
    )

    # green box horizontal wall
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL,
            x1=WIDTH - OBSTACLE_WIDTH + X_OFF,
            y1=HEIGHT - KEY_DIM * 2 / 3 + Y_OFF,
            x2=WIDTH + X_OFF,
            y2=HEIGHT - KEY_DIM * 2 / 3 + Y_OFF,
            color="green",
        )
    )
    # green box vertical wall
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.WALL,
            x1=WIDTH - OBSTACLE_WIDTH + X_OFF,
            y1=HEIGHT - KEY_DIM * 2 / 3 + Y_OFF,
            x2=WIDTH - OBSTACLE_WIDTH + X_OFF,
            y2=HEIGHT + Y_OFF,
            color="green",
        )
    )

    # start line
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.LINE,
            x1=WIDTH * 3 / 4 + X_OFF,
            y1=0 + Y_OFF,
            x2=WIDTH * 3 / 4 + X_OFF,
            y2=KEY_DIM + Y_OFF,
            color="yellow",
        )
    )
    # finish line
    ExteriorTheWorld.append(
        WorldObject(
            object_type=ObjectType.LINE,
            x1=0 + X_OFF,
            y1=HEIGHT + Y_OFF,
            x2=WIDTH - KEY_DIM * 5 / 3 + X_OFF,
            y2=HEIGHT + Y_OFF,
            color="yellow",
        )
    )
