#!/usr/bin/env python3
from WorldObject import *
from WorldRenderer import *

TheWorld = []

robot = WorldObject(objecttype=ObjectType.ROBOT, radius=0.1)

TheWorld.append(robot)
TheWorld.append(WorldObject(objecttype=ObjectType.TARGET, x=1, y=0))
TheWorld.append(WorldObject(objecttype=ObjectType.ZONE, x=2, y=0))

#print(TheWorld)

renderer = WorldRenderer(TheWorld)

running = True
while running:
    TheWorld[0].x += 0.01
    renderer.update()
    if TheWorld[0].x >= 1:
        running = False

#print(renderer)
