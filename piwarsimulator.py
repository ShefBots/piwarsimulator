#!/usr/bin/env python3
import time
from WorldObject import *
from WorldRenderer import *
from ScanObject import *
from Scan import *

TheWorld = []

robot = WorldObject(objecttype=ObjectType.ROBOT, radius=0.1)

TheWorld.append(robot) # this should always be index 0!
TheWorld.append(WorldObject(objecttype=ObjectType.TARGET, x=1, y=0.2, radius=0.056, color='blue'))
TheWorld.append(WorldObject(objecttype=ObjectType.ZONE, x=2, y=0, radius=0.1, color='blue'))

#print(TheWorld)

renderer = WorldRenderer(TheWorld)

running = True
while running:
    renderer.update()
    running = renderer.running

    TheWorld[0].x += 0.01
    if TheWorld[0].x >= 1:
        running = False
    sensorinformation = Scan(TheWorld)
    print(sensorinformation)

    time.sleep(0.1)

#print(renderer)
