#!/usr/bin/env python3
import time
from WorldObject import *
from WorldRenderer import *
from ScanObject import *
from Scan import *
from RobotBrain import *

TheWorld = []

robot = WorldObject(objecttype=ObjectType.ROBOT, radius=0.1)
robotbrain = RobotBrain(robot=robot, speed=0.01, turningspeed=3)

# the order this is constructed in is the rendering order...
TheWorld.append(robot) # this should always be index 0!
TheWorld.append(WorldObject(objecttype=ObjectType.ZONE, x=-1, y=0.5, radius=0.1, color='blue'))
TheWorld.append(WorldObject(objecttype=ObjectType.TARGET, x=1, y=0.2, radius=0.056, color='blue'))
TheWorld.append(WorldObject(objecttype=ObjectType.TARGET, x=0, y=-0.5, radius=0.056, color='blue'))

renderer = WorldRenderer(TheWorld)

running = True
while running:
    renderer.update()
    running = renderer.running

    sensorinformation = Scan(TheWorld)
#    print(sensorinformation)
    robotbrain.move(sensorinformation)
#    print(TheWorld)

# need a routine to clean up the world and remove targets that are in goals

    time.sleep(0.1)

