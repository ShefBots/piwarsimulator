#!/usr/bin/env python3
import time
import random
from WorldObject import *
from WorldRenderer import *
from ScanObject import *
from Scan import *
from RobotBrain import *

TheWorld = []

robot = WorldObject(objecttype=ObjectType.ROBOT, x=1, radius=0.1)
robotbrain = RobotBrain(robot=robot, speed=0.02, turningspeed=3)

# the order this is constructed in is the rendering order...
TheWorld.append(robot) # this should always be index 0!
TheWorld.append(WorldObject(objecttype=ObjectType.ZONE, x=-1.25, y=0.5, radius=0.1, color='blue'))
TheWorld.append(WorldObject(objecttype=ObjectType.ZONE, x=-1.25, y=-0.5, radius=0.1, color='darkgreen'))

# check zone dimensions in rules, then maximum possible number of barrels that would fit

#TheWorld.append(WorldObject(objecttype=ObjectType.TARGET, x=1.25, y=0.2, radius=0.056, color='blue'))
#TheWorld.append(WorldObject(objecttype=ObjectType.TARGET, x=0, y=-0.5, radius=0.056, color='blue'))

for i in range(0, random.randint(15, 40)):
    if random.random() < 0.5:
        c = 'blue'
    else:
        c = 'darkgreen'
    TheWorld.append(WorldObject(objecttype=ObjectType.TARGET, \
        x=random.random()*2-1, y=random.random()*2-1, radius=0.056, color=c))

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

