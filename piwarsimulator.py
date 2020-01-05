#!/usr/bin/env python3
import time
import random
from WorldObject import *
from WorldRenderer import *
from ScanObject import *
from Scan import *
from RobotBrain import *

TheWorld = []

robot = WorldObject(object_type=ObjectType.ROBOT, x=1.1, y=0.4, radius=0.1125)
robot_brain = RobotBrain(robot=robot, speed=0.02, turning_speed=3)

# the order this is constructed in is the rendering order...
TheWorld.append(robot) # this should always be index 0!
TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=0.5, y=2.1, radius=0.1, color='blue'))
TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=0.7, y=2.1, radius=0.1, color='blue'))
TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=0.9, y=2.1, radius=0.1, color='blue'))
TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=1.3, y=2.1, radius=0.1, color='darkgreen'))
TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=1.5, y=2.1, radius=0.1, color='darkgreen'))
TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=1.7, y=2.1, radius=0.1, color='darkgreen'))

# check zone dimensions in rules, then maximum possible number of barrels that would fit

#TheWorld.append(WorldObject(object_type=ObjectType.TARGET, x=1.25, y=0.2, radius=0.056, color='blue'))
#TheWorld.append(WorldObject(object_type=ObjectType.TARGET, x=0, y=-0.5, radius=0.056, color='blue'))

for i in range(0, random.randint(6, 12)):
    if random.random() < 0.5:
        c = 'blue'
    else:
        c = 'darkgreen'
    TheWorld.append(
        WorldObject(
            object_type=ObjectType.TARGET,
            x=0.3+random.random()*1.6,
            y=0.3+random.random()*1.6,
            radius=0.056,
            color=c
        )
    )

renderer = WorldRenderer(TheWorld)

running = True
while running:
    renderer.update()
    running = renderer.running

    sensor_information = Scan(TheWorld)
#    print(sensorinformation)
    robot_brain.move(sensor_information)
#    print(TheWorld)

# need a routine to clean up the world and remove targets that are in goals

    time.sleep(0.1)
