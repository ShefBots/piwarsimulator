#!/usr/bin/env python3
import time
import random
from WorldObject import *
from WorldRenderer import *
from ScanObject import *
from Scan import *
from brains.EcoDisasterBrain import EcoDisasterBrain

TheWorld = []

robot = WorldObject(object_type=ObjectType.ROBOT, x=1.1, y=0.4, radius=0.1125)
robot_brain = EcoDisasterBrain(robot=robot, speed=0.3, turning_speed=45)

# the order this is constructed in is the rendering order...
TheWorld.append(robot) # this should always be index 0!
TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=0.5, y=2.1, radius=0.1, color='blue'))
TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=0.7, y=2.1, radius=0.1, color='blue'))
TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=0.9, y=2.1, radius=0.1, color='blue'))
TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=1.3, y=2.1, radius=0.1, color='darkgreen'))
TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=1.5, y=2.1, radius=0.1, color='darkgreen'))
TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=1.7, y=2.1, radius=0.1, color='darkgreen'))

# The walls around the arena, see https://i0.wp.com/piwars.org/wp-content/uploads/2019/07/EcoDisasterPlanLabelled.png
# for walls, the radius is half the length
TheWorld.append(WorldObject(object_type=ObjectType.WALL, x=1.1, y=0, angle = 0, radius=1.1, color='gray', ignore=True))
TheWorld.append(WorldObject(object_type=ObjectType.WALL, x=1.1, y=2.2, angle = 0, radius=1.1, color='gray', ignore=True))
TheWorld.append(WorldObject(object_type=ObjectType.WALL, x=0, y=1.1, angle = 90, radius=1.1, color='gray', ignore=True))
TheWorld.append(WorldObject(object_type=ObjectType.WALL, x=2.2, y=1.1, angle = 90, radius=1.1, color='gray', ignore=True))


# check zone dimensions in rules, then maximum possible number of barrels that would fit

#TheWorld.append(WorldObject(object_type=ObjectType.TARGET, x=1.25, y=0.2, radius=0.056, color='blue'))
#TheWorld.append(WorldObject(object_type=ObjectType.TARGET, x=0, y=-0.5, radius=0.056, color='blue'))

for i in range(0, random.randint(6, 12)):
    if random.random() < 0.5:
        c = 'blue'
    else:
        c = 'darkgreen'

    # need to check if we have a valid barrel position
    x=0.3+random.random()*1.6
    y=0.3+random.random()*1.6
    while x > 0.95 and x < 1.25 and y > 0.2 and y < 0.6:
        x=0.3+random.random()*1.6
        y=0.3+random.random()*1.6

    TheWorld.append(
        WorldObject(
            object_type=ObjectType.TARGET,
            x=x,
            y=y,
            radius=0.056,
            color=c
        )
    )

renderer = WorldRenderer(TheWorld)
renderer.update()
print("getting ready...")
time.sleep(1) # wait for the world to load
print("go!")

running = True
dt = 1/60.0 # aim for 60 fps simulation
while running:
    now = time.time();
    renderer.update()
    running = renderer.running

    sensor_information = Scan(TheWorld)
    robot_brain.process(sensor_information)
    robot_brain.simulate(dt)

    tosleep = dt - (time.time() - now)
    if tosleep > 0:
        time.sleep(tosleep)

    # need a routine to clean up the world and remove targets that are in goals
