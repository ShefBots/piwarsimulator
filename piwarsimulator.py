#!/usr/bin/env python3
import time
import random
from WorldObject import *
from WorldRenderer import *
from ScanObject import *
from Scan import *
from brains.EcoDisasterBrain import EcoDisasterBrain
import numpy as np

# TODO add log class, use that for output instead of print

robot = WorldObject(object_type=ObjectType.ROBOT, x=1.1, y=0.4, radius=0.1125)
robot_brain = EcoDisasterBrain(robot=robot, speed=0.3, turning_speed=45)

TheWorld = [[WorldObject(object_type=ObjectType.UNKNOWN, x=i, y=j, radius=0.1, color='black') for i in range(25)] for j in range(25)]

TheWorld[11][4] = robot
TheWorld[5][21] = WorldObject(object_type=ObjectType.ZONE, x=0.5, y=2.1, radius=0.1, color='blue')
TheWorld[7][21] = WorldObject(object_type=ObjectType.ZONE, x=0.7, y=2.1, radius=0.1, color='blue')
TheWorld[9][21] = WorldObject(object_type=ObjectType.ZONE, x=0.9, y=2.1, radius=0.1, color='blue')
TheWorld[13][21] = WorldObject(object_type=ObjectType.ZONE, x=1.3, y=2.1, radius=0.1, color='darkgreen')
TheWorld[15][21] = WorldObject(object_type=ObjectType.ZONE, x=1.5, y=2.1, radius=0.1, color='darkgreen')
TheWorld[17][21] = WorldObject(object_type=ObjectType.ZONE, x=1.7, y=2.1, radius=0.1, color='darkgreen')

print(np.matrix(TheWorld))

# TheWorld[]
#
# # The walls around the arena, see https://i0.wp.com/piwars.org/wp-content/uploads/2019/07/EcoDisasterPlanLabelled.png
# # for walls, the radius is half the length
# TheWorld.append(WorldObject(object_type=ObjectType.WALL, x=1.1, y=0, angle = 0, radius=1.1, color='gray', ignore=True))
# TheWorld.append(WorldObject(object_type=ObjectType.WALL, x=1.1, y=2.2, angle = 0, radius=1.1, color='gray', ignore=True))
# TheWorld.append(WorldObject(object_type=ObjectType.WALL, x=0, y=1.1, angle = 90, radius=1.1, color='gray', ignore=True))
# TheWorld.append(WorldObject(object_type=ObjectType.WALL, x=2.2, y=1.1, angle = 90, radius=1.1, color='gray', ignore=True))

#TheWorld.append(WorldObject(object_type=ObjectType.TARGET, x=1.25, y=0.2, radius=0.056, color='blue'))
#TheWorld.append(WorldObject(object_type=ObjectType.TARGET, x=0, y=-0.5, radius=0.056, color='blue'))

for i in range(0, random.randint(6, 12)):
    if random.random() < 0.5:
        c = 'blue'
    else:
        c = 'darkgreen'

    x = round(random.random() * 25)
    y = round(random.random() * 21)

    TheWorld[x][y] = WorldObject(
        object_type=ObjectType.TARGET,
        x=x,
        y=y,
        radius=0.056,
        color=c
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
    # TODO should handle ctrl c and alt f4

    # TODO when running real hardware Scan() will be replaced with something
    # that talks to the sensors, and then for the renderer we'll need to create
    # a TheWorld based off of that
    # sensor_information = Scan(TheWorld)
    # robot_brain.process(sensor_information)
    # robot_brain.simulate(dt)

    tosleep = dt - (time.time() - now)
    if tosleep > 0:
        time.sleep(tosleep)
