#!/usr/bin/env python3
import time
import random
from world.WorldObject import *
from world.WorldRenderer import *
from sensors.ScanObject import *
from sensors.Scan import *
from brains.EcoDisasterBrain import EcoDisasterBrain
import numpy as np

# TODO add log class, use that for output instead of print

TheWorld = []

#robot = WorldObject(object_type=ObjectType.ROBOT, x=1.1, y=0.4, radius=0.1125)
robot = WorldObject(object_type=ObjectType.ROBOT, x=0, y=0, radius=0.1125)
robot.angle = 90
robot_brain = EcoDisasterBrain(robot=robot, speed=0.3, turning_speed=45)

# the order this is constructed in is the rendering order...
TheWorld.append(robot) # this should always be index 0!
#TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=0.5, y=2.1, radius=0.1, color='blue'))
#TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=0.7, y=2.1, radius=0.1, color='blue'))
#TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=0.9, y=2.1, radius=0.1, color='blue'))
#TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=1.3, y=2.1, radius=0.1, color='darkgreen'))
#TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=1.5, y=2.1, radius=0.1, color='darkgreen'))
#TheWorld.append(WorldObject(object_type=ObjectType.ZONE, x=1.7, y=2.1, radius=0.1, color='darkgreen'))

# The walls around the arena, see https://i0.wp.com/piwars.org/wp-content/uploads/2019/07/EcoDisasterPlanLabelled.png
# for walls, the radius is half the length
#TheWorld.append(WorldObject(object_type=ObjectType.WALL, x=1.1, y=0, angle = 0, radius=1.1, color='gray', ignore=True))
#TheWorld.append(WorldObject(object_type=ObjectType.WALL, x=1.1, y=2.2, angle = 0, radius=1.1, color='gray', ignore=True))
#TheWorld.append(WorldObject(object_type=ObjectType.WALL, x=0, y=1.1, angle = 90, radius=1.1, color='gray', ignore=True))
#TheWorld.append(WorldObject(object_type=ObjectType.WALL, x=2.2, y=1.1, angle = 90, radius=1.1, color='gray', ignore=True))


TheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=-0.460534, y=-0.017217, radius=0.056, color='red'))
TheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=-0.272540, y=0.015456, radius=0.056, color='red'))
TheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=-0.257404, y=0.290819, radius=0.056, color='red'))
TheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=0.195235, y=0.225820, radius=0.056, color='red'))
TheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=0.181215, y=-0.116322, radius=0.056, color='red'))
TheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=0.118089, y=-0.306859, radius=0.056, color='red'))

TheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=-0.283221, y=-0.236239, radius=0.056, color='darkgreen'))
TheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=0.066763, y=0.426531, radius=0.056, color='darkgreen'))
TheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=0.344891, y=0.159929, radius=0.056, color='darkgreen'))
TheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=0.429362, y=-0.031419, radius=0.056, color='darkgreen'))
TheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=0.430329, y=-0.252814, radius=0.056, color='darkgreen'))
TheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=-0.033227, y=-0.254974, radius=0.056, color='darkgreen'))


#TheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=1.25, y=0.2, radius=0.056, color='blue'))
#TheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=0, y=-0.5, radius=0.056, color='blue'))

#for i in range(0, random.randint(6, 12)):
#    if random.random() < 0.5:
#        c = 'blue'
#    else:
#        c = 'darkgreen'
#
#    # need to check if we have a valid barrel position
#    x=0.3+random.random()*1.6
#    y=0.3+random.random()*1.6
#    while x > 0.95 and x < 1.25 and y > 0.2 and y < 0.6:
#        x=0.3+random.random()*1.6
#        y=0.3+random.random()*1.6
#
#    TheWorld.append(
#        WorldObject(
#            object_type=ObjectType.BARREL,
#            x=x,
#            y=y,
#            radius=0.056,
#            color=c
#        )
#    )

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
    sensor_information = Scan(TheWorld)
#    robot_brain.process(sensor_information)
    robot_brain.simulate(dt)

    tosleep = dt - (time.time() - now)
    if tosleep > 0:
        time.sleep(tosleep)
