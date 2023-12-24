#!/usr/bin/env python3
import time
import numpy as np

from world.WorldObject import *
from world.WorldRenderer import *
from sensors.ScanObject import *
from sensors.Scan import *
from brains.RobotBrain import RobotBrain

# TODO add log class, use that for output instead of print
# TODO for now everything is simulated
# TODO proper dimensions and whatnot

ExteriorTheWorld = []

robot = WorldObject(object_type=ObjectType.ROBOT, x=0, y=0, radius=0.1, angle=0) # units metres and degress

# the order this is constructed in is also the rendering order...
ExteriorTheWorld.append(robot) # this should always be index 0!

# for walls x and y are the center of the wall and they are extended by radius
ExteriorTheWorld.append(WorldObject(object_type=ObjectType.WALL, x=0, y=-1, angle = 0, radius=1, color='gray', ignore=True))
ExteriorTheWorld.append(WorldObject(object_type=ObjectType.WALL, x=0, y=1, angle = 0, radius=1, color='gray', ignore=True))
ExteriorTheWorld.append(WorldObject(object_type=ObjectType.WALL, x=-1, y=0, angle = 90, radius=1, color='gray', ignore=True))
ExteriorTheWorld.append(WorldObject(object_type=ObjectType.WALL, x=1, y=0, angle = 90, radius=1, color='gray', ignore=True))

# some barrels
ExteriorTheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=0.5, y=-0.5, radius=0.056, color='red'))
ExteriorTheWorld.append(WorldObject(object_type=ObjectType.BARREL, x=0.5, y=0.5, radius=0.056, color='darkgreen'))

# logic for the robot
robot_brain = RobotBrain(robot=robot, speed=0.3, turning_speed=45)

renderer = WorldRenderer() # default 0,0 is centre of screen
renderer.update()
print("Getting ready...")
time.sleep(1) # wait for the world to load

running = True
print("Running...")
frame_time = 1/60.0 # aim for 60 fps simulation
while running:
    now = time.time();
    renderer.update(ExteriorTheWorld)
    running = renderer.running
    # TODO should handle ctrl c and alt f4

    # TODO when running real hardware Scan() will be replaced with something
    # that talks to the sensors, and then for the renderer we'll need to create
    # a TheWorld based off of that
    #sensor_information = Scan(TheWorld)
#    robot_brain.process(sensor_information)
    #robot_brain.simulate(dt)

    to_sleep = frame_time - (time.time() - now)
    if to_sleep > 0:
        time.sleep(to_sleep)
