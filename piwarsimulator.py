#!/usr/bin/env python3
import time
import numpy as np

from world.WorldObject import *
from world.WorldRenderer import *
from sensors.SimulatedVision360 import SimulatedVision360
from brains.RobotBrain import RobotBrain

# TODO add log class, use that for output instead of print
# TODO for now everything is simulated
# TODO proper dimensions and whatnot
# TODO should trap/handle ctrl c

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
robot_brain.add_sensor(SimulatedVision360(ExteriorTheWorld))

renderer = WorldRenderer() # default 0,0 is centre of screen
renderer.update()
print("Getting ready...")
time.sleep(1) # wait for the world to load

running = True
print("Running...")
frame_time = 1/60.0 # aim for 60 fps simulation
while running:
    now = time.time();

    robot_brain.process()
    #robot_brain.simulate(dt)

    renderer.update(ExteriorTheWorld, robot_brain.TheWorld) # see the world as it is and as the robot sees it
    #renderer.update(robot_brain.TheWorld) # see the world as the robot sees it
    running = renderer.running
    to_sleep = frame_time - (time.time() - now)
    if to_sleep > 0:
        time.sleep(to_sleep)
