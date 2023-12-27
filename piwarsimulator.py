#!/usr/bin/env python3
import time
import numpy as np
from signal import signal, SIGINT
from sys import exit

# from brains.RobotBrain import RobotBrain
from brains.EcoDisasterBrain import EcoDisasterBrain
from controllers.SimulatedMovementController import SimulatedMovementController
from sensors.SimulatedVision360 import SimulatedVision360
from world.WorldObject import *
from world.WorldRenderer import *
from world.EcoDisasterMap import *

# TODO add log class, use that for output instead of print
# TODO for now everything is simulated
# TODO proper dimensions and whatnot

running = True

def handler(signal_received, frame):
    """trap/handle ctrl c"""
    global running # why does this need to be global?
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    running = False
signal(SIGINT, handler)


ExteriorTheWorld = []

robot = WorldObject(object_type=ObjectType.ROBOT, x=0, y=0, radius=0.1, angle=0) # units metres and degress

# the order this is constructed in is also the rendering order...
ExteriorTheWorld.append(robot) # this should always be index 0!

# import a map
SimpleEcoDisasterMap(ExteriorTheWorld)

# logic for the robot
sim_controller = SimulatedMovementController(robot)
# sim_controller.set_plane_velocity([0, 0.05])
# sim_controller.set_angular_velocity(5)
# robot_brain = RobotBrain(robot=robot, controller=sim_controller, speed=0.3, turning_speed=45)
robot_brain = EcoDisasterBrain(robot=robot, controller=sim_controller, speed=0.05, turning_speed=10)
robot_brain.add_sensor(SimulatedVision360(ExteriorTheWorld))

renderer = WorldRenderer(x_res=1000, y_res=1000, world_scale=400) # default 0,0 is centre of screen
renderer.update()
print("Getting ready...")
time.sleep(1) # wait for the world to load

print("Running...")
frame_time = 1/60.0 # aim for 60 fps simulation
while running:
    now = time.time();

    robot_brain.process()
    #robot_brain.simulate(dt)

    renderer.update(ExteriorTheWorld)
    #renderer.update(ExteriorTheWorld, robot_brain.TheWorld) # see the world as it is and as the robot sees it
    #renderer.update(robot_brain.TheWorld) # see the world as the robot sees it
    if renderer.running == False:
        running = False
    to_sleep = frame_time - (time.time() - now)
    if to_sleep > 0:
        time.sleep(to_sleep)

sim_controller.stop(exiting=True)
