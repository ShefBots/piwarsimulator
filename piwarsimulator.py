#!/usr/bin/env python3
import argparse
import importlib
import time
import numpy as np
from os import listdir
from signal import signal, SIGINT
from sys import exit

# list of available brains for argument list
brains = sorted([s[:-3] for s in listdir('brains/') if "Brain.py" in s])
maps = sorted([s[:-3] for s in listdir('world/') if "Map.py" in s])

parser = argparse.ArgumentParser(description="Simulator for ShefBots Mark 1b for PiWars 2024")
parser.add_argument("--brain", help="robot brain/challenge (default RobotBrain)", default="RobotBrain", choices=brains)
parser.add_argument("--map", help=f"map (default {maps[0]})", default=maps[0], choices=maps)
parser.add_argument("--mode", help="operation mode (default simulation)", default="simulation", choices=['simulation', 'reality'])
parser.add_argument("--rendering", help="render world on screen (default true)", default='true', choices=['true', 'false'])
args = parser.parse_args()


# from brains.RobotBrain import RobotBrain
# from brains.EcoDisasterBrain import EcoDisasterBrain

from controllers.SimulatedMovementController import SimulatedMovementController
from sensors.SimulatedVision360 import SimulatedVision360

from world.WorldObject import *
from world.WorldRenderer import *

# TODO add log class, use that for output instead of print
# TODO for now everything is simulated
# TODO proper dimensions and whatnot

# TODO option to select simulated/real
# TODO option for rendering
# TODO side by side output of world and sensor

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

# import the map
print(f"Loading map {args.map}...")
map = getattr(importlib.import_module('world.'+args.map), args.map)
map(ExteriorTheWorld)

# logic for the robot
print("Loading robot controller...")
if args.mode == 'simulation':
    sim_controller = SimulatedMovementController(robot)
else:
    raise Exception("No real hardware controller yet")
# sim_controller.set_plane_velocity([0, 0.05])
# sim_controller.set_angular_velocity(5)
# robot_brain = RobotBrain(robot=robot, controller=sim_controller, speed=0.3, turning_speed=45)
# robot_brain = EcoDisasterBrain(robot=robot, controller=sim_controller, speed=0.05, turning_speed=10)

print(f"Loading {args.brain}...")
brain = getattr(importlib.import_module('brains.'+args.brain), args.brain)
robot_brain = brain(robot=robot, controller=sim_controller, speed=0.05, turning_speed=10)
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
