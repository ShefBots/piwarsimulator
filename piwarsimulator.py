#!/usr/bin/env python3
import argparse
import importlib
import time
import numpy as np
from os import listdir
from signal import signal, SIGINT
from world.ObjectType import ObjectType
from world.WorldObject import WorldObject


# TODO proper dimensions/sizes of items
# TODO control mode for real hardware
# TODO sensor_simulation mode for real hardware but simulated sensor input
# TODO side by side output of world and sensor
# TODO maps for the different scenarios

running = True  # state of simulator
frame_time = 1 / 60.0  # aim for 60 fps simulation/processing


def sigint_handler(signal_received, frame):
    """trap/handle ctrl c"""
    global running  # why does this need to be global?
    print("SIGINT or CTRL-C detected")
    running = False


signal(SIGINT, sigint_handler)

# list of available brains for argument list
brains = sorted([s[:-3] for s in listdir("brains/") if "Brain.py" in s])
maps = sorted([s[:-3] for s in listdir("world/") if "Map.py" in s])

parser = argparse.ArgumentParser(
    description="Simulator/controller for ShefBots Mark 1b for PiWars 2024"
)
parser.add_argument(
    "--brain",
    help="robot brain/challenge (default RobotBrain)",
    default="RobotBrain",
    choices=brains,
)
parser.add_argument(
    "--map", help=f"map (default {maps[0]})", default=maps[0], choices=maps
)
parser.add_argument(
    "--mode",
    help="operation mode (default simulation)",
    default="simulation",
    choices=["simulation", "sensor_simulation", "control"],
)
parser.add_argument(
    "--rendering",
    help="render world on screen (default true)",
    default="true",
    choices=["true", "false"],
)
args = parser.parse_args()

if args.rendering == "true":
    from sensors.Keyboard import Keyboard

if args.mode == "simulation":
    from controllers.SimulatedMovementController import SimulatedMovementController
    from sensors.SimulatedVision360 import SimulatedVision360
elif args.mode == "sensor_simulation":
    # TODO real hardware, simulated sensors
    pass
elif args.mode == "control":
    # TODO real hardware
    pass

# the order this is constructed in is also the rendering order...
ExteriorTheWorld = []
robot = WorldObject(
    object_type=ObjectType.ROBOT, x=0, y=0, w=0.12, h=0.15, angle=0
)  # units metres and degress
ExteriorTheWorld.append(robot)  # this should always be index 0!

# import the map
if args.mode == "simulation" or args.mode == "sensor_simulation":
    print(f"Loading map {args.map}...")
    map = getattr(importlib.import_module("world." + args.map), args.map)
    map(ExteriorTheWorld)

# logic for the robot
print("Loading robot controller...")
if args.mode == "simulation":
    controller = SimulatedMovementController(robot)
else:
    raise Exception("No real hardware controller yet")

print(f"Loading {args.brain}...")
brain = getattr(importlib.import_module("brains." + args.brain), args.brain)
robot_brain = brain(robot=robot, controller=controller, speed=0.05, turning_speed=10)

print("Attaching sensors...")
if args.mode == "simulation" or args.mode == "sensor_simulation":
    if args.rendering == "true":
        print("   Keyboard")
        robot_brain.add_sensor(Keyboard(robot_brain.speed, robot_brain.turning_speed))
    if args.brain == "EcoDisasterBrain":
        print("   360 vision")
        robot_brain.add_sensor(SimulatedVision360(ExteriorTheWorld))
else:
    # TODO real hardware
    pass

if args.rendering == "true":
    from world.WorldRenderer import *

    renderer = WorldRenderer(
        x_res=1000, y_res=1000, world_scale=400
    )  # default 0,0 is centre of screen
    renderer.update()

print("Waiting...")
time.sleep(1)  # wait for things to settle

# controller.set_plane_velocity([0, 0.15])
# controller.set_angular_velocity(12)

print("Running...")
while running:
    now = time.time()

    robot_brain.process()
    # robot_brain.simulate(dt)

    if args.rendering == "true":
        if args.mode == "control":
            renderer.update(robot_brain.TheWorld)  # see the world as the robot sees it
        else:
            renderer.update(
                ExteriorTheWorld, robot_brain.TheWorld
            )  # see the world as it is and as the robot sees it

        if renderer.running == False:
            running = False

    to_sleep = frame_time - (time.time() - now)
    if to_sleep > 0:
        time.sleep(to_sleep)

print("Quitting...")
controller.stop(exiting=True)
