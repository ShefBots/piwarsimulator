#!/usr/bin/env python3
import argparse
import importlib
import time
import numpy as np
from os import listdir
from signal import signal, SIGINT
from world.ObjectType import ObjectType
from world.WorldObject import WorldObject


# TODO brains: ecodisaster
# TODO held items
# TODO classes for real hardware
# TODO sensor_simulation (real control only) mode and control mode (all real)

running = True  # state of simulator
ctrlc_count = 0  # if hitting 3 try and sys.exit
target_frame_time = 1 / 60.0  # aim for 60 fps simulation/processing


def sigint_handler(signal_received, frame):
    """trap/handle ctrl c"""
    global running  # why does this need to be global?
    global ctrlc_count
    print("SIGINT or CTRL-C detected")
    running = False
    ctrlc_count += 1
    if ctrlc_count == 3:
        exit()


signal(SIGINT, sigint_handler)

# list of available brains for argument list
brains = sorted([s[:-3] for s in listdir("brains/") if "Brain.py" in s])
maps = sorted([s[:-3] for s in listdir("world/") if "Map.py" in s])

parser = argparse.ArgumentParser(
    description="""Simulator/controller for ShefBots Mark 1b for PiWars 2024.
    Press SPACE to engage manual control, WASD/Arrow keys for strafe, and QE for rotate."""
)
parser.add_argument(
    "--brain",
    help="robot brain/challenge (default RobotBrain)",
    # default="RobotBrain",
    # default="MinesweeperBrain",
    # default="MazeBrain",
    default="LineFollowingBrain",
    choices=brains,
)
parser.add_argument(
    "--map",
    help=f"map (default {maps[0]})",
    # default=maps[0],
    # default="MinesweeperMap",
    # default="EscapeRouteMap",
    default="LavaPalavaMap",
    choices=maps
    # "--map", help=f"map (default {maps[0]})", default="LavaPalavaMap", choices=maps
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
    from sensors.SimulatedLineOfSight import SimulatedLineOfSight
    from sensors.SimulatedVision360 import SimulatedVision360
elif args.mode == "sensor_simulation":
    # TODO real hardware
    from sensors.SimulatedLineOfSight import SimulatedLineOfSight
    from sensors.SimulatedVision360 import SimulatedVision360
elif args.mode == "control":
    # TODO real hardware
    pass

# provides robot geometry
robot = WorldObject(
    object_type=ObjectType.ROBOT,
    x=0,
    y=0,
    w=0.18,
    h=0.235,
    angle=0
    # object_type=ObjectType.ROBOT, x=0.1, y=-0.3, w=0.18, h=0.235, angle=20
)  # units metres and degress

if args.mode == "simulation" or args.mode == "sensor_simulation":
    # objects for rendering
    # the order this is constructed in is also the rendering order...
    ExteriorTheWorld = []
    ExteriorTheWorld.append(robot)  # this should always be index 0!

    # import the map
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
# robot_brain = brain(robot=robot, controller=controller, speed=0.05, turning_speed=10)
robot_brain = brain(robot=robot, controller=controller, speed=0.3, turning_speed=60)

print("Attaching sensors...")
if args.mode == "simulation" or args.mode == "sensor_simulation":
    if args.rendering == "true":
        robot_brain.add_sensor(Keyboard(robot_brain.speed, robot_brain.turning_speed))
    robot_brain.add_sensor(SimulatedLineOfSight(ExteriorTheWorld, robot_brain, 0))
    robot_brain.add_sensor(SimulatedLineOfSight(ExteriorTheWorld, robot_brain, -90))
    robot_brain.add_sensor(SimulatedLineOfSight(ExteriorTheWorld, robot_brain, 90))
    robot_brain.add_sensor(SimulatedVision360(ExteriorTheWorld, robot_brain))
else:
    # TODO real hardware
    pass

if args.rendering == "true":
    from world.WorldRenderer import *

    # default 0,0 is centre of screen
    renderer = WorldRenderer(
        x_res=900,
        y_res=900,
        num_worlds=1 if args.mode == "control" else 2,
    )
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
            renderer.update(
                Worlds=robot_brain.TheWorld
            )  # see the world as the robot sees it
        else:
            renderer.update(
                Worlds=[ExteriorTheWorld, robot_brain.TheWorld],
                Sensors=[
                    [s.fov for s in robot_brain.sensors],
                    [s.outline for s in robot_brain.sensors],
                ],
            )  # see the world as it is and as the robot sees it

        if renderer.running == False:
            running = False

    to_sleep = target_frame_time - (time.time() - now)
    if to_sleep > 0:
        time.sleep(to_sleep)

print("Quitting...")
controller.stop(exiting=True)
