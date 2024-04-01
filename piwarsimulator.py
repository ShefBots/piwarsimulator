#!/usr/bin/env python3
import argparse
import importlib
import time
import traceback
import numpy as np
from os import listdir
from signal import signal, SIGINT

# I hate altering paths, but without this piwarsengine dies
importlib.import_module("sys").path.append("../piwarsengine")

import util
from world.ObjectType import ObjectType
from world.WorldObject import WorldObject


# TODO brains: ecodisaster - gets stuck when arriving at zone
# TODO classes for real hardware
# TODO sensor_simulation (real control only) mode and control mode (all real)
# TODO update readme
# TODO launcher control?

# note this runs about 6 times slower on the Pi under Python 3.7?
# FPS to TPS for thoughts per second ? :)

# to profile:
# pip install line-profiler[all] snakeviz
# python -m cProfile -o out.prof piwarsimulator.py
# python -m snakeviz out.prof
# from line_profiler import profile
# @profile
# python -m kernprof -lvr piwarsimulator.py

running = True  # state of simulator
ctrlc_count = 0  # if hitting 3 try and sys.exit
target_frame_time = 1 / 60.0  # aim for 60 fps simulation/processing

# ROBOT_SPEED = 0.05
# TURNING_SPEED = 10
ROBOT_SPEED = 0.3
TURNING_SPEED = 30

SERIAL_PATTERN = "/dev/ttyACM*"  # serial ports to scan for hardware


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
    default="RobotBrain",
    # default="MinesweeperBrain",
    # default="MazeBrain",
    # default="LineFollowingBrain",
    # default="EcoDisasterBrain",
    choices=brains,
)
parser.add_argument(
    "--map",
    help=f"map (default {maps[0]})",
    # default=maps[0],
    # default="MinesweeperMap",
    default="EscapeRouteMap",
    # default="LavaPalavaMap",
    # default="SimpleEcoDisasterMap",
    # default="RandomEcoDisasterMap",
    choices=maps,
    # "--map", help=f"map (default {maps[0]})", default="LavaPalavaMap", choices=maps
)
parser.add_argument(
    "--mode",
    help="operation mode (default simulation)",
    default="simulation",
    # default="control",
    choices=["simulation", "sensor_simulation", "control"],
)
parser.add_argument(
    "--radio",
    help="use radio receiever for control (default false)",
    default="false",
    choices=["true", "false"],
)
parser.add_argument(
    "--rendering",
    help="render world on screen (default true)",
    default="true",
    choices=["true", "false"],
)
parser.add_argument(
    "--simplevision",
    help="rely on simpler vision system (default false)",
    default="false",
    # default="true",
    choices=["true", "false"],
)
args = parser.parse_args()

# imports for hardware etc based on settings
if args.rendering == "true":
    from sensors.Keyboard import Keyboard
if args.radio == "true":
    from sensors.RadioControl import RadioControl
if args.mode == "simulation":
    from controllers.SimulatedMovementController import SimulatedMovementController
    from sensors.SimulatedLineOfSight import SimulatedLineOfSight

    if args.simplevision == "false":
        from sensors.SimulatedVision360 import SimulatedVision360 as SimulatedVision
    else:
        from sensors.SimulatedReducedVision import (
            SimulatedReducedVision as SimulatedVision,
        )
elif args.mode == "sensor_simulation":
    from controllers.SimulatedMovementController import SimulatedMovementController
    from controllers.MovementController import MovementController
    from sensors.SimulatedLineOfSight import SimulatedLineOfSight

    if args.simplevision == "false":
        from sensors.SimulatedVision360 import SimulatedVision360 as SimulatedVision
    else:
        from sensors.SimulatedReducedVision import (
            SimulatedReducedVision as SimulatedVision,
        )
elif args.mode == "control":
    from controllers.MovementController import MovementController
    # from controllers.SimulatedMovementController import SimulatedMovementController
    from sensors.DistanceSensor import DistanceSensor
    # TODO real hardware (sensors)

# do serial stuff if needed
if not args.mode == "simulation":
    print("Preparing serial comms...")
    # we want controller input even if simulating
    # or otherwise it's someting with hardware and we will want serial

    # Find serial ports matching the pattern
    port_list = util.find_serial_ports(SERIAL_PATTERN)

    if not port_list:
        print(f"No serial ports found matching the pattern '{SERIAL_PATTERN}'.")
        serial_instances = {}
    else:
        # Create instances for each serial port
        serial_instances = util.create_serial_instances(port_list)


# provides robot geometry
robot = WorldObject(
    object_type=ObjectType.ROBOT,
    x=0,
    y=0,
    # w=0.18,  # without bumpers
    w=0.215,  # with bumpers
    h=0.235,
    angle=0,
    # object_type=ObjectType.ROBOT, x=0.1, y=-0.3, w=0.18, h=0.235, angle=20
    # object_type=ObjectType.ROBOT, x=0.1, y=-0.3, w=0.18, h=0.235, angle=20
    # object_type=ObjectType.ROBOT, x=0, y=0, w=0.18, h=0.235, angle=1
)  # units metres and degress

if args.mode == "simulation" or args.mode == "sensor_simulation":
    # objects for rendering
    # the order this is constructed in is also the rendering order...
    ExteriorTheWorld = [robot]  # robot should always be index 0!

    # import the map
    print(f"Loading map {args.map}...")
    map = importlib.import_module("world." + args.map)
    robot.center = map.START_LOCATION
    map_func = getattr(map, args.map)
    map_func(ExteriorTheWorld)

# logic for the robot
print("Loading robot controller...")
if args.mode == "simulation":
    controller = SimulatedMovementController(robot)
elif args.mode == "sensor_simulation":
    try:
        real_controller = MovementController(serial_instances)
    except Exception as e:
        print(f"Caught error: {e}")
        print(traceback.format_exc())
        print("Connection to real motor driver failed, not using")
        real_controller = None
    controller = SimulatedMovementController(
        robot, secondary_controller=real_controller
    )
else:
    try:
        # controller = SimulatedMovementController(robot)
        controller = MovementController(serial_instances)
    except Exception as e:
        running = False
        controller = None
        print(f"Caught error: {e}")
        print(traceback.format_exc())


print(f"Loading {args.brain}...")
brain = getattr(importlib.import_module("brains." + args.brain), args.brain)
robot_brain = brain(
    robot=robot, controller=controller, speed=ROBOT_SPEED, turning_speed=TURNING_SPEED
)
if args.mode == "simulation" or args.mode == "sensor_simulation":
    # this works because lists are references
    controller.holding = robot_brain.holding

print("Attaching sensors...")
if args.rendering == "true":
    robot_brain.add_sensor(Keyboard(robot_brain.speed, robot_brain.turning_speed))
if args.mode == "simulation" or args.mode == "sensor_simulation":
    # forward, right, behind, left
    robot_brain.add_sensor(SimulatedLineOfSight(ExteriorTheWorld, robot_brain, 0))
    robot_brain.add_sensor(SimulatedLineOfSight(ExteriorTheWorld, robot_brain, 90))
    robot_brain.add_sensor(SimulatedLineOfSight(ExteriorTheWorld, robot_brain, 180))
    robot_brain.add_sensor(SimulatedLineOfSight(ExteriorTheWorld, robot_brain, 270))
    robot_brain.add_sensor(SimulatedVision(ExteriorTheWorld, robot_brain))
else:
    try:
        # TODO real hardware
        # 4x line of sight
        # vision system
        robot_brain.add_sensor(DistanceSensor(serial_instances, robot, 270))  # left
    except Exception as e:
        running = False
        print(f"Caught error: {e}")
        print(traceback.format_exc())
if args.radio == "true":
    try:
        robot_brain.add_sensor(
            RadioControl(robot_brain.speed, robot_brain.turning_speed)
        )
    except Exception as e:
        running = False
        print(f"Caught error: {e}")
        print(traceback.format_exc())

if args.rendering == "true" and running == True:
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

print(f"Running... ({running})")
while running:
    now = time.monotonic()

    robot_brain.process()

    if args.rendering == "true":
        try:
            if args.mode == "control":
                renderer.update(
                    Worlds=[robot_brain.TheWorld],
                    # yes this is a list in a list. deal with it.
                    Sensors=[[s.outline for s in robot_brain.sensors]],
                    robot_brain=robot_brain,
                )  # see the world as the robot sees it
            else:
                renderer.update(
                    Worlds=[ExteriorTheWorld, robot_brain.TheWorld],
                    Sensors=[
                        [s.fov for s in robot_brain.sensors],
                        [s.outline for s in robot_brain.sensors],
                    ],
                    robot_brain=robot_brain,
                )  # see the world as it is and as the robot sees it

            if renderer.running == False:
                running = False
        except Exception as e:
            running = False
            print(f"Caught error: {e}")
            print(traceback.format_exc())

    if robot_brain.running == False:
        running = False

    to_sleep = target_frame_time - (time.monotonic() - now)
    if to_sleep > 0:
        time.sleep(to_sleep)

print("Quitting...")
if not controller is None:
    controller.stop(exiting=True)
