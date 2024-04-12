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

# TODO classes for real hardware
# TODO      launcher control?
# TODO update readme
# TODO nice program end points

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
TURNING_SPEED = 45

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
    # default="CheesedEcoDisasterBrain",
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
    choices=["simulation", "sensor_simulation", "control"],  # , "control_simulation"
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
parser.add_argument(
    "--attachment",
    help="choose an attachment (default none)",
    default="none",
    # default="gripper",
    choices=["none", "gripper", "launcher"],
)
args = parser.parse_args()

# imports for hardware etc based on settings
if args.rendering == "true":
    from sensors.Keyboard import Keyboard
if args.radio == "true":
    from sensors.RadioControl import RadioControl
if args.mode == "simulation":
    # fake control hardware, fake sensors
    from controllers.SimulatedMovementController import SimulatedMovementController
    from controllers.SimulatedGripperController import SimulatedGripperController
    from sensors.SimulatedLineOfSight import SimulatedLineOfSight

    if args.simplevision == "false":
        from sensors.SimulatedVision360 import SimulatedVision360 as SimulatedVision
    else:
        from sensors.SimulatedReducedVision import (
            SimulatedReducedVision as SimulatedVision,
        )
elif args.mode == "sensor_simulation":
    # real control hardware, fake sensors
    from controllers.SimulatedMovementController import SimulatedMovementController
    from controllers.SimulatedGripperController import SimulatedGripperController
    from controllers.MovementController import MovementController
    from controllers.GripperController import GripperController
    from sensors.SimulatedLineOfSight import SimulatedLineOfSight

    if args.simplevision == "false":
        from sensors.SimulatedVision360 import SimulatedVision360 as SimulatedVision
    else:
        from sensors.SimulatedReducedVision import (
            SimulatedReducedVision as SimulatedVision,
        )
elif args.mode == "control":
    # real control hardware, real sensors
    from controllers.MovementController import MovementController
    from controllers.GripperController import GripperController
    from sensors.DistanceSensor import DistanceSensor
elif args.mode == "control_simulation":
    # TODO fake control hardware, real sensors
    from controllers.SimulatedMovementController import SimulatedMovementController
    from controllers.SimulatedGripperController import SimulatedGripperController

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
print("Loading robot controllers...")
attachment_controller = None
if args.mode == "simulation":
    controller = SimulatedMovementController(robot)
    if args.attachment == "gripper":
        attachment_controller = SimulatedGripperController(robot)
elif args.mode == "sensor_simulation":
    try:
        real_controller = MovementController(serial_instances)
        if args.attachment == "gripper":
            real_attachment_controller = GripperController(robot, serial_instances)
    except Exception as e:
        print(f"Caught error: {e}")
        print(traceback.format_exc())
        print("Connection to real motor driver failed, not using")
        real_controller = None
    controller = SimulatedMovementController(
        robot, secondary_controller=real_controller
    )
    if args.attachment == "gripper":
        attachment_controller = SimulatedGripperController(
            robot, secondary_controller=real_attachment_controller
        )
elif args.mode == "control":
    try:
        # controller = SimulatedMovementController(robot)
        controller = MovementController(serial_instances)
        if args.attachment == "gripper":
            attachment_controller = GripperController(robot, serial_instances)
        elif args.attachment == "launcher":
            # TODO attempt to init real launcher controller
            attachment_controller = None
    except Exception as e:
        running = False
        controller = None
        print(f"Caught error: {e}")
        print(traceback.format_exc())
elif args.mode == "control_simulation":
    # TODO
    pass


print(f"Loading {args.brain}...")
brain = getattr(importlib.import_module("brains." + args.brain), args.brain)
robot_brain = brain(
    robot=robot,
    controller=controller,
    speed=ROBOT_SPEED,
    turning_speed=TURNING_SPEED,
    attachment_controller=attachment_controller,
)
if args.mode == "simulation" or args.mode == "sensor_simulation":
    # this works because lists are references
    controller.holding = robot_brain.holding
if not args.attachment == "none" and not attachment_controller is None:
    print("Attaching brain to attachment")
    attachment_controller.set_brain(robot_brain)

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
        # TODO put in the correct offset values (how far inside the edge of the robot they are)
        robot_brain.add_sensor(DistanceSensor(serial_instances, robot, 0, 1))
        robot_brain.add_sensor(DistanceSensor(serial_instances, robot, 90, 0))
        # TODO robot_brain.add_sensor(DistanceSensor(serial_instances, robot, 180, 0))
        robot_brain.add_sensor(DistanceSensor(serial_instances, robot, 270, 2))
        # TODO add_sensor vision system
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

start_time = time.monotonic()
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
                    names=["Robot Perception"],
                    robot_brain=robot_brain,
                )  # see the world as the robot sees it
            else:
                renderer.update(
                    Worlds=[ExteriorTheWorld, robot_brain.TheWorld],
                    Sensors=[
                        [s.fov for s in robot_brain.sensors],
                        [s.outline for s in robot_brain.sensors],
                    ],
                    names=["Simulation Environment", "Robot Perception"],
                    robot_brain=robot_brain,
                )  # see the world as it is and as the robot sees it

            if renderer.running == False:
                running = False
        except Exception as e:
            running = False
            print(f"Caught error: {e}")
            print(traceback.format_exc())

    # quit after a few seconds
    # if time.monotonic() - start_time > 10:
    #     running = False

    if robot_brain.running == False:
        running = False

    to_sleep = target_frame_time - (time.monotonic() - now)
    if to_sleep > 0:
        time.sleep(to_sleep)

print("Quitting...")
if not controller is None:
    controller.stop(exiting=True)
if not attachment_controller is None:
    attachment_controller.stop(exiting=True)
