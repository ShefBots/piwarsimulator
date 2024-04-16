#!/usr/bin/env python3
import argparse
import importlib
import time
import traceback
import asyncio
import numpy as np
from os import listdir
from signal import signal, SIGINT
import util
from world.ObjectType import ObjectType
from world.WorldObject import WorldObject

# TODO classes for real hardware
# TODO      launcher control?
# TODO nice program end points
# TODO stop button

# note this runs about 6 times slower on the Pi under Python 3.7?
# FPS to TPS for thoughts per second ? :)

# to profile:
# pip install line-profiler[all] snakeviz
# python -m cProfile -o out.prof piwarsimulator.py
# python -m snakeviz out.prof
# from line_profiler import profile
# @profile
# python -m kernprof -lvr piwarsimulator.py

MIN_FRAME_RATE = 10.0
MAX_FRAME_RATE = 120.0
DEFAULT_FRAME_RATE = 60.0  # aim for 60 fps simulation/processing by default

MIN_ROBOT_SPEED = 0.1
MAX_ROBOT_SPEED = 0.6
MIN_TURNING_SPEED = 22.5
MAX_TURNING_SPEED = 90.0

DEFAULT_ROBOT_SPEED = 0.3
DEFAULT_TURNING_SPEED = 45

SERIAL_PATTERN = "/dev/ttyACM*"  # serial ports to scan for hardware

# Time of flight sensor angles, indexes, and offsets
# forward, right, behind, left
HIGH_TOFS = [(0, 1, 0.06), (90, 0, 0.05), (180, 3, 0.02), (270, 2, 0.03)]
LOW_TOFS = [(0, 1, 0.03), (90, 0, 0.01), (270, 2, 0.01)]
TOF_POSITIONS = {"high": HIGH_TOFS, "low": LOW_TOFS}

# list of available brains for argument list
brains = sorted([s[:-3] for s in listdir("brains/") if "Brain.py" in s])
maps = sorted([s[:-3] for s in listdir("world/") if "Map.py" in s])

parser = argparse.ArgumentParser(
    description="""Simulator/controller for ShefBots Mark 1b for PiWars 2024.
    Press SPACE to engage manual control, WASD/Arrow keys for strafe, and QE for rotate.
    G will activate the gripper if attached."""
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
    default=maps[0],
    # default="MinesweeperMap",
    # default="EscapeRouteMap",
    # default="LavaPalavaMap",
    # default="SimpleEcoDisasterMap",
    # default="RandomEcoDisasterMap",
    choices=maps,
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
    "--vision-mode",
    help="Which vision system to use. 'simple' uses simple vision. 'simulated360' simulates perfect 360 vision. 'socketed360-*' connects to a socket either running at localhost or the protocol-defined remote host (usually 192.168.22.1) (default 'simulated360')",
    default="simulated360",
    choices=["simple", "simulated360", "socketed360-local", "socketed360-remote"]
)
parser.add_argument(
    "--attachment",
    help="choose an attachment (default none)",
    default="none",
    # default="gripper",
    choices=["none", "gripper", "launcher"],
)
parser.add_argument(
    "--beam",
    help="is the gripper equipped with the beam sensor (default false)",
    default="false",
    # default="true",
    choices=["true", "false"],
)
parser.add_argument(
    "--robot_speed",
    help=f"the top robot speed (min={MIN_ROBOT_SPEED}, max={MAX_ROBOT_SPEED}, default {DEFAULT_ROBOT_SPEED})",
    type=lambda s: util.check_in_range(s, MIN_ROBOT_SPEED, MAX_ROBOT_SPEED),
    default=DEFAULT_ROBOT_SPEED,
)
parser.add_argument(
    "--turning_speed",
    help=f"the top robot speed (min={MIN_TURNING_SPEED}, max={MAX_TURNING_SPEED}, default {DEFAULT_TURNING_SPEED})",
    type=lambda s: util.check_in_range(s, MIN_TURNING_SPEED, MAX_TURNING_SPEED),
    default=DEFAULT_TURNING_SPEED,
)
parser.add_argument(
    "--frame_rate",
    help=f"number of times update (call process()) per second (min={MIN_FRAME_RATE}, max={MAX_FRAME_RATE}, default {DEFAULT_FRAME_RATE})",
    type=lambda s: util.check_in_range(s, MIN_FRAME_RATE, MAX_FRAME_RATE),
    default=DEFAULT_FRAME_RATE,
)
parser.add_argument(
    "--tof_position",
    help="tof sensors are mounted low or high (default high)",
    default=list(TOF_POSITIONS.keys())[0],
    # default="low",
    choices=TOF_POSITIONS.keys(),
)
args = parser.parse_args()

if util.is_true(args.rendering):
    from world.WorldRenderer import * # This is now imported up here, 

async def main_loop():
    global args

    running = True  # state of simulator
    ctrlc_count = 0  # if hitting 3 try and sys.exit

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



    # imports for hardware etc based on settings
    if util.is_true(args.rendering):
        from sensors.Keyboard import Keyboard
    if util.is_true(args.radio):
        from sensors.RadioControl import RadioControl
    if args.mode.lower() == "simulation":
        # fake control hardware, fake sensors
        from controllers.SimulatedMovementController import SimulatedMovementController
        from controllers.SimulatedGripperController import SimulatedGripperController
        from sensors.SimulatedLineOfSight import SimulatedLineOfSight
        from sensors.SimulatedBeamSensor import SimulatedBeamSensor

        match args.vision_mode:
            case "simple":
                from sensors.SimulatedReducedVision import (
                    SimulatedReducedVision as SimulatedVision,
                )
            case "simulated360":
                from sensors.SimulatedVision360 import SimulatedVision360 as SimulatedVision
            case "socketed360-local" | "socketed360-remote":
                from sensors.Vision360 import Vision360 as SimulatedVision
                
    elif args.mode.lower() == "sensor_simulation":
        # real control hardware, fake sensors
        from controllers.SimulatedMovementController import SimulatedMovementController
        from controllers.SimulatedGripperController import SimulatedGripperController
        from controllers.MovementController import MovementController
        from controllers.GripperController import GripperController
        from sensors.SimulatedLineOfSight import SimulatedLineOfSight
        from sensors.SimulatedBeamSensor import SimulatedBeamSensor

        match args.vision_mode:
            case "simple":
                from sensors.SimulatedReducedVision import (
                    SimulatedReducedVision as SimulatedVision,
                )
            case "simulated360":
                from sensors.SimulatedVision360 import SimulatedVision360 as SimulatedVision
            case "socketed360-local" | "socketed360-remote":
                from sensors.Vision360 import Vision360 as SimulatedVision
    elif args.mode.lower() == "control":
        # real control hardware, real sensors
        from controllers.MovementController import MovementController
        from controllers.GripperController import GripperController
        from controllers.LauncherController import LauncherController
        from sensors.DistanceSensor import DistanceSensor
    elif args.mode.lower() == "control_simulation":
        # TODO fake control hardware, real sensors
        from controllers.SimulatedMovementController import SimulatedMovementController
        from controllers.SimulatedGripperController import SimulatedGripperController

    # do serial stuff if needed
    if not args.mode.lower() == "simulation":
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
    )  # units metres and degress

    if args.mode.lower() == "simulation" or args.mode.lower() == "sensor_simulation":
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
    if args.mode.lower() == "simulation":
        controller = SimulatedMovementController(robot)
        if args.attachment.lower() == "gripper":
            attachment_controller = SimulatedGripperController(robot)
    elif args.mode.lower() == "sensor_simulation":
        try:
            real_controller = MovementController(serial_instances)
            if args.attachment.lower() == "gripper":
                real_attachment_controller = GripperController(robot, serial_instances)
        except Exception as e:
            print(f"Caught error: {e}")
            print(traceback.format_exc())
            print("Connection to real motor driver failed, not using")
            real_controller = None
        controller = SimulatedMovementController(
            robot, secondary_controller=real_controller
        )
        if args.attachment.lower() == "gripper":
            attachment_controller = SimulatedGripperController(
                robot, secondary_controller=real_attachment_controller
            )
    elif args.mode.lower() == "control":
        try:
            controller = MovementController(serial_instances)
            if args.attachment.lower() == "gripper":
                attachment_controller = GripperController(robot, serial_instances)
            elif args.attachment.lower() == "launcher":
                attachment_controller = LauncherController(serial_instances)
        except Exception as e:
            running = False
            controller = None
            print(f"Caught error: {e}")
            print(traceback.format_exc())
    elif args.mode.lower() == "control_simulation":
        # TODO
        pass


    print(f"Loading {args.brain}...")
    brain = getattr(importlib.import_module("brains." + args.brain), args.brain)
    robot_brain = brain(
        robot=robot,
        controller=controller,
        speed=args.robot_speed,
        turning_speed=args.turning_speed,
        attachment_controller=attachment_controller,
    )
    if args.mode.lower() == "simulation" or args.mode.lower() == "sensor_simulation":
        # this works because lists are references
        controller.holding = robot_brain.holding
    if not args.attachment.lower() == "none" and not attachment_controller is None:
        print("Attaching brain to attachment")
        attachment_controller.set_brain(robot_brain)

    print("Attaching sensors...")
    if util.is_true(args.rendering):
        robot_brain.add_sensor(Keyboard(robot_brain.speed, robot_brain.turning_speed))
    if args.mode.lower() == "simulation" or args.mode.lower() == "sensor_simulation":
        # add the simulated time of flight sensors
        for _, v in enumerate(TOF_POSITIONS[args.tof_position.lower()]):
            robot_brain.add_sensor(
                SimulatedLineOfSight(ExteriorTheWorld, robot_brain, v[0])
            )
        # Add the vision sensor, only giving it the sim data if it's actually a simulated sensor (not the socketed one)
        robot_brain.add_sensor(SimulatedVision(ExteriorTheWorld, robot_brain) if "socketed360" not in args.vision_mode else SimulatedVision(remote_connect = args.vision_mode == "socketed360-remote"))
        if args.attachment.lower() == "gripper" and util.is_true(args.beam):
            robot_brain.add_sensor(SimulatedBeamSensor(ExteriorTheWorld))
    else:
        try:
            # add the time of flight sensors
            for _, v in enumerate(TOF_POSITIONS[args.tof_position.lower()]):
                robot_brain.add_sensor(
                    DistanceSensor(serial_instances, robot, v[0], v[1], offset=v[2])
                )
            # TODO add_sensor vision system
        except Exception as e:
            running = False
            print(f"Caught error: {e}")
            print(traceback.format_exc())
    if util.is_true(args.radio):
        try:
            robot_brain.add_sensor(
                RadioControl(robot_brain.speed, robot_brain.turning_speed)
            )
        except Exception as e:
            running = False
            print(f"Caught error: {e}")
            print(traceback.format_exc())

    if util.is_true(args.rendering) and running == True:
#        from world.WorldRenderer import *

        # default 0,0 is centre of screen
        renderer = WorldRenderer(
            x_res=900,
            y_res=900,
            num_worlds=1 if args.mode.lower() == "control" else 2,
            target_fps=args.frame_rate,
        )
        renderer.update()

#async def main_loop():
#    global running

    print("Waiting...")
    await asyncio.sleep(1) # Wait for things to settle (terrifying)
    #time.sleep(1)  # wait for things to settle

    target_frame_time = 1.0 / args.frame_rate
    start_time = time.monotonic()
    print(f"Running... ({running})")
    while running:
        now = time.monotonic()

        robot_brain.process()

        if util.is_true(args.rendering):
            try:
                if args.mode.lower() == "control":
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
            await asyncio.sleep(to_sleep)
            #time.sleep(to_sleep)

    print("Quitting...")
    if not controller is None:
        controller.stop(exiting=True)
    if not attachment_controller is None:
        attachment_controller.stop(exiting=True)

loop = asyncio.get_event_loop()
loop.run_until_complete(main_loop())
loop.close()
