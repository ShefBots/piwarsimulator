# piwarsimulator
Robot simulator to work on PiWars logic. This is a work in progress, revivified for 2024. The software for most of the challenges is complete, with Eco-Disaster being the most incomplete. Manual robot control can be engaged by pressing spacebar and then driving with WASD/arrow for strafe and QE for rotate.

```
$ ./piwarsimulator.py -h
pygame 2.5.2 (SDL 2.28.3, Python 3.11.7)
Hello from the pygame community. https://www.pygame.org/contribute.html
usage: piwarsimulator.py [-h] [--brain {EcoDisasterBrain,LineFollowingBrain,MazeBrain,MinesweeperBrain,RobotBrain}] [--map {EscapeRouteMap,LavaPalavaMap,MinesweeperMap,SimpleEcoDisasterMap}]
                         [--mode {simulation,sensor_simulation,control}] [--rendering {true,false}]

Simulator/controller for ShefBots Mark 1b for PiWars 2024. Press SPACE to engage manual control, WASD/Arrow keys for strafe, and QE for rotate.

options:
  -h, --help            show this help message and exit
  --brain {EcoDisasterBrain,LineFollowingBrain,MazeBrain,MinesweeperBrain,RobotBrain}
                        robot brain/challenge (default RobotBrain)
  --map {EscapeRouteMap,LavaPalavaMap,MinesweeperMap,SimpleEcoDisasterMap}
                        map (default EscapeRouteMap)
  --mode {simulation,sensor_simulation,control}
                        operation mode (default simulation)
  --rendering {true,false}
                        render world on screen (default true)
```

## Getting started

The first step is to create a development environment.
This will first involve setting up a venv of conda environment (described after this).
The parent folder should contain a copy of [piwarsengine](https://github.com/ShefBots/piwarsengine), obtained via a `git clone https://github.com/ShefBots/piwarsengine.git` command.
Finally once everything is installed, run `python3 piwarsimulator.py`.

### venv
Make a virtualenv, activate it and install packages:
```
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```
### Conda
Or if you prefer conda:
```
conda env create -f environment.yml
```


## Architecture

This section might now be a bit out of date, but in general this code once finished can either be run in simulation or control mode. It consists of these main components:

* TheWorld – list of everything in the world (either detected or when simulation synthetic), comprised of WorldObjects that contain coordinates and type information
* RobotBrain – provides basic logic for scanning and simulation (super classes provide dedicated logic)
* Sensor – Provides sensor information (vision system, time of flight, optical flow, radio receiver) 
* Controller – Acts on the robot (wheels or launcher or ExteriorTheWorld in simulation)
* WorldRender – displays the world

Operating in control mode the robot:

* Scans all Sensors
* The RobotBrain builds TheWorld
* The RobotBrain makes decisions
* Acts through a Controller
* (optional) World rendering

Operating in simulation, virtual sensors are using an ExteriorTheWorld list consisting of the things in the world, where element 0 is the robot. Then:

Operating in simulation:

* Sensors return output based on ExteriorTheWorld 
* The RobotBrain builds TheWorld
* The RobotBrain makes decisions
* Acts through changing its location in ExteriorTheWorld via SimulationController
* World rendering

SimulationController "implements runnable" (but Python equivalent) to move the location at specified speeds. Only does one movement and/or one rotation at a time. 
