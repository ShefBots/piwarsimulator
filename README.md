# piwarsimulator
Robot simulator to work on PiWars logic. This is a work in progress, revivified for 2024.

```
$ ./piwarsimulator.py -h
usage: piwarsimulator.py [-h] [--brain {EcoDisasterBrain,RobotBrain}] [--map {SimpleEcoDisasterMap}] [--mode {simulation,sensor_simulation,control}]
                         [--rendering {true,false}]

Simulator/controller for ShefBots Mark 1b for PiWars 2024

options:
  -h, --help            show this help message and exit
  --brain {EcoDisasterBrain,RobotBrain}
                        robot brain/challenge (default RobotBrain)
  --map {SimpleEcoDisasterMap}
                        map (default SimpleEcoDisasterMap)
  --mode {simulation,sensor_simulation,control}
                        operation mode (default simulation)
  --rendering {true,false}
                        render world on screen (default true)
```

## Getting started

Create a development environment, then run `python3 piwarsimulator.py`.

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

This code, once finished, can either be run in simulation or control mode. It consists of these main components:

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
