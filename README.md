# piwarsimulator
Robot simulator to work on PiWars logic. This is a work in progress, revivified for 2024. The software for most of the challenges is complete, with Eco-Disaster being the most incomplete. Manual robot control can be engaged by pressing spacebar and then driving with WASD/arrow for strafe and QE for rotate.
Manual control on real hardware may be engaged using `--radio true` and then flicking the enable channel switch.

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
  --map {EscapeRouteMap,LavaPalavaMap,MinesweeperMap,RandomEcoDisasterMap,SimpleEcoDisasterMap}
                        map (default EscapeRouteMap)
  --mode {simulation,sensor_simulation,control}
                        operation mode (default simulation)
  --radio {true,false}  use radio receiever for control (default false)
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

### Network Setup

If you're running a Pi Zero and would like to connect to it over USB Gadget Ethernet, or if you want one of the sensors to talk to the main over USB Gadget Ethernet, it is fairly straightforward to configure.

1. On the Pi Zero enable gadget mode by editing `/boott/firmware/config.txt` and adding 
```
[all]
dtoverlay=dwc2
```
to the end of the file. Note the `[all]` section may already exist, in which case just add `dtoverlay=dwc2` to that section.

2. Edit `/boot/firmware/cmdline.txt` and add `modules-load=dwc2,g_ether` to the end of the kernel arguments.

3. Create ` /etc/network/interfaces.d/usb0` with the contents
```
auto usb0
allow-hotplug usb0
face usb0 inet static
address 192.168.XX.YY
netmask 255.255.255.0
```
where XX and YY are the last two octets of the IP address you want the zero to have.

4. Create `/etc/NetworkManager/conf.d/unmanage.conf` with the contents
```
[keyfile]
unmanaged-devices=interface-name:usb0
```
which will let the ifup script we just wrote work.

5. Reboot and you should be good to go. Note the keyboard may not work any more until you reverse step 1.

For the host pi, only steps 3 and 4 are needed. Be sure to use a different IP address in the same subnet (only change the last octet).

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
