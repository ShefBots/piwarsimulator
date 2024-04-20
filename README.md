# piwarsimulator
Robot simulator to work on PiWars logic. This is a work in progress, revivified for 2024. The software for most of the challenges is complete. Manual robot control can be engaged by pressing spacebar and then driving with WASD/arrow for strafe and QE for rotate. G will activate the gripper when attached (although objects cannot be picked up manually).
Manual control on real hardware may be engaged using `--radio true` and then flicking the enable channel switch. The mirror switch controls the launcher/gripper.

**The robot will not start movement without releasing the parking break.** This is done by using the gripper toggle button.


```
$ ./piwarsimulator.py -h
pygame 2.5.2 (SDL 2.28.3, Python 3.11.7)
Hello from the pygame community. https://www.pygame.org/contribute.html
usage: piwarsimulator.py [-h]
                         [--brain {CheesedEcoDisasterBrain,EcoDisasterBrain,LineFollowingBrain,MazeBrain,MinesweeperBrain,RobotBrain,TOFollowingBrain}]
                         [--map {EmptyMap,EscapeRouteMap,LavaPalavaMap,MinesweeperMap,RandomEcoDisasterMap,SimpleEcoDisasterMap}]
                         [--mode {simulation,sensor_simulation,control,control_simulation,everything_sim_but_vision}]
                         [--radio {true,false}] [--rendering {true,false}] [--vision_mode {none,omnicam,simple}]
                         [--omnicam_socket_mode {local,remote}] [--attachment {none,gripper,launcher}] [--beam {true,false}]
                         [--robot_speed ROBOT_SPEED] [--turning_speed TURNING_SPEED] [--frame_rate FRAME_RATE]
                         [--tof_position {high,low}] [--leds {true,false}] [--enable_safeties {true,false}]
                         [--resolution RESOLUTION]

Simulator/controller for ShefBots robot for PiWars 2024. Press SPACE to engage manual control, WASD/Arrow keys for strafe, and QE
for rotate. G will activate the gripper if attached. The robot will not start until the parking break is released using the
gipper key.

options:
  -h, --help            show this help message and exit
  --brain {CheesedEcoDisasterBrain,EcoDisasterBrain,LineFollowingBrain,MazeBrain,MinesweeperBrain,RobotBrain,TOFollowingBrain}
                        robot brain/challenge (default RobotBrain)
  --map {EmptyMap,EscapeRouteMap,LavaPalavaMap,MinesweeperMap,RandomEcoDisasterMap,SimpleEcoDisasterMap}
                        map (default EmptyMap)
  --mode {simulation,sensor_simulation,control,control_simulation,everything_sim_but_vision}
                        operation mode (default simulation)
  --radio {true,false}  use radio receiever for control (default false)
  --rendering {true,false}
                        render world on screen (default true)
  --vision_mode {none,omnicam,simple}
                        While in simulation modes, rely on simpler vision system (default omnicam)
  --omnicam_socket_mode {local,remote}
                        When using real sensors, whether the 360 vision system should contact localhost ('local') or 192.168.22.1
                        ('remote') (defaultremote)
  --attachment {none,gripper,launcher}
                        choose an attachment (default none)
  --beam {true,false}   is the gripper equipped with the beam sensor (default false)
  --robot_speed ROBOT_SPEED
                        the top robot speed (min=0.2, max=0.9, default 0.3)
  --turning_speed TURNING_SPEED
                        the top robot speed (min=45, max=180.0, default 45)
  --frame_rate FRAME_RATE
                        number of times update (call process()) per second (min=10.0, max=120.0, default 60.0)
  --tof_position {high,low}
                        tof sensors are mounted low or high (default high)
  --leds {true,false}   light up LEDs (default true)
  --enable_safeties {true,false}
                        enforce safe robot behaivour - slowdown and don't collide (default true)
  --resolution RESOLUTION
                        window resolution (min 300, max 1200, default 900)
```

Note the `CheesedEcoDisasterBrain` applies a simpler (although less robust) algorithm to solve the challenge and is to be preferred. The more complex `EcoDisasterBrain` routine is incomplete. 

## Getting started

The first step is to create a development environment.
This will first involve setting up a venv of conda environment (described after this).
The parent folder should contain a copy of [piwarsengine](https://github.com/ShefBots/piwarsengine), obtained via a `git clone --recurse-submodules https://github.com/ShefBots/piwarsengine.git` command.
(If you already had a checkout and just need the submodules, run `git submodule update --init`.)
The folder that this copy of piwarsengine is in should be just called "piwarsengine", i.e. alongside the "piwarsimulator" folder there should be a folder called "piwarsengine" that contains the sub-folders "comms", "devices", "examples" etc.
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

### Coding style

Code style is enforced with the [Black Formatter](https://github.com/psf/black) to ensure code styling. This is available in [PIP](https://pypi.org/project/black/) or alternatively, as a [VS code plugin](https://marketplace.visualstudio.com/items?itemName=ms-python.black-formatter). There is probably a plugin for your IDE too.

### Network Setup

If you're running a Pi Zero and would like to connect to it over USB Gadget Ethernet, or if you want one of the sensors to talk to the main over USB Gadget Ethernet, it is fairly straightforward to configure.

1. On the Pi Zero enable gadget mode by editing `/boott/firmware/config.txt` and adding 
```
[all]
dtoverlay=dwc2
```
to the end of the file. Note the `[all]` section may already exist, in which case just add `dtoverlay=dwc2` to that section.

2. Edit `/boot/firmware/cmdline.txt` and add `modules-load=dwc2,g_ether g_ether.host_addr=AA:BB:CC:DD:EE:FA g_ether.dev_addr=AA:BB:CC:DD:EE:FB` to the end of the kernel arguments.

3. Create ` /etc/network/interfaces.d/usb0` with the contents
```
auto usb0
allow-hotplug usb0
iface usb0 inet static
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

You may also want persistent device naming on your Linux PC which can be acheived by adding `/etc/udev/rules.d/71-persistent-net.rules` with the content:
```
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="AA:BB:CC:DD:EE:FA", ATTR{type}=="1", KERNEL=="*", NAME="usb0"
```
afterwhich the USB gadget Ethernet device will always be usb0.

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

Operating in simulation, virtual sensors use an ExteriorTheWorld list consisting of the things in the world, where element 0 is the robot. Then:

Operating in simulation:

* Sensors return output based on ExteriorTheWorld 
* The RobotBrain builds TheWorld
* The RobotBrain makes decisions
* Acts through changing its location in ExteriorTheWorld via SimulationController
* (optional) World rendering

SimulationController "implements runnable" (but Python equivalent) to move the location at the specified velocities. It only moves the robot at the set speed, it cannot queue a list of movements to make.
