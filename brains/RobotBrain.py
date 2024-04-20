#!/usr/bin/env python3
import math
import numpy as np
from time import monotonic as time
from shapely.geometry import Polygon
from time import monotonic
from brains.ExecutionState import ExecutionState
from util import rotate_by
from world.WorldObject import *
from world.ObjectType import *


class RobotBrain:
    """
    the basics of every brain
    includes some common logic
    brains (logic) for different challenges inherit this
    """

    COLLISION_TOLERANCE = 0.02  # m, how close is too close
    # precision when trying to match objects based on location
    HOLDING_TOLERANCE = 0.01  # m

    # where time of flight sensors are installed pointing
    SENSOR_HEADINGS = [0, 90, 180, 270]

    # how long between state changes to timeout for
    EXECUTION_TIMEOUT = 60

    # seconds to move when doing square up procedure
    SQUARE_UP_DURATION = 1

    def __init__(self, **kwargs):
        self.robot = kwargs.get("robot", None)
        self.attachment_controller = kwargs.get("attachment_controller", None)
        self._controller = kwargs.get("controller", None)
        self.speed = kwargs.get("speed", 0.001)  # 1 mm/s
        self.turning_speed = kwargs.get("turning_speed", 1)  # 1 degrees/s
        assert isinstance(self.robot, WorldObject)
        assert self.robot.object_type == ObjectType.ROBOT
        self.robot.brain = self

        # for letting the brain tell the program to quit
        self.running = kwargs.get("running", True)

        # the long term goal - we get this from a scan result and
        # use it to cross verify with subsequent scan results
        self.goal = None

        # anything we're transporting (exterior world objects when simulating)
        self.holding = []

        # what we're doing (roughly)
        self.state = ExecutionState.NO_CONTROL
        # keep track of previous states to time out and quit nicely
        self.last_state = ExecutionState.NO_CONTROL
        self.last_state_change_time = time()
        self.last_velocities = ([0, 0], 0)
        # don't start moving until the parking break is removed
        self.parking_break = True
        print("Starting with parking break set")

        # for squaring up
        self.square_up_heading = 0  # direction we're aligning to
        self.square_time = 0  # where we are in alignment process
        self.square_distance = 0  # distance away at start of alignment
        self.square_rotate_time = 0  # how much time to rotate for to get into alignment
        self.square_up_pass = 0  # do it twice for best results

        # for sensor output
        self.sensors = []  # any sensors
        self.sensor_last_reading = []  # the last time that sensor was read/updated
        self.TheWorld = [None]  # any objects returned by sensors
        self.sensor_measurements = {
            "manual_control": False
        }  # direct sensor measurements

        # keep track of objects in case of sensors that drop out
        self.sensor_objects = []

        # are we close to colliding with something?
        self.collision = None
        self.do_collision_detection = True

        # do collision detection and slowing down near things
        self.enable_safeties = kwargs.get("enable_safeties", True)

        # distances to the nearest wall in dirction of TOF sensors
        self.distances = [None] * len(self.SENSOR_HEADINGS)

        # anything attached to us affecting our shape
        # only used in TheWorld so only needs to be relative to pointing north unrotated
        self.attachment_outline = Polygon()

    def add_sensor(self, sensor):
        self.sensors.append(sensor)
        self.sensor_last_reading.append(time())

    def poll_sensors(self):
        """poll all sensors attached to the robot and check for collisions"""
        # add in ourself to start with
        # scanned coordinates should be relative to robot north
        self.TheWorld[0] = WorldObject(
            object_type=ObjectType.ROBOT,
            x=0,
            y=0,
            w=self.robot.width,
            h=self.robot.height,
            angle=0,
        )
        self.TheWorld[0].outline = self.TheWorld[0].outline.union(
            self.attachment_outline
        )

        # this has to be here otherwise a missing sensor not returning it = bad time
        self.sensor_measurements["manual_control"] = False

        # if objects on the curret scan is empty, reuse the previous objets with an offset applied
        # base the offset on the current velocity & rotation speed to estimate the new object locations
        # store the new estimated location in the saved reading for the next time around
        for id, s in enumerate(self.sensors):

            # read the sensor
            objects, readings = s.do_scan()

            # we had a good reading from this sensor
            if len(objects) > 0 and not objects[0] is None:

                # remove old objects from this sensor
                self.remove_by_sensor_id(id)

                # set the sensor id of the objects
                for k in range(len(objects)):
                    objects[k].sensor_id = id

                # record the objects in the world
                self.TheWorld += objects

                # update the last reading time of this sensor
                self.sensor_last_reading[id] = time()
            elif s.safe_to_guess and len(objects) == 1 and objects[0] is None:
                # otherwise we reuse the objects from this sensor and move them according to our last velocity
                # print("REUSING PREVIOUS READINGS!!! EXPECT DRIFT!!!")

                # work out how much we've moved since the last san
                now = time()
                dt = now - self.sensor_last_reading[id]
                # -ve becaues the objects move in the opposite direction of the robot?
                distance = -self._controller.vel * dt
                rotation = self._controller.theta_vel * dt

                idx_to_update = self.find_obj_by_sensor_id(id)
                for k in range(len(idx_to_update)):
                    pos = self.TheWorld[idx_to_update[k]].center
                    pos = pos + distance
                    pos = rotate_by(pos, rotation)
                    self.TheWorld[idx_to_update[k]].center = pos
                    self.TheWorld[idx_to_update[k]].angle = (
                        self.TheWorld[idx_to_update[k]].angle - rotation
                    )

                # update held item coordinates as well...
                if len(self.holding) == 1 and self.holding[0].sensor_id == id:
                    pos = self.holding[0].center
                    pos = pos + distance
                    pos = rotate_by(pos, rotation)
                    self.holding[0].center = pos
                    self.holding[0].angle = self.holding[0].angle - rotation

                # update time
                self.sensor_last_reading[id] = now
            else:
                # no reading and can't guess, lose the readings
                self.remove_by_sensor_id(id)

            for k, v in readings.items():
                self.sensor_measurements[k] = v

        # could call a sensor fusion routine here, e.g., if two objects are < 5 cm apart merge into one

    def find_obj_by_sensor_id(self, id):
        obj_ids = [obj.sensor_id for obj in self.TheWorld]
        return [k for k, v in enumerate(obj_ids) if v == id]

    def remove_by_sensor_id(self, id):
        """remove all scanned objects in TheWorld matching the given id"""
        idx_to_remove = self.find_obj_by_sensor_id(id)
        for k in sorted(idx_to_remove, reverse=True):
            del self.TheWorld[k]

    def process(self):
        """
        basic logic is to just not hit anything & respond to control input
        also to handle some shared routines (mostly squaring up)
        also to handle execution timeouts (brain not doing anything smart)
        Return false if the child brain shouldn't do anything
        """
        self.poll_sensors()
        controller_ok = self._controller.poke()
        if not controller_ok:
            print("UH OH! Lost connection with controller!!!")
        self.check_for_collision()
        self.find_distances()
        if self.enable_safeties:
            self.velocity_modify()  # check if we should slow & do so

        if self.sensor_measurements["do_quit"]:
            # this is triggered by the Escape key
            print("Quit requested")
            self.running = False
            return False

        if self.parking_break == True:
            if time() - self.last_state_change_time > 1:
                print("Parking brake is on!")
                self.last_state_change_time = time()
            # parking break is released by actioning the gripper or fire switches
            if self.sensor_measurements["gripper_toggle"] or (
                "fire" in self.sensor_measurements.keys()
                and self.sensor_measurements["fire"]
            ):
                self.parking_break = False
                self.last_state_change_time = time()
                print("Parking break released")
                # special case: parking break is a toggle inside of the keyboard class
                # we need to reach in and unset the toogle to keep the gripper from opening
                # as soon as manu control is engaged
                for s in enumerate(self.sensors):
                    if type(s[1]).__name__ == "Keyboard":
                        s[1].gripper_status = not s[1].gripper_status
                        break
            else:
                # don't do anything until the parking break is released
                return False

        # do we always want to try and stop on collisions?
        # sometimes we'll let higher level logic take over
        if (
            self.enable_safeties
            and self.do_collision_detection == True
            and not self.collision is None
        ):
            self.controller_stop()

        # print(self.sensor_measurements)
        if self.sensor_measurements["manual_control"]:
            self.last_state_change_time = time()
            if not self.state == ExecutionState.MANUAL_CONTROL:
                print(f"Storing execution state {self.state} and velocities")
                self.last_state = self.state
                self.state = ExecutionState.MANUAL_CONTROL
                self.last_velocities = (
                    self._controller.vel,
                    self._controller.theta_vel,
                )
            self.set_plane_velocity(
                [
                    self.sensor_measurements["sideways_vel"],
                    self.sensor_measurements["forward_vel"],
                ]
            )
            self.set_angular_velocity(self.sensor_measurements["angular_vel"])

            # don't want to import so name check will have to do
            if not self.attachment_controller == None and (
                type(self.attachment_controller).__name__
                == "SimulatedGripperController"
                or type(self.attachment_controller).__name__ == "GripperController"
            ):
                open = self.sensor_measurements["gripper_toggle"]
                # let the radio reciever also control the gripper
                if "fire" in self.sensor_measurements.keys():
                    # controller overrides keyboard
                    open = self.sensor_measurements["fire"]

                if (
                    open
                    and not self.attachment_controller.gripper_state
                    == self.attachment_controller.GRIPPER_OPEN
                ):  # opening
                    self.attachment_controller.open_gripper()
                    if len(self.holding) == 1:
                        # we're holding something, drop it off
                        self.holding.pop(0)
                elif (
                    not open
                    and not self.attachment_controller.gripper_state
                    == self.attachment_controller.GRIPPER_CLOSED
                ):  # closing
                    self.attachment_controller.close_gripper()
                    # TODO grab something if it's in the gripper
                    # find if something is in TheWorld within a certain distance of the gripper and grab?

            if not self.attachment_controller == None and (
                type(self.attachment_controller).__name__ == "LauncherController"
            ):
                # send commands to the launcher if attached and those commands are received
                if "brushless_speed" in self.sensor_measurements.keys():
                    self.attachment_controller.set_motor_speed(
                        self.sensor_measurements["brushless_speed"]
                    )
                if "tilt" in self.sensor_measurements.keys():
                    self.attachment_controller.set_tilt(
                        self.sensor_measurements["tilt"]
                    )
                if "fire" in self.sensor_measurements.keys():
                    self.attachment_controller.check_fire(
                        self.sensor_measurements["fire"]
                    )

        else:
            if self.state == ExecutionState.MANUAL_CONTROL:
                # resume where we were
                print(f"Restoring execution state {self.last_state}")
                self.state = self.last_state
                if not self.sensor_measurements["vel_key_pressed"]:
                    print("Restoring velocities")
                    self.set_plane_velocity(self.last_velocities[0])
                    self.set_angular_velocity(self.last_velocities[1])

        # timer on state to change to self.running = False
        if (
            not self.state == self.last_state
            and not self.state == ExecutionState.MANUAL_CONTROL
        ):
            print(f"Execution state changed")
            self.last_state = self.state
            self.last_state_change_time = time()
        elif (
            time() - self.last_state_change_time > self.EXECUTION_TIMEOUT
            and not self.state == ExecutionState.MANUAL_CONTROL
        ):
            print(f"Execution time out {self.EXECUTION_TIMEOUT}")
            self.running = False
            return False

        if self.state == ExecutionState.SQUARING_UP:
            self.square_up()
        return True

    def find_goal(self):
        """default brain has no goal"""
        return (None, 9e99)

    def check_for_collision(self):
        if not self.enable_safeties:
            return
        for obj in self.TheWorld[1:]:  # ignore the robot in 0
            if (
                self.TheWorld[0].get_distance(obj) < self.COLLISION_TOLERANCE
                and not self.is_holding(obj)
                and not (
                    # can't collide with flat objects
                    obj.object_type == ObjectType.MINE
                    or obj.object_type == ObjectType.LINE
                    or obj.object_type == ObjectType.ZONE
                )
            ):
                if self.do_collision_detection:
                    print(f"WARNING: {obj} is a bit close")
                self.collision = obj
                return

    @staticmethod
    def match_objects(obj1, obj2):
        """try and determine if obj1 and obj2 are the same object based on coordinates"""
        if (
            obj1.object_type == obj2.object_type
            and obj1.color == obj2.color
            and obj1.get_distance(obj2, relative_to="center")
            < RobotBrain.HOLDING_TOLERANCE
        ):
            # assume that if they are sufficiently close they're the same
            return True
        else:
            # they aren't the same
            return False

    def is_holding(self, obj2):
        """check if obj is something we're holding"""
        for obj1 in self.holding:
            if RobotBrain.match_objects(obj1, obj2):
                return True
        return False

    def find_closest(self, object_type, color="", exclude=[], relative_to="outline"):
        """
        find the closest object_type
        note color should be a pygame.color.Color
        exclude a list of WorldObjects
        if relative_to is a tuple or list, treat that as an offset from the center of the robot
        """
        closest = None
        closest_distance = 9e99
        for obj in self.TheWorld[1:]:  # skip the robot and check everything else
            dist = self.TheWorld[0].get_distance(obj, relative_to=relative_to)
            if (
                obj.object_type == object_type
                and (color == "" or obj.color == color)
                and not self.is_holding(obj)
                and not any(RobotBrain.match_objects(obj, obj2) for obj2 in exclude)
            ):
                if dist < closest_distance:
                    closest = obj
                    closest_distance = dist

        return (closest, closest_distance)

    def find_in_front(self, object_type, color="", exclude=[], relative_to="outline"):
        """
        find something somewhat in front of the robot
        based on find_closest
        note color should be a pygame.color.Color
        exclude a list of WorldObjects
        if relative_to is a tuple or list, treat that as an offset from the center of the robot
        """
        closest = None
        closest_distance = 9e99
        for obj in self.TheWorld[1:]:  # skip the robot and check everything else
            dist = self.TheWorld[0].get_distance(obj, relative_to=relative_to)
            t = obj.center[1]
            if isinstance(relative_to, (tuple, list, np.ndarray)):
                t = obj.center - relative_to
            else:
                raise Exception("can only find closest points in front")
            if (
                obj.object_type == object_type
                and (color == "" or obj.color == color)
                and not self.is_holding(obj)
                and not any(RobotBrain.match_objects(obj, obj2) for obj2 in exclude)
                and t[1] > 0  # this should filter for objects in front
                and abs(t[0]) < 0.25  # only straight ahead
            ):
                if dist < closest_distance:
                    # I'd like to dist > self.TheWorld[0].height/2 but then it looses tracking
                    closest = obj
                    closest_distance = dist

        return (closest, closest_distance)

    def find_distances(self):
        """find the walls reported by sensors in each heading direction"""

        # reset distances
        self.distances = [None] * len(self.SENSOR_HEADINGS)

        # for each sensor find if there's a wall in that direction
        for k, _ in enumerate(self.SENSOR_HEADINGS):
            for obj in self.TheWorld[1:]:
                if obj.object_type != ObjectType.WALL:
                    continue
                if obj.heading == self.SENSOR_HEADINGS[k]:
                    self.distances[k] = self.TheWorld[0].get_distance(obj)
                    break
            # fall back to a real number for comparisons
            if self.distances[k] is None:
                self.distances[k] = 9e99

    def distance_forward(self):
        """return the distance to ahead wall"""
        return self.distances[self.SENSOR_HEADINGS.index(0)]

    def distance_right(self):
        """return the distance to left wall"""
        return self.distances[self.SENSOR_HEADINGS.index(90)]

    def distance_back(self):
        """return the distance to left wall"""
        return self.distances[self.SENSOR_HEADINGS.index(180)]

    def distance_left(self):
        """return the distance to left wall"""
        return self.distances[self.SENSOR_HEADINGS.index(270)]

    def square_up(self):
        """
        make sure we're parallel to a wall in a direction
        move side to side checking forward and side sensors distance change
        use angle of that to rotate a specified amount
        """

        dist = self.distances[self.SENSOR_HEADINGS.index(self.square_up_heading)]

        # direction of moment for alignment
        # speed = -0.075  # want this fixed because self.speed could vary
        speed = 0.075  # could decide based on where there was more space?
        positive = 1
        if self.square_up_heading == 0:
            alignment_vector = np.array([speed, 0])
        elif self.square_up_heading == 90:
            # note, not super tested aligning to side walls
            alignment_vector = np.array([0, speed])
        elif self.square_up_heading == 270:
            # note, not super tested aligning to side walls
            positive = -1
            speed = -speed
            alignment_vector = np.array([0, speed])
        elif self.square_up_heading == 180:
            positive = -1
            speed = -speed
            alignment_vector = np.array([speed, 0])
        else:
            dist = None  # trigger a failure unknown heading

        if self.square_up_pass == 1:
            speed = -speed
            alignment_vector = -alignment_vector

        # if no reading in alignment direction cancel and return
        if dist is None:
            print("No distance reading, canceling squaring")
            self.square_up_cancel()
            return

        time_at = monotonic() - self.square_time

        if self.square_time == 0:
            # routine start
            self.square_time = monotonic()
            self.square_distance = dist
            self.square_rotate_time = 0
            self.set_plane_velocity(alignment_vector)

        elif time_at >= self.SQUARE_UP_DURATION and self.square_rotate_time == 0:
            # finished moving left, check and align
            self.controller_stop()

            base = speed * self.SQUARE_UP_DURATION
            height = self.square_distance - dist

            # needs a magic factor, function of distance and angle... iterate instead
            angle_to_wall = positive * math.degrees(math.atan(height / base))

            print(f"Square across {base} rise {height} angle {angle_to_wall}")

            if math.fabs(angle_to_wall) < 2 or math.fabs(height) < 0.003:
                # already pretty aligned
                print("Already aligned, canceling squaring")
                self.square_up_cancel()
                return

            self.square_rotate_time = math.fabs(
                angle_to_wall / (self.turning_speed / 3)
            )
            if angle_to_wall < 0:
                self.set_angular_velocity(-self.turning_speed / 3)
            else:
                self.set_angular_velocity(self.turning_speed / 3)

        elif (
            time_at >= self.SQUARE_UP_DURATION + self.square_rotate_time
            and not self._controller.theta_vel == 0
        ):
            # finished rotating to align
            self.controller_stop()
            self.set_plane_velocity(-alignment_vector)

        elif time_at >= self.SQUARE_UP_DURATION * 2 + self.square_rotate_time:
            # finished moving back
            # return control to the brain
            print(f"Done squaring up ({self.square_up_pass})")
            if self.square_up_pass == 5:
                print("Pass limit reached, canceling squaring")
                self.square_up_cancel()
                return
            else:
                self.square_up_pass += 1
                self.square_time = 0

    def square_up_cancel(self):
        self.controller_stop()
        self.square_time = 0
        self.square_rotate_time = 0
        self.square_up_pass = 0
        self.state = ExecutionState.PROGRAM_CONTROL

    def velocity_modify(self, speed_limit=None):
        # we should slow down if the gripper is open or if approaching a wall
        # (because momentum is a thing)

        if speed_limit is None:
            speed_limit = self.speed / 2

        vel = self._controller.vel

        # is the gripper not closed?
        if (
            not self.attachment_controller == None
            and (
                type(self.attachment_controller).__name__
                == "SimulatedGripperController"
                or type(self.attachment_controller).__name__ == "GripperController"
            )
            and not self.attachment_controller.gripper_state
            == self.attachment_controller.GRIPPER_CLOSED
        ):
            # CHECK BEAM SENSOR and reduce speed further if beam is hit
            if (
                "beam" in self.sensor_measurements.keys()
                and self.sensor_measurements["beam"]
            ):
                # print("Barrel speed limit hit")
                speed_limit = speed_limit / 2
            clamp = lambda v: min(max(v, -speed_limit / 2), speed_limit / 2)
            vel = [clamp(v) for v in vel]
            self._controller.set_plane_velocity(vel)

        # force the robot to slow down if the gripper is open or if it is approaching a wall
        if self.distance_forward() < 0.2 and vel[1] > speed_limit:
            vel[1] = speed_limit
        elif self.distance_back() < 0.2 and vel[1] < -speed_limit:
            vel[1] = -speed_limit

        if self.distance_right() < 0.2 and vel[0] > speed_limit:
            vel[0] = speed_limit
        elif self.distance_left() < 0.2 and vel[0] < -speed_limit:
            vel[0] = -speed_limit

        if not np.any(vel == self._controller.vel):
            print("Modifying velocity")
            self._controller.set_plane_velocity(vel)

    # set velocities here so that they can be adjusted based on sensor inputs
    def set_angular_velocity(self, theta):
        """set angular velocity in degrees per second"""
        self._controller.set_angular_velocity(theta)

    def set_plane_velocity(self, vel):
        """velocity aligned to the robot (sideways, forwards)"""
        assert len(vel) == 2, "plane velocity is always 2 components"
        self._controller.set_plane_velocity(vel)
        if self.enable_safeties:
            self.velocity_modify()

    def controller_stop(self, exiting=False):
        """stop moving"""
        self._controller.stop(exiting)
