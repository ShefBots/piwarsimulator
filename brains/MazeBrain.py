#!/usr/bin/env python3
import math
import copy

from world.WorldObject import *
from world.ObjectType import *
from brains.RobotBrain import RobotBrain


class MazeBrain(RobotBrain):
    """logic for the escape route following challenge"""

    SENSOR_HEADINGS = [-90, 0, 90]
    # how close to get to walls before moving to next program stage
    WALL_STOP_DISTANCE = 0.15

    # things the robot could be doing
    ALIGNMENT = 0
    MOVE_LEFT = 1
    MOVE_FORWARD = 2
    MOVE_RIGHT = 3

    CURRENT_MODE = 0

    def process(self):
        """do the basic brain stuff then do specific ecodisaster things"""

        # check sensors and stop if collision is imminent
        super().process()
        # don't do anything if the manual override is triggered
        if self.sensor_measurements["manual_control"]:
            return

        if self.CURRENT_MODE == self.ALIGNMENT:
            # TODO wall alignment routine
            # move side to side checking forward sensor distance change
            # use angle of that to rotate a specified amount
            self.CURRENT_MODE = self.MOVE_LEFT
            self.controller.set_plane_velocity([-self.speed, 0])

        # get the distances to walls in each heading
        distances = self.find_distances()

        if self.CURRENT_MODE == self.MOVE_LEFT:
            if not distances[0] == None and distances[0] < self.WALL_STOP_DISTANCE:
                self.controller.set_plane_velocity([0, self.speed])
                self.CURRENT_MODE = self.MOVE_FORWARD

        elif self.CURRENT_MODE == self.MOVE_FORWARD:
            if not distances[1] == None and distances[1] < self.WALL_STOP_DISTANCE:
                if distances[2] == None or distances[2] > self.WALL_STOP_DISTANCE:
                    # there's space on the right, go that way
                    self.controller.set_plane_velocity([self.speed, 0])
                    self.CURRENT_MODE = self.MOVE_RIGHT
                elif distances[0] == None or distances[0] > self.WALL_STOP_DISTANCE:
                    # there's space on the left, go that way
                    self.controller.set_plane_velocity([-self.speed, 0])
                    self.CURRENT_MODE = self.MOVE_LEFT

            # TODO no reading from any sensors? STOP

        elif self.CURRENT_MODE == self.MOVE_RIGHT:
            if not distances[2] == None and distances[2] < self.WALL_STOP_DISTANCE:
                self.controller.set_plane_velocity([0, self.speed])
                self.CURRENT_MODE = self.MOVE_FORWARD

        # which ever of the three time of flight sensors has the furthest reading, head towards that
        # or move so that ideally each time of flight reports 5 cm
        # do forward sensor, then left sensor, then right sensor

    def find_distances(self):
        """find the walls in each direction"""

        distances = [None] * len(self.SENSOR_HEADINGS)

        for k, _ in enumerate(distances):
            for obj in self.TheWorld[1:]:
                if obj.object_type != ObjectType.WALL:
                    continue
                if obj.heading == self.SENSOR_HEADINGS[k]:
                    distances[k] = self.TheWorld[0].get_distance(obj)
                    break

        return distances
