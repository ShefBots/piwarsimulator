import math
import copy

from WorldObject import *
from ObjectType import *
from brains.RobotBrain import RobotBrain


class EcoDisasterBrain(RobotBrain):
    def move(self, sensor_information):
        # find something to move towards
        goal = self.find_goal(sensor_information)

        # if we didn't find anything, were we already moving towards something?
#        if goal != None and self.goal == None:
#            print("setting new long term goal")
#            self.goal = copy.copy(goal)
        if goal == None and self.goal == None:
            print("no goals, doing nothing")
            return

        # are we within grabbing distance of the goal?
        tr = self.robot.radius + self.held_radius()
        if goal.object_type == ObjectType.ZONE:
            pass
        else:
            tr += goal.radius
        if goal.distance < tr:
            if goal.object_type == ObjectType.TARGET:
                print("grabbing goal!")
                self.holding.append(goal.parent)
            if goal.object_type == ObjectType.ZONE and len(self.holding) > 0:
                self.holding[0].ignore = True
                print("dropping off target!")
                self.holding.pop(0)
                # move backwards a little after dropping off target
                self.execute_move(-self.speed, goal.heading)
            return


        # for now operate off the short term goal only

        # rotate so we are facing the target
        heading_offset = goal.heading - self.robot.angle
        # NOTE: sometimes we rotate in the wrong direction
        # this is probably an x - 360 type issue?
        if abs(heading_offset) > self.turning_speed / 2:
            if heading_offset > 0:
                self.execute_rotate(self.turning_speed)
            else:
                self.execute_rotate(-self.turning_speed)
        else: # we're already facing so move
            self.execute_move(self.speed, goal.heading)

    def find_goal(self, sensor_information):
        """find the closest TARGET or ZONE"""
        closest = None
        closest_distance = 9e99
        for obj in sensor_information:
            # only look for a target if we're holding nothing
            if obj.object_type == ObjectType.TARGET and len(self.holding) == 0:
                if obj.distance < closest_distance:
                    closest = obj
                    closest_distance = obj.distance
            # otherwise find the zone that matches the target colour
            elif len(self.holding) > 0 and obj.object_type == ObjectType.ZONE and obj.color == self.holding[0].color:
                return obj

        return closest
