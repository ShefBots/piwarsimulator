import math
import copy

from WorldObject import *
from ObjectType import *
from brains.RobotBrain import RobotBrain


class EcoDisasterBrain(RobotBrain):
    def process(self, sensor_information):

        # find something to move towards
        goal = self.find_goal(sensor_information)
        if self.goal == None and goal != None: # may only want to do this if we're not holding something?
            self.goal = goal.parent
        # TODO verify goal and self.goal are the same in case of dodgy sensor input

        # collision_check_result = self.check_for_collision(sensor_information, ignore=[self.goal])
        # if collision_check_result == True:
        #     return

        # this maybe should be before check_for_collision as it emptys the movement_queue?
        if not self.movement_queue:
            self.goal = goal.parent
        else:
            # we're currently executing movement to a goal, so don't do any new thinking
            return

        if self.goal == None:
#            print("no goals, doing nothing")
            return

        # are we within grabbing distance of the goal?
        # this should only execute if we're facing the goal
        # TODO if the angles don't match, backup rotate, try to grab again
        # TODO add the movement for this to the movement_queue
        # i'd argue this doesn't belong in RobotTrain because the gripper is a specific atachment for EcoDisaster?
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
                # self.execute_move(-self.speed, goal.heading)
            return

        # rotate so we are facing the target
        heading_offset = goal.heading - self.robot.angle
        # this should be the fix for the it turns a complete circle sometimes bug
        if heading_offset > 180:
            heading_offset -= 360
        elif heading_offset < -180:
            heading_offset += 360
#        print("%f - %f = %f" % (goal.heading, self.robot.angle, heading_offset))
        if abs(heading_offset) > 5:
            self.movement_queue.append([2, heading_offset])

        # move towards the target
        to_travel = goal.distance - self.robot.radius - self.held_radius() - goal.radius + 0.02 # a little margin to make sure we get there
        if goal.object_type == ObjectType.ZONE:
           to_travel += goal.radius # make sure we travel into the zone
        self.movement_queue.append([1, to_travel])

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

    def move_to_zone(self, sensor_information, step):
        # print("running move to zone")
        if step == 0:
            # self.movement_queue.append([2, 90])
            self.execute_rotate(90)
            print("turning 90")

        if step == 1:
            for obj in sensor_information:
                # print(obj.object_type, obj.heading)
                if obj.object_type == ObjectType.WALL and obj.heading == 0:
                    # print(obj.distance)
                    wall_distance = obj.distance
                    break
            # self.movement_queue.append([1, wall_distance - self.robot.radius])
            self.execute_move(wall_distance - self.robot.x, heading=90)
            print("moving ", wall_distance - self.robot.radius)

        if step == 2:
            # self.movement_queue.append([2, -90])
            self.execute_rotate(-90)
            print("turning -90")
        #
        if step == 3:
            for obj in sensor_information:
                print(obj.object_type, obj.heading)
                if obj.object_type == ObjectType.WALL and obj.heading < 20 and obj.heading < -20:
                    print(obj.distance)
                    wall_distance = obj.distance
                    # break
            print("moving ", wall_distance - self.robot.y - self.robot.radius)
            # self.movement_queue.append([1, wall_distance - self.robot.radius])
            self.execute_move(wall_distance - self.robot.y - self.robot.radius, heading=0)

            print("robot pos ", self.robot.x, self.robot.y)
        #
        if step == 4:
            print("turning -90")
            self.execute_rotate(-90)
        #     # self.movement_queue.append([2, -90])
        #
        if step == 5:
            for obj in sensor_information:
                print(obj.object_type, obj.heading, obj.x, obj.y)
                if obj.object_type == ObjectType.ZONE:
                    print(obj.distance)
                    zone_distance = obj.distance
                    break
            # self.movement_queue.append([1, zone_distance])
            self.execute_move(zone_distance - self.robot.x - self.robot.radius, heading=90)
            print("moving ", zone_distance - self.robot.x - self.robot.radius)

        if step == 6:
            print("turning -90")
            self.execute_rotate(-90)
