import math
import copy

import pyvisgraph as pvg
import shapely.geometry as geo

from WorldObject import *
from ObjectType import *
from brains.RobotBrain import RobotBrain


class EcoDisasterBrain(RobotBrain):
    def __init__(self, robot, speed, turning_speed):
        print(robot)

        super().__init__(self, robot=robot, speed=speed, turning_speed=turning_speed)
        self.head_for_zone = True

    def process(self, sensor_information):
        if self.movement_queue:
            robot_location = pvg.Point(self.robot.x, self.robot.y)
            self.robot.polygon = geo.Point(self.robot.x, self.robot.y).buffer(self.robot.radius)

            robot_hitbox = self.robot.polygon

            for x, y in robot_hitbox.exterior.coords:
                colliding_id = self.graph.point_in_polygon(pvg.Point(x, y))
                if colliding_id != -1:
                    print("colliding")
                    # new_target = self.graph.closest_point(robot_location, colliding_id)
                    # new_target = pvg.Point(self.robot.x, self.robot.y - 0.5)

                    self.execute_move(-self.speed / 5, self.robot.angle)
                    self.graph, self.path = self.find_path(sensor_information)
                    return
                    # print(new_target)

        else:
            self.graph, self.path = self.find_path(sensor_information, self.head_for_zone)

        for i, point in enumerate(self.path):
            print(point)
            # find the object associated with this point
            for obj in sensor_information:
                for x, y in obj.polygon.exterior.coords:
                    if round(x, 1) == point.x and round(y, 1) == point.y:
                        self.goal_wo = obj.parent
                        self.goal = obj
                        break

            # collision_check_result = self.check_for_collision(sensor_information, ignore=[self.goal])
            # if collision_check_result:
            #     return

    #         if self.goal == None:
    # #            print("no goals, doing nothing")
    #             return

            if self.goal and not self.head_for_zone:
                tr = self.robot.radius + self.held_radius()
                # if goal.object_type == ObjectType.ZONE:
                #     pass
                # else:

                tr += self.goal.radius
                if self.goal.distance < tr:
                    if self.goal.object_type == ObjectType.TARGET:
                        print("grabbing goal!")
                        self.holding.append(self.goal.parent)
                    if self.goal.object_type == ObjectType.ZONE and len(self.holding) > 0:
                        self.holding[0].ignore = True
                        print("dropping off target!")
                        self.holding.pop(0)
                        # move backwards a little after dropping off target
                        # self.execute_move(-self.speed, goal.heading)
                    return

                # rotate so we are facing the target
                heading_offset = self.goal.heading - self.robot.angle
                # this should be the fix for the it turns a complete circle sometimes bug
                if heading_offset > 180:
                    heading_offset -= 360
                elif heading_offset < -180:
                    heading_offset += 360
        #        print("%f - %f = %f" % (goal.heading, self.robot.angle, heading_offset))
                if abs(heading_offset) > 5:
                    self.movement_queue.append([2, heading_offset])

                # move towards the target
                to_travel = self.goal.distance - self.robot.radius - self.held_radius() - self.goal.radius + 0.02 # a little margin to make sure we get there
                if self.goal.object_type == ObjectType.ZONE:
                   to_travel += self.goal.radius # make sure we travel into the zone
                self.movement_queue.append([1, to_travel])
            else:
                print("travelling to ", point)
                distance = math.sqrt(math.pow(point.x - self.robot.x, 2) + math.pow(point.y - self.robot.y, 2))
                if abs(point.x - self.robot.x) < 1e-1 and point.y < self.robot.y: # special case for tan
                    heading = -180
                else:
                    heading = math.degrees(math.atan2(point.x - self.robot.x, point.y - self.robot.y))

                heading_offset = heading - self.robot.angle
                # this should be the fix for the it turns a complete circle sometimes bug
                if heading_offset > 180:
                    heading_offset -= 360
                elif heading_offset < -180:
                    heading_offset += 360
        #        print("%f - %f = %f" % (goal.heading, self.robot.angle, heading_offset))
                if abs(heading_offset) > 5:
                    self.movement_queue.append([2, heading_offset])

                # move towards the target
                to_travel = distance - self.robot.radius - self.held_radius() + 0.02 # a little margin to make sure we get there
                self.movement_queue.append([1, to_travel])

            # are we within grabbing distance of the goal?
            # this should only execute if we're facing the goal
            # TODO if the angles don't match, backup rotate, try to grab again
            # TODO add the movement for this to the movement_queue
            # i'd argue this doesn't belong in RobotTrain because the gripper is a specific atachment for EcoDisaster?




#         # find something to move towards
#         goal = self.find_goal(sensor_information)
#         if self.goal == None and goal != None: # may only want to do this if we're not holding something?
#             self.goal = goal.parent
#         # TODO verify goal and self.goal are the same in case of dodgy sensor input
#
#

    def find_path(self, sensor_information, head_for_zone=True, target=None):
        graph_objs = []
        for obj in sensor_information[6:]:
            object_coords = []
            for coords in obj.polygon.exterior.coords:
                object_coords.append(pvg.Point(coords[0], coords[1]))

            graph_objs.append(object_coords)

        graph = pvg.VisGraph()
        graph.build(graph_objs)

        if target:
            path = graph.shortest_path(pvg.Point(self.robot.x, self.robot.y), target)
            return graph, path
        else:
            if head_for_zone:
                possible_targets = possible_targets = [pvg.Point(obj.x, obj.y) for obj in sensor_information[1:7]]
            else:
                possible_targets = [pvg.Point(obj.x, obj.y) for obj in sensor_information[11:]]

            for target in possible_targets:
                path = graph.shortest_path(pvg.Point(self.robot.x, self.robot.y), target)
                # TODO: actually pick a path somehow
                break

            if head_for_zone:
                self.head_for_zone = False

        return graph, path

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
