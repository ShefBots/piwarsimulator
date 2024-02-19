#!/usr/bin/env
import copy
import numpy as np


class Pathfinding:
    """
    Nagivate obstacles to a goal

    Cosntruct with a 2D numpy array of obstacles and the goal
    Pathfind with the execute function and a 2D numpy array with the starting location execute(map)
    Should return the solved map with the path to take
    Member move_record has the list of steps to the goal
    """

    # debug print output?
    DO_PRINT = 0

    # constants for different types of map locations
    EMPTY = 0
    GOAL = 1
    OBSTACLE = 2
    VISITED = 4
    ME = 8

    OK = 16
    BLOCKED = 32  # can't progress or no path found
    ARRIVED = 64

    # contants for directions (ii, jj)
    UP = (1, 0)
    DOWN = (-1, 0)
    LEFT = (0, -1)
    RIGHT = (0, 1)

    def __init__(self, obstacle_map):
        # the obstacles to avoid and the goal to get to
        self.obstacle_map = obstacle_map

        # goal location
        self.goal = Pathfinding.find(obstacle_map, Pathfinding.GOAL)

        # moves made when calculating the newmap
        self.move_record = []

    def print_map(self, map=None, basic=False):
        # top left in 0,0
        if not map is None:
            t = self.obstacle_map + map
        else:
            t = self.obstacle_map

        if basic:
            print(np.flipud(t))
        else:
            for ii in range(t.shape[0] - 1, -1, -1):
                for jj in range(0, t.shape[1]):
                    if t[ii, jj] == Pathfinding.EMPTY:
                        print("  ", end="")
                    elif t[ii, jj] == Pathfinding.GOAL:
                        print("G ", end="")
                    elif t[ii, jj] == Pathfinding.OBSTACLE:
                        print("O ", end="")
                    elif t[ii, jj] == Pathfinding.VISITED:
                        print("V ", end="")
                    elif t[ii, jj] == Pathfinding.ME:
                        print("M ", end="")
                    elif t[ii, jj] == Pathfinding.ME + Pathfinding.GOAL:
                        print("A ", end="")
                    else:
                        print("U ", end="")
                print("")

    def print(self, text):
        if self.DO_PRINT:
            print(text)

    @staticmethod
    def find(map, thing, num_of=1):
        ii, jj = np.where(map == thing)

        # only one me
        if num_of == 1:
            assert len(ii) == 1
            assert len(jj) == 1

        return ii[0], jj[0]

    @staticmethod
    def find_me(map):
        return Pathfinding.find(map, Pathfinding.ME)

    def execute(self, map):
        map = copy.deepcopy(map)
        state = Pathfinding.BLOCKED
        self.move_record = []  # reset move record
        return map, state
