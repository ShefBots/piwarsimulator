#!/usr/bin/env
import numpy as np


class Pathfinding:
    # constants for map locations
    EMPTY = 0
    GOAL = 1
    OBSTACLE = 2
    VISITED = 3
    ME = 4

    OK = 10
    BLOCKED = 11
    ARRIVED = 12

    def __init__(self, obstacle_map):
        self.obstacle_map = obstacle_map
        self.map = np.zeros_like(self.obstacle_map)

        # get the i j coordinates of the goal
        (goal_ii, goal_jj) = np.where(self.obstacle_map == Pathfinding.GOAL)
        # only one goal
        assert len(goal_ii) == 1
        assert len(goal_jj) == 1
        self.goal_ii = goal_ii[0]
        self.goal_jj = goal_jj[0]

        # move me off obstacle map to the local map
        (ii, jj) = Pathfinding.find_me(self.obstacle_map)
        self.obstacle_map[ii, jj] = Pathfinding.EMPTY
        self.map[ii, jj] = Pathfinding.ME

    def print_map(self, map=None):
        if map is None:
            map = self.map
        print(self.obstacle_map + map)

    @staticmethod
    def find_me(map):
        (ii, jj) = np.where(map == Pathfinding.ME)

        # only one me
        assert len(ii) == 1
        assert len(jj) == 1

        return (ii[0], jj[0])

    def move(self, map, xdir=0, ydir=0):
        (ii, jj) = np.where(map == Pathfinding.ME)
        if (
            self.obstacle_map[ii + ydir, jj + xdir] == Pathfinding.OBSTACLE
            or map[ii - 1, jj] == Pathfinding.VISITED
        ):
            return (map, Pathfinding.BLOCKED)
        print("Success")

        map[ii, jj] = Pathfinding.VISITED
        map[ii + ydir, jj + xdir] = Pathfinding.ME
        return (map, Pathfinding.OK)

    def move_up(self, map):
        print("Trying to move up")
        return self.move(map, ydir=-1)

    def move_down(self, map):
        print("Trying to move down")
        return self.move(map, ydir=1)

    def move_left(self, map):
        print("Trying to move left")
        return self.move(map, xdir=-1)

    def move_right(self, map):
        print("Trying to move right")
        return self.move(map, xdir=1)


if __name__ == "__main__":
    print("Running in testing mode")

    obstacle_map = np.zeros((11, 11))
    obstacle_map[0, :] = Pathfinding.OBSTACLE  # top wall
    obstacle_map[-1, :] = Pathfinding.OBSTACLE  # bottom wall
    obstacle_map[:, 0] = Pathfinding.OBSTACLE  # left wall
    obstacle_map[:, -1] = Pathfinding.OBSTACLE  # right wall

    obstacle_map[4, 0:6] = Pathfinding.OBSTACLE  # to get around
    obstacle_map[1, 1] = Pathfinding.GOAL  # where to get to
    obstacle_map[9, 3] = Pathfinding.ME  # starting location

    pf = Pathfinding(obstacle_map)
    pf.print_map()
    (map, sate) = pf.move_right(pf.map)
    pf.print_map(map)
