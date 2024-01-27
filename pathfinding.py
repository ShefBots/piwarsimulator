#!/usr/bin/env
import copy
import numpy as np
from itertools import compress
from time import sleep

# could keep track of visited/dead ends as obstacles?


class Pathfinding:
    """
    Nagivate a 2D numpy array of obstacles to a goal

    Create with an obstacle_map
    Call nextmove2 with a map, returns new map
    Stores internally move list

    Will need to reset move list before new calls to nextmove2

    Create with momentum weight = 0 to not do that
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
    BLOCKED = 32
    ARRIVED = 64

    # contants for directions (ii, jj)
    UP = (1, 0)
    DOWN = (-1, 0)
    LEFT = (0, -1)
    RIGHT = (0, 1)

    # perceived distance benefit to continue going in the same direction
    MOMENTUM_WEIGHT = 2

    def __init__(self, obstacle_map, momentum_weight=MOMENTUM_WEIGHT):
        self.obstacle_map = obstacle_map
        # self.map = np.zeros_like(self.obstacle_map)

        # get the i j coordinates of the goal
        (goal_ii, goal_jj) = np.where(self.obstacle_map == Pathfinding.GOAL)
        # only one goal
        assert len(goal_ii) == 1
        assert len(goal_jj) == 1
        self.goal_ii = goal_ii[0]
        self.goal_jj = goal_jj[0]

        self.momentum_weight = momentum_weight

        # moves made when calculating nextmove2
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
            for ii in np.arange(t.shape[0] - 1, -1, -1):
                for jj in np.arange(0, t.shape[1]):
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
        (ii, jj) = np.where(map == thing)

        # only one me
        if num_of == 1:
            assert len(ii) == 1
            assert len(jj) == 1

        return (ii[0], jj[0])

    @staticmethod
    def find_me(map):
        return Pathfinding.find(map, Pathfinding.ME)

    def move(self, map, xydir):
        xdir = xydir[1]
        ydir = xydir[0]
        map = copy.deepcopy(map)  # backtracking does not work with references
        if self.DO_PRINT == 1:
            if xydir == Pathfinding.UP:
                self.print("Trying to move up")
            elif xydir == Pathfinding.DOWN:
                self.print("Trying to move down")
            elif xydir == Pathfinding.LEFT:
                self.print("Trying to move left")
            elif xydir == Pathfinding.RIGHT:
                self.print("Trying to move right")
        (ii, jj) = Pathfinding.find_me(map)
        if (
            self.obstacle_map[ii + ydir, jj + xdir] == Pathfinding.OBSTACLE
            or map[ii + ydir, jj + xdir] == Pathfinding.VISITED
        ):
            return (map, Pathfinding.BLOCKED)
        self.print("Success")

        map[ii, jj] = Pathfinding.VISITED
        map[ii + ydir, jj + xdir] = Pathfinding.ME
        return (map, Pathfinding.OK)

    def nextmove2(self, map, last_move=None):
        map = copy.deepcopy(map)  # backtracking does not work with references

        if last_move is None:
            last_move = Pathfinding.UP

        state = Pathfinding.OK

        # ii+1 is up
        # ii-1 is up
        # jj-1 is left
        # jj+1 is right

        (ii, jj) = Pathfinding.find_me(map)
        self.print(f"At {ii}, {jj}")

        if ii == self.goal_ii and jj == self.goal_jj:
            self.print("AT GOAL YAY!!!")
            self.move_record.append(last_move)
            return (map, Pathfinding.ARRIVED)

        if self.DO_PRINT:
            self.print_map(map)
            sleep(0.1)

        # try a movement in each direction
        # sort possible new locations by distance
        # try those first
        directions = [
            # (ii, jj)
            Pathfinding.UP,
            Pathfinding.DOWN,
            Pathfinding.LEFT,
            Pathfinding.RIGHT,
        ]
        newmaps = [None] * len(directions)
        states = np.zeros_like(newmaps)
        dists = np.ones_like(newmaps) * 9e99
        for cc, direction in enumerate(directions):
            (newmaps[cc], states[cc]) = self.move(map, xydir=direction)
            if not states[cc] == Pathfinding.BLOCKED:
                (iis, jjs) = Pathfinding.find_me(newmaps[cc])
                dists[cc] = (self.goal_ii - iis) ** 2 + (self.goal_jj - jjs) ** 2
                if direction == last_move and dists[cc] > self.momentum_weight + 2:
                    # momentum gets problematic as we get closer, maybe use proportional?
                    dists[cc] -= self.momentum_weight  # momentum effect

        # remove BLOCKED options
        k = states != Pathfinding.BLOCKED
        directions = list(compress(directions, k))  # logical selection by k
        newmaps = list(compress(newmaps, k))  # logical selection by k
        dists = dists[k]

        # try nearest possible move first
        k = np.argsort(dists)
        directions[:] = [directions[i] for i in k]  # reorder by index order in k
        newmaps[:] = [newmaps[i] for i in k]  # reorder by index order in k
        dists = dists[k]

        for cc, _newmap in enumerate(newmaps):
            last_move = directions[cc]
            (newmap, state) = self.nextmove2(_newmap, last_move)
            if state == Pathfinding.ARRIVED:
                self.move_record.append(last_move)
                return (newmap, state)

        if len(directions) == 0 or state == Pathfinding.BLOCKED:
            self.print("Backtracking!")
            newmap = map

        return (newmap, state)


if __name__ == "__main__":
    print("Running in testing mode")

    obstacle_map = np.zeros((11, 11))
    obstacle_map[0, :] = Pathfinding.OBSTACLE  # top wall
    obstacle_map[-1, :] = Pathfinding.OBSTACLE  # bottom wall
    obstacle_map[:, 0] = Pathfinding.OBSTACLE  # left wall
    obstacle_map[:, -1] = Pathfinding.OBSTACLE  # right wall

    obstacle_map[6, 0:6] = Pathfinding.OBSTACLE  # to get around
    obstacle_map[9, 1] = Pathfinding.GOAL  # where to get to

    map = np.zeros_like(obstacle_map)
    map[1, 3] = Pathfinding.ME  # starting location

    pf = Pathfinding(obstacle_map)

    newmap, _ = pf.nextmove2(map)

    # pf.print_map(newmap, basic=True)
    pf.print_map(newmap)

    print(pf.move_record)
