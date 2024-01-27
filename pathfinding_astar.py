#!/usr/bin/env
import copy
import numpy as np
from heapq import heappop, heappush
from pathfinding import Pathfinding


class AStar(Pathfinding):
    """
    Nagivate a 2D numpy array of obstacles to a goal using an A* algorithm
    Or at least I assume it's A*...

    Based on ChatGPT Jan 27 2024 query:
    "I have a 2D numpy array. Values of 0 represent empty space. A value of 1
    represents the goal location to get to. A value of 8 represents the starting
    location. A value of 2 indicates an obstacle to be avoided. Can you write a
    class that uses the Astar path finding algorithm with this data to determine
    the path from the starting location to the goal?
    """

    # debug print output?
    DO_PRINT = 0

    def __init__(self, obstacle_map):
        self.obstacle_map = obstacle_map
        self.width = len(obstacle_map[0])
        self.height = len(obstacle_map)

        self.goal = Pathfinding.find(obstacle_map, Pathfinding.GOAL)  # Goal position

        self.actions = [
            Pathfinding.UP,
            Pathfinding.DOWN,
            Pathfinding.LEFT,
            Pathfinding.RIGHT,
        ]

        # moves made when calculating the newmap
        self.move_record = []

    def heuristic(self, a, b):
        # Manhattan distance heuristic
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def is_valid(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def is_passable(self, x, y):
        return self.is_valid(x, y) and self.obstacle_map[x][y] != AStar.OBSTACLE

    def reconstruct_path(self, came_from):
        current = self.goal
        path = []
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def astar(self, map):
        newmap = copy.deepcopy(map)

        start = Pathfinding.find(map, Pathfinding.ME)  # Starting position

        frontier = []
        heappush(frontier, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while frontier:
            _, current = heappop(frontier)

            if current == self.goal:
                path = self.reconstruct_path(came_from)
                for x in path:
                    newmap[x[0], x[1]] = AStar.VISITED
                # self.move_record = path

                # this is a wonk way to get movement direction, matching pathfinding output
                for c in range(0, len(path) - 1):
                    self.move_record.append(
                        (
                            int(path[c + 1][0] - path[c][0]),
                            int(path[c + 1][1] - path[c][1]),
                        )
                    )
                self.move_record.reverse()

                return newmap

            for dx, dy in self.actions:
                next_x, next_y = current[0] + dx, current[1] + dy
                new_cost = cost_so_far[current] + 1

                if self.is_passable(next_x, next_y) and (
                    (next_x, next_y) not in cost_so_far
                    or new_cost < cost_so_far[(next_x, next_y)]
                ):
                    cost_so_far[(next_x, next_y)] = new_cost
                    priority = new_cost + self.heuristic(self.goal, (next_x, next_y))
                    heappush(frontier, (priority, (next_x, next_y)))
                    came_from[(next_x, next_y)] = current

        return map  # No path found


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

    pf = AStar(obstacle_map)

    newmap = pf.astar(map)

    if not len(pf.move_record) == 0:
        print("Path found:", pf.move_record)
        pf.print_map(newmap)
    else:
        print("No path found")