# ChatCPT Jan 27 2024
# I have a 2D numpy array. Values of 0 represent empty space. A value of 1 represents the goal location to get to. A value of 4 represents the starting location. A value of 2 indicates an obstacle to be avoided. Can you write a class that uses the Astar path finding algorithm with this data to determine the path from the starting location to the goal?

import numpy as np
from heapq import heappop, heappush


# TODO inherit from pathfinding and override 

class AStar:
    EMPTY = 0
    GOAL = 1
    OBSTACLE = 2
    VISITED = 4
    ME = 8

    def __init__(self, grid):
        self.grid = grid
        self.width = len(grid[0])
        self.height = len(grid)
        self.actions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Right, Left, Down, Up

    def heuristic(self, a, b):
        # Manhattan distance heuristic
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def is_valid(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def is_passable(self, x, y):
        return self.is_valid(x, y) and self.grid[x][y] != AStar.OBSTACLE

    def reconstruct_path(self, came_from, goal):
        current = goal
        path = []
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def astar(self, start, goal):
        frontier = []
        heappush(frontier, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while frontier:
            _, current = heappop(frontier)

            if current == goal:
                return self.reconstruct_path(came_from, goal)

            for dx, dy in self.actions:
                next_x, next_y = current[0] + dx, current[1] + dy
                new_cost = cost_so_far[current] + 1

                # if self.is_passable(next_x, next_y) and (next_x, next_y) not in cost_so_far or new_cost < cost_so_far[(next_x, next_y)]:
                if self.is_passable(next_x, next_y) and ((next_x, next_y) not in cost_so_far or new_cost < cost_so_far[(next_x, next_y)]):
                # if self.is_passable(next_x, next_y):
                    # if (next_x, next_y) not in cost_so_far or new_cost < cost_so_far[(next_x, next_y)]:
                    cost_so_far[(next_x, next_y)] = new_cost
                    priority = new_cost + self.heuristic(goal, (next_x, next_y))
                    heappush(frontier, (priority, (next_x, next_y)))
                    came_from[(next_x, next_y)] = current

        return None  # No path found


def print_map(t):
    for ii in np.arange(t.shape[0] - 1, -1, -1):
        for jj in np.arange(0, t.shape[1]):
            if t[ii, jj] == AStar.EMPTY:
                print("  ", end="")
            elif t[ii, jj] == AStar.GOAL:
                print("G ", end="")
            elif t[ii, jj] == AStar.OBSTACLE:
                print("O ", end="")
            elif t[ii, jj] == AStar.VISITED:
                print("V ", end="")
            elif t[ii, jj] == AStar.ME:
                print("M ", end="")
            elif t[ii, jj] == AStar.ME + AStar.GOAL:
                print("A ", end="")
            else:
                print("U ", end="")
        print("")


# Example usage:
if __name__ == "__main__":
    # grid = np.array([
    #     [0, 2, 0, 0, 0],
    #     [8, 2, 0, 2, 0],
    #     [0, 2, 0, 2, 0],
    #     [0, 2, 0, 2, 1],
    #     [0, 0, 0, 0, 0]
    # ])

    grid = np.zeros((11, 11))
    grid[0, :] = AStar.OBSTACLE  # top wall
    grid[-1, :] = AStar.OBSTACLE  # bottom wall
    grid[:, 0] = AStar.OBSTACLE  # left wall
    grid[:, -1] = AStar.OBSTACLE  # right wall

    grid[6, 0:6] = AStar.OBSTACLE  # to get around
    grid[9, 1] = AStar.GOAL  # where to get to

    grid[1, 3] = AStar.ME  # starting location

    astar = AStar(grid)
    # start = (1, 0)  # Starting position
    # goal = (4, 3)   # Goal position

    (ii, jj) = np.where(grid == AStar.ME)
    start = (ii[0], jj[0])
    (ii, jj) = np.where(grid == AStar.GOAL)
    goal = (ii[0], jj[0])
    print(f"start: {start}")
    print(f"goal: {goal}")

    path = astar.astar(start, goal)
    if path:
        print("Path found:", path)
        for x in path:
            # print(f"{x[0]} , {x[1]}")
            # if grid[x[1], x[0]] == 0:
            grid[x[0], x[1]] = AStar.VISITED
        print_map(grid)
        # print(grid)
    else:
        print("No path found")
