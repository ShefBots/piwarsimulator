#!/usr/bin/env
import numpy as np
from heapq import heappop, heappush

if __name__ == "__main__":
    from pathfinding import Pathfinding
else:
    from algorithms.pathfinding import Pathfinding

# The real way to make this work would be to:
# a) add angle as a possible movement; and
# b) do the collision checks in the Pathfinding routine and cache the result
# The latter should let a higher resolution be used and let angle work to best result
# Note that 0, 45 and 90 degree turns are rotationally symetric to the rest of the possibilities


class AStar(Pathfinding):
    """
    Nagivate a 2D numpy array of obstacles to a goal using an A* algorithm
    Or at least I assume it's A*...

    Based on ChatGPT Jan 27 2024 query:
    "I have a 2D numpy array. Values of 0 represent empty space. A value of 1
    represents the goal location to get to. A value of 8 represents the starting
    location. A value of 2 indicates an obstacle to be avoided. Can you write a
    class that uses the Astar path finding algorithm with this data to determine
    the path from the starting location to the goal?"

    Only two mistakes in ChatGPT's code. A KeyError fixed with some () around
    the `not in cost_so_far` bit of code, and indexes being flipped in the
    `is_passable` function for `obstacle_map`. Obviously a fair bit has been
    tweaked since the initial output to make it consistent with how code here
    expects pathfinding results to be.
    """

    # debug print output?
    DO_PRINT = 0

    def __init__(self, obstacle_map, one_goal=True):
        # if len(np.where(obstacle_map == Pathfinding.GOAL)) == 1:
        #     one_goal = True
        # else:
        #     one_goal = False
        super(AStar, self).__init__(obstacle_map, one_goal)

        self.width = len(obstacle_map[0])
        self.height = len(obstacle_map)

        self.actions = [
            Pathfinding.UP,
            Pathfinding.LEFT,
            Pathfinding.RIGHT,
            Pathfinding.DOWN,  # marginally faster to try going down last?
        ]

    # def heuristic(self, a, b):
    #     return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance heuristic
    #     return (a[0] - b[0])*(a[0] - b[0]) + (a[1] - b[1])*(a[1] - b[1])  # makes things diagonal

    # def is_valid(self, x, y):
    #     return 0 <= x < self.width and 0 <= y < self.height

    # def is_passable(self, x, y):
    #     return self.is_valid(x, y) and self.obstacle_map[x][y] != Pathfinding.OBSTACLE
    #     return 0 <= x < self.width and 0 <= y < self.height and self.obstacle_map[x][y] != Pathfinding.OBSTACLE

    def reconstruct_path(self, came_from, current):
        # current = self.goal
        path = []
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def execute(self, map):
        newmap, state = super().execute(map)

        start = Pathfinding.find_me(map)  # Starting position

        # local scope is marginally faster
        goal = self.goal
        actions = self.actions
        width = self.width
        height = self.height
        obstacle_map = self.obstacle_map
        OBSTACLE = Pathfinding.OBSTACLE

        frontier = []
        heappush(frontier, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while frontier:
            _, current = heappop(frontier)

            # if current == goal:
            if current in goal:
                path = self.reconstruct_path(came_from, current)
                for x in path:
                    newmap[x[0], x[1]] = Pathfinding.VISITED
                newmap[current] = Pathfinding.ME

                # this is a wonk way to get movement direction, matching pathfinding output
                for c in range(0, len(path) - 1):
                    self.move_record.append(
                        (
                            int(path[c + 1][0] - path[c][0]),
                            int(path[c + 1][1] - path[c][1]),
                        )
                    )
                self.move_record.reverse()

                return newmap, Pathfinding.ARRIVED

            # for dx, dy in self.actions:
            for dx, dy in actions:
                next_x, next_y = current[0] + dx, current[1] + dy
                new_cost = cost_so_far[current] + 1

                # if self.is_passable(next_x, next_y) and (
                # if 0 <= next_x < self.width and 0 <= next_y < self.height and self.obstacle_map[next_x][next_y] != Pathfinding.OBSTACLE and (
                if (
                    0 <= next_x < width
                    and 0 <= next_y < height
                    and obstacle_map[next_x][next_y] != OBSTACLE
                    and (
                        (next_x, next_y) not in cost_so_far
                        or new_cost < cost_so_far[(next_x, next_y)]
                    )
                ):
                    cost_so_far[(next_x, next_y)] = new_cost
                    # priority = new_cost + self.heuristic(self.goal, (next_x, next_y))
                    # priority = new_cost + abs(goal[0] - next_x) + abs(goal[1] - next_y)
                    priority = new_cost + sum(
                        [
                            abs(goal_x - next_x) + abs(goal_y - next_y)
                            for goal_x, goal_y in goal
                        ]
                    ) / len(goal)

                    heappush(frontier, (priority, (next_x, next_y)))
                    came_from[(next_x, next_y)] = current

        return map, state  # No path found


if __name__ == "__main__":
    print("Running in testing mode")

    obstacle_map = np.zeros((11, 11))
    obstacle_map[0, :] = Pathfinding.OBSTACLE  # top wall
    obstacle_map[-1, :] = Pathfinding.OBSTACLE  # bottom wall
    obstacle_map[:, 0] = Pathfinding.OBSTACLE  # left wall
    obstacle_map[:, -1] = Pathfinding.OBSTACLE  # right wall

    obstacle_map[6, 0:6] = Pathfinding.OBSTACLE  # to get around
    obstacle_map[9, 1] = Pathfinding.GOAL  # where to get to
    obstacle_map[9, 2] = Pathfinding.GOAL
    obstacle_map[8, 1] = Pathfinding.GOAL

    map = np.zeros_like(obstacle_map)
    map[1, 3] = Pathfinding.ME  # starting location

    pf = AStar(obstacle_map, one_goal=False)

    newmap, state = pf.execute(map)

    # if not len(pf.move_record) == 0:
    if state == Pathfinding.ARRIVED:
        print("Path found:", pf.move_record)
        pf.print_map(newmap)
    else:
        print("No path found")
