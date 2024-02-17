#!/usr/bin/env python3
import time
from random import randint
from threading import Thread
from world.WorldObject import *


class MinesweeperMap(Thread):
    """
    the minesweeper map
    4x4 grid of mines, one of which is lit up red at a time
    with the robot over the lit mine for > 1 second, another mine lights up
    """

    OVERLAP_TARGET = (
        0.49  # how much of the robot should be on the tile before it's counted as on
        # a little bit elss than in MinesweeperBrain to avoid getting stuck
    )

    def __init__(self, ExteriorTheWorld):
        super(MinesweeperMap, self).__init__()
        self.robot = ExteriorTheWorld[0]
        assert isinstance(self.robot, WorldObject)
        assert self.robot.object_type == ObjectType.ROBOT

        # let the thread just be interrupted on exit and terminate
        # https://www.mandricmihai.com/2021/01/Python%20threads%20how%20to%20timeout%20and%20use%20KeyboardIntrerupt%20CTRL%20%20C.html
        self.daemon = True

        # for walls x and y are the center of the wall and they are extended by radius
        ExteriorTheWorld.append(
            WorldObject(
                object_type=ObjectType.WALL,
                x=0,
                y=-0.81,
                angle=90,
                l=1.62,
                color="gray",
            )
        )
        ExteriorTheWorld.append(
            WorldObject(
                object_type=ObjectType.WALL,
                x=0,
                y=+0.81,
                angle=90,
                l=1.62,
                color="gray",
            )
        )
        ExteriorTheWorld.append(
            WorldObject(
                object_type=ObjectType.WALL,
                x=-0.81,
                y=0,
                angle=00,
                l=1.62,
                color="gray",
            )
        )
        ExteriorTheWorld.append(
            WorldObject(
                object_type=ObjectType.WALL,
                x=+0.81,
                y=0,
                angle=00,
                l=1.62,
                color="gray",
            )
        )

        self.mines = []
        for i in range(0, 4):
            for j in range(0, 4):
                self.mines.append(
                    WorldObject(
                        object_type=ObjectType.MINE,
                        x=i * 0.4 - 0.6,
                        y=j * 0.4 - 0.6,
                        w=0.38,
                        h=0.38,
                        color=Color("gray80"),
                    )
                )
        self.active_mine = 0
        self.activate_mine()
        self.mine_touched_time = 0

        ExteriorTheWorld += self.mines

        self.start()

    def activate_mine(self):
        # set one mine active
        new_mine = randint(1, len(self.mines)) - 1
        while self.mines[new_mine].color == Color("red"):
            new_mine = randint(1, len(self.mines)) - 1
        self.mines[self.active_mine].color = Color("gray80")
        self.active_mine = new_mine
        self.mines[self.active_mine].color = Color("red")

    def run(self):
        self.running = True
        while self.running == True:
            if (
                self.robot.outline.intersects(self.mines[self.active_mine].outline)
                and self.robot.outline.intersection(
                    self.mines[self.active_mine].outline
                ).area
                / self.robot.outline.area
                > self.OVERLAP_TARGET
            ):
                if self.mine_touched_time == 0:
                    self.mine_touched_time = time.monotonic()
                on_for = time.monotonic() - self.mine_touched_time
                if on_for > 1:  # 1 second nominally?
                    self.activate_mine()
            else:
                self.mine_touched_time = 0

            time.sleep(0.25)  # assume the judge can't react any faster than 250 ms
