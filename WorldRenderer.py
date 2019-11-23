#!/usr/bin/env python3
import pygame

class WorldRenderer:
    """Render the world so we can see what the robot is doing!"""
    TheWorld = None

    def __init__(self, TheWorld):
        self.TheWorld = TheWorld


    def __str__(self):
        return repr(self.TheWorld)
