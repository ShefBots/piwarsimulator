#!/usr/bin/env python3
from sensors.Sensor import Sensor


class RadioControl(Sensor):
    """collects radio contorller inputs and passes them back"""

    def __init__(self):
        print("Activating radio controller sensor")
