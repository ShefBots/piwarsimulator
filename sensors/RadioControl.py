#!/usr/bin/env python3
from sensors.Sensor import Sensor


class RadioControl(Sensor):
    """collects radio contorller inputs and passes them back"""

    def __init__(self):
        super().__init__()
        print("Activating radio controller sensor")
