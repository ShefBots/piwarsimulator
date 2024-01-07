#!/usr/bin/env python3


class Sensor:
    """sensors see/detect things"""

    def __init__(self):
        self.outline = None  # area covered by the sensor relative to robot
        self.fov = None  # area covered by the sensor in global ExteriorTheWorld coordinate system

    def do_scan(self):
        """
        return the results of a scan/poll, by default nothing
        returns a list of WorldObjects and a dictionary of other readings
        """

        return [], {}
