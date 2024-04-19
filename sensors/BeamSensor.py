#!/usr/bin/env python3
from sensors.Sensor import Sensor
from util import get_io_controller as get_io_controller


class BeamSensor(Sensor):
    """collects radio contorller inputs and passes them back"""

    def __init__(self, serial_instances):
        super().__init__()
        print("Activating gripper beam sensor...")

        self.io_controller = get_io_controller(serial_instances)

    def do_scan(self):
        beam_crossed = self.io_controller.barrel_state()
        # print(f"Beam Crossed: {beam_crossed}")
        return [], {"beam": beam_crossed}
