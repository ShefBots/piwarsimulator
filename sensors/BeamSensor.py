#!/usr/bin/env python3
from sensors.Sensor import Sensor

from devices import IOController


class BeamSensor(Sensor):
    """collects radio contorller inputs and passes them back"""

    def __init__(self, serial_instances):
        super().__init__()
        print("Activating gripper beam sensor...")

        self.io_controller = None

        if IOController.EXPECTED_ID in serial_instances.keys():
            self.io_controller = IOController(
                serial_instances[IOController.EXPECTED_ID]
            )
        else:
            raise Exception("Could not find IO controller hardware")

    def do_scan(self):
        beam_crossed = self.io_controller.barrel_state()
        #print(f"Beam Crossed: {beam_crossed}")
        return [], {"beam": beam_crossed}
