#!/usr/bin/env python3
from sensors.Sensor import Sensor
from comms.sbus import (
    SBusReceiver,
    analog_decoder,
    analog_biased_decoder,
    binary_decoder,
)


class RadioControl(Sensor):
    """collects radio contorller inputs and passes them back"""

    SBUS_PORT = "/dev/serial0"
    SBUS_CHANNELS = 8
    SBUS_TIMEOUT = 0.1

    FORWARD_CHANNEL = 2
    RIGHT_CHANNEL = 3
    TURN_CHANNEL = 0
    SPEED_CHANNEL = 5
    EN_CHANNEL = 4

    LIN_SCALE = 1.0
    ANG_SCALE = 180.0

    def __init__(self, speed, turning_speed):
        super().__init__()
        print("Activating radio controller sensor...")
        self.speed = speed
        self.turning_speed = turning_speed

        # Based on @ZodiusUnfusers's piwarsengine remote_controlled.py example
        self.controller = SBusReceiver(
            self.SBUS_PORT, self.SBUS_CHANNELS, self.SBUS_TIMEOUT
        )
        # Forward/Backward
        self.controller.assign_channel_decoder(self.FORWARD_CHANNEL, analog_decoder)
        # Right/Left
        self.controller.assign_channel_decoder(self.RIGHT_CHANNEL, analog_decoder)
        # Angular
        self.controller.assign_channel_decoder(self.TURN_CHANNEL, analog_decoder)
        # Enable channel
        self.controller.assign_channel_decoder(self.EN_CHANNEL, binary_decoder)
        # Speed
        self.controller.assign_channel_decoder(
            self.SPEED_CHANNEL, analog_biased_decoder
        )

        print("Establishing Connection...")
        while not self.controller.is_connected():
            self.controller.check_receive()
        print("Connection Established.")

    def do_scan(self):
        forward_vel = 0
        sideways_vel = 0
        angular_vel = 0
        manual_control = False

        if self.controller.is_connected() and self.controller.check_receive():
            manual_control = not self.controller.read_channel(self.EN_CHANNEL)

            if manual_control:
                max_speed = self.controller.read_channel(self.SPEED_CHANNEL)

                # note all speeds here are scaled according to the max speed
                # control and then the max speed (& turning speed) of robot
                forward_vel = (
                    self.controller.read_channel(self.FORWARD_CHANNEL)
                    * self.LIN_SCALE
                    * max_speed
                    * self.speed
                )
                sideways_vel = (
                    self.controller.read_channel(self.RIGHT_CHANNEL)
                    * self.LIN_SCALE
                    * max_speed
                    * self.speed
                )
                angular_vel = (
                    self.controller.read_channel(self.TURN_CHANNEL)
                    * self.ANG_SCALE
                    * max_speed
                    * self.turning_speed
                )

        inputs = {
            "forward_vel": forward_vel,
            "sideways_vel": sideways_vel,
            "angular_vel": angular_vel,
            "manual_control": manual_control,
        }

        return [], inputs
