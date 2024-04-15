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

    EN_CHANNEL = 7  # Right most switch: enable manual control

    BRUSHLESS_CHANNEL = 5  # Left Pot: Brushless motor speed
    TILT_CHANNEL = 6  # Right Pot: Tilt the turret up and down
    FIRE_CHANNEL = 4  # Left most switch: fire/advance

    LIN_SCALE = 1.0
    ANG_SCALE = 180.0

    def __init__(self, speed, turning_speed):
        super().__init__()
        print("Activating radio remote controller sensor...")
        self.speed = speed
        self.turning_speed = turning_speed

        # Based on @ZodiusUnfusers's piwarsengine remote_controlled.py example
        self.remote = SBusReceiver(
            self.SBUS_PORT, self.SBUS_CHANNELS, self.SBUS_TIMEOUT
        )
        # Forward/Backward
        self.remote.assign_channel_decoder(self.FORWARD_CHANNEL, analog_decoder)
        # Right/Left
        self.remote.assign_channel_decoder(self.RIGHT_CHANNEL, analog_decoder)
        # Angular
        self.remote.assign_channel_decoder(self.TURN_CHANNEL, analog_decoder)

        # Enable manual control
        self.remote.assign_channel_decoder(self.EN_CHANNEL, binary_decoder)

        # Launcher brushless
        self.remote.assign_channel_decoder(
            self.BRUSHLESS_CHANNEL, analog_biased_decoder
        )
        # Launcher tilt
        self.remote.assign_channel_decoder(self.TILT_CHANNEL, analog_biased_decoder)
        # Launcher fire/advance
        self.remote.assign_channel_decoder(self.FIRE_CHANNEL, binary_decoder)

        print("Establishing Connection...")
        while not self.remote.is_connected():
            self.remote.check_receive()
        print("Connection Established.")

    def do_scan(self):
        forward_vel = 0
        sideways_vel = 0
        angular_vel = 0
        manual_control = False

        brushless_speed = None
        tilt = None
        fire = None

        if self.remote.is_connected() and self.remote.check_receive():
            manual_control = self.remote.read_channel(self.EN_CHANNEL)

            if manual_control:
                # read channel returns -1 to 1, scale by robot speed for velocity
                forward_vel = (
                    self.remote.read_channel(self.FORWARD_CHANNEL) * self.speed
                )
                sideways_vel = self.remote.read_channel(self.RIGHT_CHANNEL) * self.speed
                angular_vel = (
                    self.remote.read_channel(self.TURN_CHANNEL) * self.turning_speed
                )

                # for launcher control
                # brushless and tilt will be 0 to 1
                brushless_speed = self.remote.read_channel(self.BRUSHLESS_CHANNEL)
                tilt = self.remote.read_channel(self.TILT_CHANNEL)
                fire = self.remote.read_channel(self.FIRE_CHANNEL)  # 1 for fire

        inputs = {
            "forward_vel": forward_vel,
            "sideways_vel": sideways_vel,
            "angular_vel": angular_vel,
            "manual_control": manual_control,
        }

        if not brushless_speed is None:
            inputs["brushless_speed"] = brushless_speed

        if not tilt is None:
            inputs["tilt"] = tilt

        if not fire is None:
            inputs["fire"] = fire

        return [], inputs
