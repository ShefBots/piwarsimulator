#!/usr/bin/env python3
import math
from sensors.Sensor import Sensor
from world.ObjectType import *
from world.WorldObject import *
from omnicam.protocol import Mode, COMMUNICATION_PORT, REMOTE_ADDR
from websockets.sync.client import connect as sync_connect
import threading
import atexit

import time


class Vision360(Sensor):
    """class for interfacing with the 360 vision system"""

    def __init__(self, remote_connect=False):
        super().__init__()
        print(f"Activating connection to the 360 degree vision system in {'REMOTE' if remote_connect else 'LOCAL'} mode")

        # Start the websocket connection

        # Spin up a thread to monitor the websocket
        self.close_websocket_event = threading.Event()
        self.websocket_thread = threading.Thread(target=Vision360.handle_websocket_connection,
                                                 args=(None,self.close_websocket_event)
                                                )
        self.websocket_thread.start()

    def handle_websocket_connection(websocket, close_event):
        while True:
            print("Haha, I'm a forever loop!")
            time.sleep(1)
            if close_event.is_set():
                print("Closing Vision360 websocket connection...")
                break

    def do_scan(self):
        """
        return list of barrels, zones, and red mines, and the nearest white line ahead as world objects
        """

        ### COMMUNICATE WITH SYSTEM HERE ###

        scan_result = []

        # create objects if they were detected
        if True:

            # create a dummy barrel
            scanned_obj = WorldObject(
                object_type=ObjectType.BARREL,
                x=0.2,
                y=0.2,
                radius=0.025,
                color="darkgreen",
            )

            # angle relative to the robot
            scanned_obj.heading = math.degrees(
                math.atan2(scanned_obj.center[0], scanned_obj.center[1])
            )

            # add it to the list of objects found
            scan_result.append(scanned_obj)

        # return the objects found
        return scan_result, {}

    def disconnect_websocket_server(self):
        # Could've made the thread a daemon, but I'mma try and actually tidy up after myself
        self.close_websocket_event.set()
        self.websocket_thread.join()
