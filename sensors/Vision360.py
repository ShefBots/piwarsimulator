#!/usr/bin/env python3
import math
from sensors.Sensor import Sensor
from world.ObjectType import *
from world.WorldObject import *
from omnicam.protocol import Mode, COMMUNICATION_PORT, REMOTE_ADDR
from websockets.sync.client import connect as sync_connect
import websockets
import threading

import time

class Vision360Error(Exception):
    def __init__(self, additional_info, e, *args):
        super().__init__(args)
        self.e = e
        self.additional_info = additional_info
    def __str__(self):
        return(f"An error occured in the 360 vision socket connection (connection to {REMOTE_ADDR}).\n{self.additional_info}\nFull error:{self.e}")
        

class Vision360(Sensor):
    """class for interfacing with the 360 vision system"""

    def __init__(self, remote_connect=False):
        super().__init__()
        print(f"Activating connection to the 360 degree vision system in {'REMOTE' if remote_connect else 'LOCAL'} mode")

        # Spin up a thread to contain the websocket
        self.close_websocket_event = threading.Event()
        self.websocket_thread = threading.Thread(target=Vision360.handle_websocket_connection,
                                                 args=(remote_connect, self.close_websocket_event)
                                                )
        self.websocket_thread.start()

    def handle_websocket_connection(remote_connect, close_event):
        server_addr = REMOTE_ADDR if remote_connect else "localhost"
        try:
            print("Attempting to connect to the 360 vision system...")
            with sync_connect(f"ws://{server_addr}:{COMMUNICATION_PORT}") as websocket:
                print(f"\tConnection established to server at {server_addr}.")
                while not close_event.is_set():
                    data = websocket.recv()
                    print(f"Recieved:\n{data}")
            print("360 vision system connection closed.")

        except websockets.exceptions.ConnectionClosed as e:
            raise Vision360Error("Connection closed by server.", e)
        except ConnectionRefusedError as e:
            raise Vision360Error("Connection refused by server. Check the server's status (and/or HOST LOCATION) and try again.", e)
        except TimeoutError as e:
            raise Vision360Error("Connection timed out. Check the server's status (and/or HOST LOCATION) and try again.", e)
        except OSError as e:
            raise Vision360Error(f"Connection failed due to TCP connection failure. Likely a network or system issue.\n{e.strerror}", e)
        except websockets.exceptions.InvalidHandshake:
            raise Vision360Error("Connection failed due to an invalid handshake. This shouldn't happen. ...is there a corrupted network path?", e)

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
