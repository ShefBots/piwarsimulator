#!/usr/bin/env python3
import math
from sensors.Sensor import Sensor
from world.ObjectType import *
from world.WorldObject import *
from omnicam.protocol import Mode, COMMUNICATION_PORT, REMOTE_ADDR, MODE_IDENTIFIER_INDICATOR
from websockets.sync.client import connect as sync_connect
import websockets
import threading
import json

import time

class Vision360SocketError(Exception):
    def __init__(self, additional_info, e=None, *args):
        super().__init__(args)
        self.e = e
        self.additional_info = additional_info
    def __str__(self):
        return(f"An error occured in the 360 vision socket connection (connection to {REMOTE_ADDR}).\n{self.additional_info}\nFull error:{self.e}")

class Vision360Error(Exception):
    def __init__(self, additional_info, *args):
        super().__init__(args)
        self.additional_info = additional_info
    def __str__(self):
        return(f"An error occured in the 360 vision system.\n{self.additional_info}")

BRAIN_TO_MODE_MAPPING = {
        "MinesweeperBrain": Mode.TASK_MINESWEEPER,
        "LineFollowingBrain": Mode.TASK_LAVA_PALAVA,
        "EcoDisasterBrain": Mode.TASK_ECO_DISASTER,
        "CheesedEcoDisasterBrain": Mode.TASK_ECO_DISASTER,
        "RobotBrain": Mode.TASK_MINESWEEPER,
        }
        

class Vision360(Sensor):
    """class for interfacing with the 360 vision system"""

    def __init__(self, brain_type, remote_connect=False):
        super().__init__()
        print(f"Activating connection to the 360 degree vision system in {'REMOTE' if remote_connect else 'LOCAL'} mode")

        if(brain_type not in BRAIN_TO_MODE_MAPPING.keys()):
            raise Vision360Error(f"Invalid brain type '{brain_type}'. Must be one of {BRAIN_TO_MODE_MAPPING.keys()}")
        self._brain_type = BRAIN_TO_MODE_MAPPING[brain_type]

        # Spin up a thread to contain the websocket
        self._close_websocket_event = threading.Event()
        self._visual_data = {}
        self._visual_data_lock = threading.Lock()
        self._websocket_thread = threading.Thread(target=Vision360.handle_websocket_connection,
                                                  args=(self, remote_connect)
                                                 )
        self._websocket_thread.start()

    def handle_websocket_connection(self, remote_connect):
        server_addr = REMOTE_ADDR if remote_connect else "localhost"
        try:
            print("Attempting to connect to the 360 vision system...")
            with sync_connect(f"ws://{server_addr}:{COMMUNICATION_PORT}") as websocket:
                print(f"\tConnection established to server at {server_addr}.")
                print(f"Transmitting mode \"{self._brain_type}\" command to the server...")
                websocket.send(self._brain_type.value)
                print(f"\tMode command transmitted.")
                print(f"Flushing the websocket...")
                while not self._close_websocket_event.is_set():
                    data = json.loads(websocket.recv())
                    if data[MODE_IDENTIFIER_INDICATOR] == self._brain_type.value:
                        with self._visual_data_lock:
                            self._visual_data = data
                        break
                print(f"\tWebsocket flushed.")
                while not self._close_websocket_event.is_set():
                    # Process the data
                    data = json.loads(websocket.recv())
                    # Acquire lock and update the data
                    with self._visual_data_lock:
                        self._visual_data = data
            print("360 vision system connection closed.")

        except websockets.exceptions.ConnectionClosed as e:
            raise Vision360SocketError("Connection closed by server.", e)
        except ConnectionRefusedError as e:
            raise Vision360SocketError("Connection refused by server. Check the server's status (and/or HOST LOCATION) and try again.", e)
        except TimeoutError as e:
            raise Vision360SocketError("Connection timed out. Check the server's status (and/or HOST LOCATION) and try again.", e)
        except OSError as e:
            raise Vision360SocketError(f"Connection failed due to TCP connection failure. Likely a network or system issue.\n{e.strerror}", e)
        except websockets.exceptions.InvalidHandshake:
            raise Vision360SocketError("Connection failed due to an invalid handshake. This shouldn't happen. ...is there a corrupted network path?", e)

    def do_scan(self):
        """
        return list of barrels, zones, and red mines, and the nearest white line ahead as world objects
        """

        # Aquire data retrieved from the websocket
        visual_data = {}
        with self._visual_data_lock:
            visual_data = self._visual_data

        # Actually process it

        print(f"Visual data: {visual_data}")

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
        self._close_websocket_event.set()
        self._websocket_thread.join()
