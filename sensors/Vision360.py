#!/usr/bin/env python3
import asyncio
import websockets
import json
import math
from sensors.Sensor import Sensor
from world.ObjectType import *
from world.WorldObject import *

import sys
sys.path.insert(1, "omnicam-zero") # Horrible hacky bit to import the omnicam-zero library because it has a dash in it.
from protocol import Mode, COMMUNICATION_PORT, REMOTE_ADDR


class Vision360(Sensor):
    """class for interfacing with the 360 vision system"""

    async def connect(self, remote):
        self.websocket = await websockets.connect(f"ws://{REMOTE_ADDR if remote else 'localhost'}:{COMMUNICATION_PORT}")
        try:
            while True:
                data= await self.websocket.recv()
                print(f"Vision360 data recieved: {data}")
                
                # TODO: This is probably janky multithreading
                # We've got new data, so wait until the data lock is available and then update the data
                await self.data_lock.wait()
                self.data = json.loads(data)
        
        except websockets.ConnectionClosed:
            print("Connection closed")
        except Exception as e:
            print(f"WEBSOCKET CONNECTION ERROR: {e}")

    def __init__(self, remote_connect=False):
        super().__init__()
        print("Activating connection to the 360 degree vision system")

        self.data_lock = asyncio.Event()
        self.data = {}
        asyncio.create_task(self.connect(remote_connect))

    def do_scan(self):
        """
        return list of barrels, zones, and red mines, and the nearest white line ahead as world objects
        """

        # We're reading the data so it's not available
        self.data_lock.clear()

        print(self.data)

        # Okay we're done with the data now
        self.data_lock.set()

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
