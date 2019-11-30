#!/usr/bin/env python3
from ScanObject import *

def Scan(TheWorld):
    result = []
    robot = TheWorld[0]
    for obj in TheWorld[1:len(TheWorld)]:
        if obj.ignore == 0:
            y = ScanObject.DoScan(obj, robot)
            result.append(y)

    return result
