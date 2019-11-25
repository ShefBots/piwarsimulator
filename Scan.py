#!/usr/bin/env python3
from ScanObject import *

def Scan(TheWorld):
    result = []
    robot = TheWorld[0]
    for x in TheWorld[1:len(TheWorld)]:
        y = ScanObject.DoScan(x, robot)
        result.append(y)

    return result
