#!/usr/bin/env python3
from sensors.ScanObject import *

def Scan(TheWorld):
    result = []
    robot = TheWorld[0]
    for obj in TheWorld[1:len(TheWorld)]:
        if not obj.ignore:
            y = ScanObject.do_scan(obj, robot)
            # TODO add some uncertainty to objects that are further away
            result.append(y)

    return result
