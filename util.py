#!/usr/bin/env python3
import argparse
import importlib
import numpy as np
import shapely
from glob import glob


def get_io_controller(serial_instances):
    from devices import IOController

    if IOController.EXPECTED_ID in serial_instances.keys():
        return IOController(serial_instances[IOController.EXPECTED_ID])
    else:
        raise Exception("Could not find IO controller hardware")


def sqr_magnitude_of(vector):
    return np.sum(np.power(vector, 2))


def rotate_by(vector, angle):
    angle_rad = np.radians(angle)
    cos_angle = np.cos(angle_rad)
    sin_angle = np.sin(angle_rad)

    T = np.array([[cos_angle, -sin_angle], [sin_angle, cos_angle]])

    return np.matmul(T, vector)


def outline_xy(outline):
    """return the xy coordinates of the outline"""
    if isinstance(outline, shapely.LineString) or isinstance(outline, shapely.Point):
        # return outline.xy
        # xy = outline.coords._coords
        xy = shapely.get_coordinates(outline)
    else:
        # return outline.exterior.xy
        # xy = outline.exterior.coords._coords
        xy = shapely.get_coordinates(outline.exterior)
    return xy[:, 0], xy[:, 1]


# a lot of overheads in shapely, slim it back some
def fast_translate(geom, xoff, yoff):
    # matrix = (1.0, 0.0, 0.0, 1.0, xoff, yoff)
    # return affine_transform(geom, matrix)

    # a, b, d, e = 1.0, 0.0, 0.0, 1.0
    # A = np.array([[a, b], [d, e]], dtype=float)
    off = np.array([xoff, yoff], dtype=float)

    def _affine_coords(coords):
        # return np.matmul(A, coords.T).T + off
        return coords + off

    return shapely.transform(geom, _affine_coords, include_z=False)


def find_serial_ports(pattern="/dev/ttyACM*"):
    return glob(pattern)


def create_serial_instances(port_list):
    # only need these imports here
    from serial import SerialException
    from comms.serial import SerialComms
    from devices import DEVICE_ID_LIST as SERIAL_DEVICE_ID_LIST

    serial_instances = {}
    for port in port_list:
        try:
            ser = SerialComms(port)
            print(f"Serial port {port} opened successfully.")
            try:
                identity = ser.identify()
                if identity in SERIAL_DEVICE_ID_LIST:
                    print(
                        f"Found device: {hex(identity)} ({SERIAL_DEVICE_ID_LIST[identity]})"
                    )
                else:
                    print(f"Found unknown device: {hex(identity)}")
                serial_instances[identity] = ser
            except ValueError:
                raise Exception("Not a recognised serial device")

        except SerialException as e:
            raise Exception(f"Error opening serial port {port}: {e}")
    return serial_instances


def check_in_range(value, mi, mx):
    e = argparse.ArgumentTypeError(
        f"{value} is invalid (must be between float {mi} and {mx})"
    )
    try:
        ivalue = float(value)
    except:
        raise e
    if ivalue >= mi and ivalue <= mx:
        return ivalue
    raise e


def is_true(var):
    """check if an argument is true"""
    if var.lower() == "true" or var == "1":
        return True
    return False
