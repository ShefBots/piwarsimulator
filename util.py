#!/usr/bin/env python3
import numpy as np

def sqr_magnitude_of(vector):
    return np.sum(np.power(vector, 2))

def rotate_by(vector, angle):
    angle_rad = np.radians(angle)
    cos_angle = np.cos(angle_rad)
    sin_angle = np.sin(angle_rad)

    T = np.array([[cos_angle, -sin_angle], [sin_angle, cos_angle]])

    return np.matmul(T, vector)