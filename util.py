#!/usr/bin/env python3
import numpy as np
import shapely


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
    # if isinstance(outline, LineString):
    if outline.geom_type == "LineString" or outline.geom_type == "Point":
        return outline.xy
    else:
        return outline.exterior.xy


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
