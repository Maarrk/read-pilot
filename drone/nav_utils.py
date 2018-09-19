"""
Functions useful for navigation
"""

from math import sin, cos, tan, pi, sqrt, atan
from dronekit_guidance import get_distance_metres
from dronekit import LocationGlobal
import utm


def raycast(alpha, beta, pitch, roll, heading, altitude):
    """
    Intersects a vector with ground, all angles in radians
    Assumes camera pointing downward, with image top (rpi cable) to drone tail
    @param alpha: angle from optical axis to right of camera
    @param beta: angle from optical axis to bottom of camera
    @param pitch: as given by Pixhawk
    @param roll: as given by Pixhawk
    @param heading: standard compass
    @param altitude: above a horizontal ground level
    @return: easting, northing in the same units as altitude
    """

    # easting, northing without heading
    base_e = tan(alpha - roll)
    base_n = tan(pitch + beta)

    return (altitude * (cos(heading) * base_e + sin(heading) * base_n),
            altitude * (cos(heading) * base_n - sin(heading) * base_e))


def sub(a, b):
    """
    Element-wise subtraction
    """
    out = list(a)
    for i in range(len(a)):
        out[i] = a[i] - b[i]

    return out


def add(a, b):
    """
    Element-wise addition
    """
    out = list(a)
    for i in range(len(a)):
        out[i] = a[i] + b[i]

    return out


def mult(a, s):
    """
    Multiply by scalar
    """
    return [x * s for x in a]


def dot(a, b):
    """
    Dot product
    sum of element-wise multiplication
    """
    out = list(a)
    for i in range(len(a)):
        out[i] = a[i] * b[i]

    return sum(out)


def normalized(a):
    """
    Vector normalized to length 1
    """
    vec_len = sqrt(dot(a, a))
    return mult(a, 1 / vec_len)


def length(a):
    """
    Length of vector
    """
    return sqrt(dot(a, a))


def clamp(val, minimum, maximum):
    """
    Clamps the value between minimum and maximum
    If minimum and maximum are in wrong order, swaps them
    """
    if minimum > maximum:
        tmp = minimum
        minimum = maximum
        maximum = tmp

    if minimum > val:
        return minimum
    elif maximum < val:
        return maximum
    else:
        return val


# def drone_distance(drone_a, drone_b):
#     """
#     Get distance in metres between two drones
#     """
#     loc_a = LocationGlobal(drone_a[dk.lat], drone_a[dk.lon])
#     loc_b = LocationGlobal(drone_b[dk.lat], drone_b[dk.lon])
#     return get_distance_metres(loc_a, loc_b)


def wpt_ndarray_to_tobik(array_locations):
    target_locations = []
    for i in range(array_locations.shape[0]):
        target_locations.append((array_locations[i][0], array_locations[i][1]))
    return target_locations


def get_distance_wgs(loc1, loc2):
    """
    Returns approximate distance in metres,
    based on dronekit example code
    @param loc1: tuple lat, lon
    @param loc2: tuple lat, lon
    @return: distance in  metres
    """
    assert len(loc1) == 2
    assert len(loc2) == 2

    dlat = loc2[0] - loc1[0]
    dlong = loc2[1] - loc1[1]
    return sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
