
import numpy as np
from enum import IntEnum
import math

R_earth = 6378100

class ThrustMode(IntEnum):
    Direct_Mode = 0
    Vector_Mode = 1
    H_Mode = 2
    T_Mode = 3

# Function to calculate the thrust on each thruster from power (x, y) and rotation (z)
def vector_thrust_decomposition(x, y, z):
    """
    :param x: Power on x-axis (Forward); (-1,1) <-> (backward, forward); Float
    :param y: Power on y-axis (Left); (-1, 1) <-> (right, left); Float
    :param z: Power on z-axis (Clockwise); (-1, 1) <-> (counterclockwise, clockwise); Float
    :return: Thrust on each thruster (l, r, b); (-1,1); Float
    """

    A = np.array([[1.0, -1.0, 0.0], [-1.0, -1.0, +2.0], [2.0, 2.0, 2.0]])
    B = np.array([x, y, z])

    return np.linalg.solve(A, B)


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


def calculate_cog_and_speed(coords, delta_t):
    unit_vec = []
    speeds = []
    for i in range(len(coords)-1):
        delta_lat = math.radians(coords[i+1][0] - coords[i][0])
        delta_lon = math.radians(coords[i+1][1] - coords[i][1])
        lat_avg = math.radians((coords[i][0] + coords[i+1][0]) / 2)

        delta_y = (delta_lat) * R_earth
        delta_x = (delta_lon) * R_earth * math.cos(lat_avg)

        l = math.sqrt(delta_x**2 + delta_y**2)
        speeds.append( l/ delta_t)

        if l != 0.0:
            unit_vec.append(np.array([delta_y / l, delta_x / l]))
        else:
            unit_vec.append(np.array([0, 0]))

    avg_speed = float(sum(speeds)) / len(speeds)

    unit_vec = np.array(unit_vec).mean(axis=0)
    cog = math.atan2(unit_vec[1], unit_vec[0])

    return cog, avg_speed