
import numpy as np

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