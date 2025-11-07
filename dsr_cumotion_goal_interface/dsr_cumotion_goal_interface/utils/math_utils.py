# dsr_motion_command/utils/math_utils.py
import math
import numpy as np

def euler_zyz_to_quaternion(z1: float, y: float, z2: float):
    """
    Convert ZYZ Euler angles (radians) to quaternion (x, y, z, w).
    No external dependencies (pure math version).

    Args:
        z1: first rotation around Z axis
        y:  second rotation around Y axis
        z2: third rotation around Z axis

    Returns:
        (qx, qy, qz, qw): quaternion components
    """

    cz1 = math.cos(z1 / 2)
    sz1 = math.sin(z1 / 2)
    cy  = math.cos(y  / 2)
    sy  = math.sin(y  / 2)
    cz2 = math.cos(z2 / 2)
    sz2 = math.sin(z2 / 2)

    # Formula for intrinsic Z–Y–Z rotation → quaternion
    qw = cz1 * cy * cz2 - sz1 * cy * sz2
    qx = cz1 * sy * sz2 + sz1 * sy * cz2
    qy = sz1 * sy * sz2 - cz1 * sy * cz2
    qz = cz1 * cy * sz2 + sz1 * cy * cz2

    return qx, qy, qz, qw



def euler_to_quaternion(rx: float, ry: float, rz: float):
    """Convert Euler angles (radians) to quaternion (x, y, z, w)."""
    cr = math.cos(rx / 2)
    sr = math.sin(rx / 2)
    cp = math.cos(ry / 2)
    sp = math.sin(ry / 2)
    cy = math.cos(rz / 2)
    sy = math.sin(rz / 2)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def _quaternion_to_matrix(qx, qy, qz, qw):
    """Convert quaternion (x, y, z, w) to 3x3 rotation matrix."""
    R = np.zeros((3, 3))
    R[0, 0] = 1 - 2 * (qy**2 + qz**2)
    R[0, 1] = 2 * (qx*qy - qz*qw)
    R[0, 2] = 2 * (qx*qz + qy*qw)
    R[1, 0] = 2 * (qx*qy + qz*qw)
    R[1, 1] = 1 - 2 * (qx**2 + qz**2)
    R[1, 2] = 2 * (qy*qz - qx*qw)
    R[2, 0] = 2 * (qx*qz - qy*qw)
    R[2, 1] = 2 * (qy*qz + qx*qw)
    R[2, 2] = 1 - 2 * (qx**2 + qy**2)
    return R


def _matrix_to_quaternion(R):
    """Convert 3x3 rotation matrix to quaternion (x, y, z, w)."""
    q = np.empty(4)
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        q[3] = 0.25 * s
        q[0] = (R[2, 1] - R[1, 2]) / s
        q[1] = (R[0, 2] - R[2, 0]) / s
        q[2] = (R[1, 0] - R[0, 1]) / s
    else:
        i = np.argmax([R[0, 0], R[1, 1], R[2, 2]])
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s
    return q

