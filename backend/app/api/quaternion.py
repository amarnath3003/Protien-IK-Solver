"""
Quaternion <-> rotation matrix conversion.

three.js (and most frontend 3D libraries) work natively with quaternions
for orientation, while the kinematics core uses 4x4 homogeneous
transforms. These helpers translate between the two at the API boundary
only, so the solver/kinematics core never needs to know about
quaternions.
"""

from __future__ import annotations

import numpy as np


def quaternion_to_matrix(q: list[float]) -> np.ndarray:
    """q = [x, y, z, w] (three.js / scipy convention) -> 3x3 rotation matrix."""
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1 - (xx + yy)],
    ])


def matrix_to_quaternion(R: np.ndarray) -> list[float]:
    """3x3 rotation matrix -> [x, y, z, w] quaternion (three.js convention).
    Standard Shepperd's method for numerical stability."""
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    trace = m00 + m11 + m22

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    return [float(x), float(y), float(z), float(w)]


def pose_to_transform(position: list[float], quaternion: list[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quaternion_to_matrix(quaternion)
    T[:3, 3] = position
    return T


def transform_to_pose(T: np.ndarray) -> tuple[list[float], list[float]]:
    return T[:3, 3].tolist(), matrix_to_quaternion(T[:3, :3])
