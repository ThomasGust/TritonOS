from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np


@dataclass
class Quaternion:
    """Quaternion stored as (w, x, y, z).

    Convention used throughout this project:
      - q represents the rotation that maps BODY vectors into WORLD: v_w = q ⊗ v_b ⊗ q*.
      - WORLD frame is arbitrary, but yaw is interpreted as rotation about +Z (right-handed).

    If your axes/signs are different, use the mount matrix in the config.
    """

    w: float
    x: float
    y: float
    z: float

    def as_np(self) -> np.ndarray:
        return np.array([self.w, self.x, self.y, self.z], dtype=float)

    @staticmethod
    def identity() -> "Quaternion":
        return Quaternion(1.0, 0.0, 0.0, 0.0)


@staticmethod
def from_rotation_matrix(R: np.ndarray) -> "Quaternion":
    """Create a quaternion from a 3x3 rotation matrix.

    R must map BODY -> WORLD (same convention as this project).
    """
    R = np.asarray(R, dtype=float).reshape(3, 3)
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0  # S=4*qw
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
    return Quaternion(float(qw), float(qx), float(qy), float(qz)).normalized()
    def normalized(self) -> "Quaternion":
        n = math.sqrt(self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z)
        if n <= 0.0:
            return Quaternion.identity()
        return Quaternion(self.w/n, self.x/n, self.y/n, self.z/n)

    def conj(self) -> "Quaternion":
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def __mul__(self, other: "Quaternion") -> "Quaternion":
        # Hamilton product
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z
        return Quaternion(
            w=w1*w2 - x1*x2 - y1*y2 - z1*z2,
            x=w1*x2 + x1*w2 + y1*z2 - z1*y2,
            y=w1*y2 - x1*z2 + y1*w2 + z1*x2,
            z=w1*z2 + x1*y2 - y1*x2 + z1*w2,
        )

    def rotate(self, v_b: Iterable[float]) -> np.ndarray:
        """Rotate BODY vector to WORLD."""
        vx, vy, vz = [float(x) for x in v_b]
        vq = Quaternion(0.0, vx, vy, vz)
        rq = self * vq * self.conj()
        return np.array([rq.x, rq.y, rq.z], dtype=float)

    def inv_rotate(self, v_w: Iterable[float]) -> np.ndarray:
        """Rotate WORLD vector to BODY."""
        vx, vy, vz = [float(x) for x in v_w]
        vq = Quaternion(0.0, vx, vy, vz)
        rq = self.conj() * vq * self
        return np.array([rq.x, rq.y, rq.z], dtype=float)

    def integrate_gyro(self, omega_rad_s: Iterable[float], dt: float) -> "Quaternion":
        """Integrate body gyro (rad/s) forward by dt."""
        gx, gy, gz = [float(x) for x in omega_rad_s]
        # q_dot = 0.5 * q ⊗ [0, gx, gy, gz]
        q = self
        qdot = Quaternion(
            0.5 * (-q.x*gx - q.y*gy - q.z*gz),
            0.5 * ( q.w*gx + q.y*gz - q.z*gy),
            0.5 * ( q.w*gy - q.x*gz + q.z*gx),
            0.5 * ( q.w*gz + q.x*gy - q.y*gx),
        )
        return Quaternion(
            q.w + qdot.w*dt,
            q.x + qdot.x*dt,
            q.y + qdot.y*dt,
            q.z + qdot.z*dt,
        ).normalized()


def quat_to_euler_deg(q: Quaternion) -> Tuple[float, float, float]:
    """Return (roll, pitch, yaw) in degrees.

    Assumes aerospace sequence (X=roll, Y=pitch, Z=yaw), right-handed.
    """
    q = q.normalized()
    w, x, y, z = q.w, q.x, q.y, q.z

    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w*y - z*x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi/2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def wrap_degrees(angle: float) -> float:
    """Wrap to [-180, 180)."""
    a = (angle + 180.0) % 360.0 - 180.0
    # handle -180 edge to keep stable printing
    return 180.0 if a == -180.0 else a