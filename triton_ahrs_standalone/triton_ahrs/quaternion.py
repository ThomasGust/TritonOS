from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np


@dataclass
class Quaternion:
    """Quaternion stored as (w, x, y, z).

    Convention used throughout this project:
      - q represents the rotation that maps BODY vectors into WORLD:
            v_w = q ⊗ v_b ⊗ q*
      - Composition: (q2 * q1) applies q1, then q2.

    WORLD frame is arbitrary but right-handed.
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

    def normalized(self) -> "Quaternion":
        n2 = self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z
        if not math.isfinite(n2) or n2 <= 0.0:
            return Quaternion.identity()
        inv = 1.0 / math.sqrt(n2)
        return Quaternion(self.w*inv, self.x*inv, self.y*inv, self.z*inv)

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

    # -------------------------- constructors --------------------------

    @staticmethod
    def from_axis_angle(axis: Iterable[float], angle_rad: float) -> "Quaternion":
        ax, ay, az = [float(v) for v in axis]
        n = math.sqrt(ax*ax + ay*ay + az*az)
        if n <= 1e-12:
            return Quaternion.identity()
        ax, ay, az = ax/n, ay/n, az/n
        s = math.sin(0.5 * angle_rad)
        return Quaternion(
            math.cos(0.5 * angle_rad),
            ax * s,
            ay * s,
            az * s,
        ).normalized()

    @staticmethod
    def from_two_vectors(u: Iterable[float], v: Iterable[float]) -> "Quaternion":
        """Return quaternion rotating unit vector u to unit vector v.

        Robust for nearly-opposite vectors.
        """
        u = np.asarray(list(u), dtype=float)
        v = np.asarray(list(v), dtype=float)
        nu = float(np.linalg.norm(u))
        nv = float(np.linalg.norm(v))
        if nu <= 1e-12 or nv <= 1e-12:
            return Quaternion.identity()
        u = u / nu
        v = v / nv

        d = float(np.dot(u, v))
        if d >= 1.0 - 1e-12:
            return Quaternion.identity()

        if d <= -1.0 + 1e-12:
            # 180 deg: pick an arbitrary orthogonal axis
            # choose axis orthogonal to u with largest component stability
            if abs(u[0]) < abs(u[1]):
                axis = np.cross(u, np.array([1.0, 0.0, 0.0]))
            else:
                axis = np.cross(u, np.array([0.0, 1.0, 0.0]))
            axis_n = float(np.linalg.norm(axis))
            if axis_n <= 1e-12:
                axis = np.cross(u, np.array([0.0, 0.0, 1.0]))
                axis_n = float(np.linalg.norm(axis))
            axis = axis / max(axis_n, 1e-12)
            return Quaternion.from_axis_angle(axis, math.pi)

        c = np.cross(u, v)
        q = Quaternion(1.0 + d, float(c[0]), float(c[1]), float(c[2]))
        return q.normalized()

    @staticmethod
    def from_rotation_matrix(R: np.ndarray) -> "Quaternion":
        """Convert 3x3 rotation matrix to quaternion.

        Uses a numerically-stable branch method.
        Expects a proper rotation matrix.
        """
        R = np.asarray(R, dtype=float)
        if R.shape != (3, 3):
            raise ValueError("R must be 3x3")

        tr = float(R[0, 0] + R[1, 1] + R[2, 2])
        if tr > 0.0:
            S = math.sqrt(tr + 1.0) * 2.0  # S=4*qw
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        else:
            # Find the major diagonal element
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

    def to_rotation_matrix(self) -> np.ndarray:
        q = self.normalized()
        w, x, y, z = q.w, q.x, q.y, q.z
        # Body->World
        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
            [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
            [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
        ], dtype=float)

    # -------------------------- operations --------------------------

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
        q = self
        # q_dot = 0.5 * q ⊗ [0, gx, gy, gz]
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

    Aerospace sequence: Z (yaw), Y (pitch), X (roll).
    This matches the Madgwick update math in this project.
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
    return 180.0 if a == -180.0 else a
