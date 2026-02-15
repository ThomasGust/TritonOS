from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .quaternion import Quaternion


@dataclass
class MadgwickConfig:
    beta: float = 0.08  # default works well at 100–200 Hz for many IMUs


class MadgwickAHRS:
    """Madgwick AHRS (quaternion) with IMU-only fallback.

    - update_imu(): gyro + accel
    - update_ahrs(): gyro + accel + mag

    Notes:
    - Provide gyro in rad/s, accel in m/s^2, mag in uT.
    - Internally accel/mag are normalized.
    """

    def __init__(self, *, cfg: MadgwickConfig | None = None):
        self.cfg = cfg or MadgwickConfig()
        self.q = Quaternion.identity()

    def reset(self) -> None:
        self.q = Quaternion.identity()

    def update(
        self,
        gyro_rad_s: np.ndarray,
        accel_m_s2: np.ndarray,
        dt: float,
        mag_uT: Optional[np.ndarray] = None,
    ) -> Quaternion:
        if mag_uT is None:
            self.q = self._update_imu(self.q, gyro_rad_s, accel_m_s2, dt)
        else:
            self.q = self._update_ahrs(self.q, gyro_rad_s, accel_m_s2, mag_uT, dt)
        return self.q

    @staticmethod
    def _normalize3(v: np.ndarray) -> Optional[np.ndarray]:
        n = float(np.linalg.norm(v))
        if not math.isfinite(n) or n <= 1e-12:
            return None
        return v / n

    def _update_imu(self, q: Quaternion, gyro: np.ndarray, accel: np.ndarray, dt: float) -> Quaternion:
        beta = float(self.cfg.beta)

        a = self._normalize3(accel)
        if a is None:
            return q.integrate_gyro(gyro, dt)
        ax, ay, az = float(a[0]), float(a[1]), float(a[2])

        # Shorthand
        q1, q2, q3, q4 = q.w, q.x, q.y, q.z

        # Objective function f(q)
        f1 = 2.0*(q2*q4 - q1*q3) - ax
        f2 = 2.0*(q1*q2 + q3*q4) - ay
        f3 = 2.0*(0.5 - q2*q2 - q3*q3) - az

        # Jacobian transpose times f -> gradient
        s1 = -2.0*q3*f1 + 2.0*q2*f2
        s2 =  2.0*q4*f1 + 2.0*q1*f2 - 4.0*q2*f3
        s3 = -2.0*q1*f1 + 2.0*q4*f2 - 4.0*q3*f3
        s4 =  2.0*q2*f1 + 2.0*q3*f2

        # Normalize step
        sn = math.sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4)
        if sn > 1e-12:
            s1, s2, s3, s4 = s1/sn, s2/sn, s3/sn, s4/sn

        gx, gy, gz = float(gyro[0]), float(gyro[1]), float(gyro[2])

        # Quaternion derivative from gyros
        qDot1 = 0.5*(-q2*gx - q3*gy - q4*gz) - beta*s1
        qDot2 = 0.5*( q1*gx + q3*gz - q4*gy) - beta*s2
        qDot3 = 0.5*( q1*gy - q2*gz + q4*gx) - beta*s3
        qDot4 = 0.5*( q1*gz + q2*gy - q3*gx) - beta*s4

        qn = Quaternion(
            q1 + qDot1*dt,
            q2 + qDot2*dt,
            q3 + qDot3*dt,
            q4 + qDot4*dt,
        ).normalized()
        return qn

    def _update_ahrs(self, q: Quaternion, gyro: np.ndarray, accel: np.ndarray, mag: np.ndarray, dt: float) -> Quaternion:
        beta = float(self.cfg.beta)

        a = self._normalize3(accel)
        m = self._normalize3(mag)
        if a is None or m is None:
            return q.integrate_gyro(gyro, dt)

        ax, ay, az = float(a[0]), float(a[1]), float(a[2])
        mx, my, mz = float(m[0]), float(m[1]), float(m[2])

        q1, q2, q3, q4 = q.w, q.x, q.y, q.z

        # Reference direction of Earth's magnetic field (computed from current q)
        # h = q ⊗ m ⊗ q*
        hx = 2.0*mx*(0.5 - q3*q3 - q4*q4) + 2.0*my*(q2*q3 - q1*q4) + 2.0*mz*(q2*q4 + q1*q3)
        hy = 2.0*mx*(q2*q3 + q1*q4) + 2.0*my*(0.5 - q2*q2 - q4*q4) + 2.0*mz*(q3*q4 - q1*q2)
        bx = math.sqrt(hx*hx + hy*hy)
        bz = 2.0*mx*(q2*q4 - q1*q3) + 2.0*my*(q3*q4 + q1*q2) + 2.0*mz*(0.5 - q2*q2 - q3*q3)

        # Objective function: gravity + magnetic field
        f1 = 2.0*(q2*q4 - q1*q3) - ax
        f2 = 2.0*(q1*q2 + q3*q4) - ay
        f3 = 2.0*(0.5 - q2*q2 - q3*q3) - az
        f4 = 2.0*bx*(0.5 - q3*q3 - q4*q4) + 2.0*bz*(q2*q4 - q1*q3) - mx
        f5 = 2.0*bx*(q2*q3 - q1*q4) + 2.0*bz*(q1*q2 + q3*q4) - my
        f6 = 2.0*bx*(q1*q3 + q2*q4) + 2.0*bz*(0.5 - q2*q2 - q3*q3) - mz

        # Gradient step (J^T f)
        s1 = (
            -2.0*q3*f1 + 2.0*q2*f2
            - 2.0*bz*q3*f4 + (-2.0*bx*q4 + 2.0*bz*q2)*f5 + 2.0*bx*q3*f6
        )
        s2 = (
            2.0*q4*f1 + 2.0*q1*f2 - 4.0*q2*f3
            + 2.0*bz*q4*f4 + (2.0*bx*q3 + 2.0*bz*q1)*f5 + (2.0*bx*q4 - 4.0*bz*q2)*f6
        )
        s3 = (
            -2.0*q1*f1 + 2.0*q4*f2 - 4.0*q3*f3
            + (-4.0*bx*q3 - 2.0*bz*q1)*f4 + (2.0*bx*q2 + 2.0*bz*q4)*f5 + (2.0*bx*q1 - 4.0*bz*q3)*f6
        )
        s4 = (
            2.0*q2*f1 + 2.0*q3*f2
            + (-4.0*bx*q4 + 2.0*bz*q2)*f4 + (-2.0*bx*q1 + 2.0*bz*q3)*f5 + 2.0*bx*q2*f6
        )

        sn = math.sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4)
        if sn > 1e-12:
            s1, s2, s3, s4 = s1/sn, s2/sn, s3/sn, s4/sn

        gx, gy, gz = float(gyro[0]), float(gyro[1]), float(gyro[2])

        qDot1 = 0.5*(-q2*gx - q3*gy - q4*gz) - beta*s1
        qDot2 = 0.5*( q1*gx + q3*gz - q4*gy) - beta*s2
        qDot3 = 0.5*( q1*gy - q2*gz + q4*gx) - beta*s3
        qDot4 = 0.5*( q1*gz + q2*gy - q3*gx) - beta*s4

        qn = Quaternion(
            q1 + qDot1*dt,
            q2 + qDot2*dt,
            q3 + qDot3*dt,
            q4 + qDot4*dt,
        ).normalized()
        return qn
