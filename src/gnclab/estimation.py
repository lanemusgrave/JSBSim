"""Sensor models and state estimators for gnc_trainer (Beard & McLain ch. 7-8).

All SI units inside (m, m/s, rad, rad/s).  JSBSim provides the truth; the
:class:`Sensors` class corrupts it the way a small-UAS avionics suite would,
and the estimators reconstruct the states the autopilot needs.

* :class:`Sensors`              gyro (bias + noise), accelerometer (specific force), baro, pitot, GPS
* :class:`ComplementaryAttitude` gyro integration + accelerometer "gravity vector" tilt
* :class:`AttitudeEKF`          B&M sec. 8.6: states [phi, theta], gyro-driven, accel update using airspeed
* :class:`AltitudeKF`           states [h, h_dot]: accelerometer-driven, baro update
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.80665
FT = 0.3048


def truth(fdm) -> dict:
    """Truth states (SI) read from JSBSim."""
    m = fdm["inertia/mass-slugs"]
    # accelerometer = specific force = non-gravitational force / mass (body axes)
    fx = fdm["forces/fbx-total-lbs"] / m * FT
    fy = fdm["forces/fby-total-lbs"] / m * FT
    fz = fdm["forces/fbz-total-lbs"] / m * FT
    return {
        "p": fdm["velocities/p-rad_sec"], "q": fdm["velocities/q-rad_sec"], "r": fdm["velocities/r-rad_sec"],
        "phi": fdm["attitude/phi-rad"], "theta": fdm["attitude/theta-rad"], "psi": fdm["attitude/psi-rad"],
        "f": np.array([fx, fy, fz]),
        "h": fdm["position/h-sl-ft"] * FT, "hdot": fdm["velocities/h-dot-fps"] * FT,
        "Va": fdm["velocities/vt-fps"] * FT,
        "Vg": fdm["velocities/vg-fps"] * FT, "chi": fdm["flight-path/psi-gt-rad"],
    }


@dataclass
class Sensors:
    """Small-UAS sensor suite.  Noise values are 1-sigma; call :meth:`read` every frame."""

    gyro_sigma: float = 0.0025          # rad/s
    gyro_bias: tuple = (0.004, -0.003, 0.002)   # rad/s (~0.2 deg/s): constant, unknown to the filters
    accel_sigma: float = 0.05           # m/s^2 (0.005 g)
    baro_sigma: float = 0.6             # m
    pitot_sigma: float = 0.3            # m/s
    gps_sigma_h: float = 1.5            # m (horizontal); vertical uses 2x
    gps_rate_hz: float = 5.0
    seed: int = 0
    rng: np.random.Generator = field(init=False)

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)
        self._gps_next = 0.0
        self.gps = None

    def read(self, fdm, t: float) -> dict:
        tr = truth(fdm)
        n = self.rng.normal
        out = {
            "gyro": np.array([tr["p"], tr["q"], tr["r"]]) + np.array(self.gyro_bias) + n(0, self.gyro_sigma, 3),
            "accel": tr["f"] + n(0, self.accel_sigma, 3),
            "baro_h": tr["h"] + n(0, self.baro_sigma),
            "Va": tr["Va"] + n(0, self.pitot_sigma),
            "gps_new": False,
        }
        if t + 1e-9 >= self._gps_next:
            self._gps_next += 1.0 / self.gps_rate_hz
            self.gps = {"h": tr["h"] + n(0, 2 * self.gps_sigma_h), "Vg": tr["Vg"] + n(0, 0.1),
                        "chi": tr["chi"] + n(0, 0.1 / max(tr["Vg"], 1.0))}
            out["gps_new"] = True
        out["gps"] = self.gps
        return out


def euler_rates(phi, theta, p, q, r):
    t, c, s = math.tan(theta), math.cos(phi), math.sin(phi)
    return p + (q * s + r * c) * t, q * c - r * s


class ComplementaryAttitude:
    """phi, theta = gyro integration, slowly pulled toward the accelerometer tilt.

    The accelerometer tilt assumes the only specific force is "1 g up" - true in
    steady straight flight, FALSE in a coordinated turn (the specific force then
    stays along body -z, so the accel says "wings level").  Crossover tau [s].
    """

    def __init__(self, tau: float = 2.0):
        self.tau = tau
        self.phi = self.theta = 0.0

    def update(self, gyro, accel, dt, **_):
        p, q, r = gyro
        phid, thd = euler_rates(self.phi, self.theta, p, q, r)
        ax, ay, az = accel
        phi_a = math.atan2(-ay, -az)
        theta_a = math.atan2(ax, math.hypot(ay, az))
        k = dt / (self.tau + dt)
        self.phi = (1 - k) * (self.phi + phid * dt) + k * phi_a
        self.theta = (1 - k) * (self.theta + thd * dt) + k * theta_a
        return self.phi, self.theta


class AttitudeEKF:
    """B&M sec. 8.6 attitude EKF.  x = [phi, theta]; gyro drives the prediction,
    the accelerometer corrects it using the steady-flight model
        f_x =  g sin(theta)
        f_y =  r Va - g cos(theta) sin(phi)
        f_z = -q Va - g cos(theta) cos(phi)
    which explains the turn's centripetal acceleration through r*Va and q*Va.
    """

    def __init__(self, q_proc: float = 1e-6, r_accel: float = 0.05 ** 2 * 25):
        self.x = np.zeros(2)
        self.P = np.eye(2) * 0.1
        self.Q = np.eye(2) * q_proc
        self.R = np.eye(3) * r_accel      # inflated: the model ignores u_dot, alpha, gusts

    def predict(self, gyro, dt):
        p, q, r = gyro
        phi, th = self.x
        phid, thd = euler_rates(phi, th, p, q, r)
        self.x = self.x + dt * np.array([phid, thd])
        c, s, t, ct = math.cos(phi), math.sin(phi), math.tan(th), math.cos(th)
        A = np.array([[(q * c - r * s) * t, (q * s + r * c) / ct ** 2],
                      [-q * s - r * c, 0.0]])
        Ad = np.eye(2) + dt * A
        self.P = Ad @ self.P @ Ad.T + self.Q * dt

    def correct(self, gyro, accel, Va):
        p, q, r = gyro
        phi, th = self.x
        c, s, ct, st = math.cos(phi), math.sin(phi), math.cos(th), math.sin(th)
        h = np.array([G * st, r * Va - G * ct * s, -q * Va - G * ct * c])
        H = np.array([[0.0, G * ct],
                      [-G * ct * c, G * st * s],
                      [G * ct * s, G * st * c]])
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (np.asarray(accel) - h)
        self.P = (np.eye(2) - K @ H) @ self.P

    def update(self, gyro, accel, dt, Va=25.0):
        self.predict(gyro, dt)
        self.correct(gyro, accel, Va)
        return tuple(self.x)


class AltitudeKF:
    """x = [h, h_dot].  Prediction uses the vertical acceleration from the
    accelerometer rotated with the estimated attitude; baro corrects h."""

    def __init__(self, h0: float, q_acc: float = 0.5 ** 2, r_baro: float = 0.6 ** 2):
        self.x = np.array([h0, 0.0])
        self.P = np.diag([1.0, 1.0])
        self.q_acc, self.R = q_acc, r_baro

    def update(self, accel, phi, theta, baro_h, dt):
        fx, fy, fz = accel
        f_down = -math.sin(theta) * fx + math.sin(phi) * math.cos(theta) * fy + math.cos(phi) * math.cos(theta) * fz
        h_ddot = -(f_down + G)                          # up-positive vertical acceleration
        F = np.array([[1, dt], [0, 1]])
        self.x = F @ self.x + np.array([0.5 * dt * dt, dt]) * h_ddot
        Gq = np.array([[0.5 * dt * dt], [dt]])
        self.P = F @ self.P @ F.T + Gq @ Gq.T * self.q_acc
        Hm = np.array([[1.0, 0.0]])
        S = Hm @ self.P @ Hm.T + self.R
        K = self.P @ Hm.T / S
        self.x = self.x + (K * (baro_h - self.x[0])).ravel()
        self.P = (np.eye(2) - K @ Hm) @ self.P
        return tuple(self.x)
