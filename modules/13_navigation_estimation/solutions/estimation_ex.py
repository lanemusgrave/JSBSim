"""Module 13 exercise A - solution: attitude EKF with gyro-bias states."""

import math

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.autopilot import engage_lateral, engage_longitudinal  # noqa: E402
from gnclab.estimation import G, AttitudeEKF, Sensors, euler_rates, truth  # noqa: E402
from gnclab.trim import trim  # noqa: E402


class BiasEKF:
    """x = [phi, theta, bp, bq, br]; gyro_true = gyro_meas - b."""

    def __init__(self, q_att=1e-6, q_bias=1e-9, r_accel=0.05 ** 2 * 25):
        self.x = np.zeros(5)
        self.P = np.diag([0.1, 0.1, 1e-4, 1e-4, 1e-4])
        self.Q = np.diag([q_att, q_att, q_bias, q_bias, q_bias])
        self.R = np.eye(3) * r_accel

    def update(self, gyro, accel, dt, Va):
        phi, th, bp, bq, br = self.x
        p, q, r = np.asarray(gyro) - self.x[2:]
        phid, thd = euler_rates(phi, th, p, q, r)
        self.x[:2] += dt * np.array([phid, thd])
        c, s, t, ct = math.cos(phi), math.sin(phi), math.tan(th), math.cos(th)
        A = np.zeros((5, 5))
        A[0, :2] = [(q * c - r * s) * t, (q * s + r * c) / ct ** 2]
        A[1, :2] = [-q * s - r * c, 0.0]
        A[0, 2:] = [-1.0, -s * t, -c * t]            # d(phidot)/d(b)
        A[1, 2:] = [0.0, -c, s]                       # d(thetadot)/d(b)
        Ad = np.eye(5) + dt * A
        self.P = Ad @ self.P @ Ad.T + self.Q * dt
        phi, th = self.x[:2]
        c, s, ct, st = math.cos(phi), math.sin(phi), math.cos(th), math.sin(th)
        h = np.array([G * st, r * Va - G * ct * s, -q * Va - G * ct * c])
        H = np.zeros((3, 5))
        H[0, :2] = [0.0, G * ct]
        H[1, :2] = [-G * ct * c, G * st * s]
        H[2, :2] = [G * ct * s, G * st * c]
        H[1, 4] = -Va                                 # r = r_meas - br
        H[2, 3] = Va                                  # q = q_meas - bq
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (np.asarray(accel) - h)
        self.P = (np.eye(5) - K @ H) @ self.P
        return phi, th


fdm = make_fdm("gnc_trainer")
initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048, "psi-true-deg": 0})
trim(fdm, "full")
engage_longitudinal(fdm)
engage_lateral(fdm)
dt = fdm.get_delta_t()
sens = Sensors(seed=1)
e2, e5 = AttitudeEKF(), BiasEKF()
e2.x[1] = e5.x[1] = fdm["attitude/theta-rad"]
err2, err5 = [], []


def cb(f, t):
    f["ap/chi-cmd-rad"] = math.radians(90) * (1 if (t // 20) % 2 else -1) if t > 5 else 0.0
    m = sens.read(f, t)
    a2 = e2.update(m["gyro"], m["accel"], dt, Va=m["Va"])
    e5.update(m["gyro"], m["accel"], dt, Va=m["Va"])
    tr = truth(f)
    err2.append(np.degrees([a2[0] - tr["phi"], a2[1] - tr["theta"]]))
    err5.append(np.degrees([e5.x[0] - tr["phi"], e5.x[1] - tr["theta"]]))


run(fdm, 40.0 if args.fast else 90.0, {"h": "position/h-sl-ft"}, callback=cb, record_every=120)
err2, err5 = np.array(err2), np.array(err5)
late = slice(len(err2) // 2, None)
print(f"true gyro bias        [deg/s]: {np.degrees(sens.gyro_bias).round(3)}")
print(f"estimated bias (end)  [deg/s]: {np.degrees(e5.x[2:]).round(3)}")
for name, e in (("2-state EKF", err2), ("5-state (bias) EKF", err5)):
    print(f"{name:20s} second-half RMS: roll {np.sqrt(np.mean(e[late, 0] ** 2)):.2f} deg, "
          f"pitch {np.sqrt(np.mean(e[late, 1] ** 2)):.2f} deg")
