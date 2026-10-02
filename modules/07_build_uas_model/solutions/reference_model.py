"""Module 07 - an INDEPENDENT Python implementation of the gnc_trainer physics.

Beard & McLain chapter 3-4 equations, written directly in numpy from the same
parameter set (imported from build_gnc_trainer.py).  Used to cross-check the
JSBSim model: if two independent implementations of the same data give the
same trim and the same linear modes, both are probably right.

Flat Earth, constant gravity, no wind.  State derivatives are returned for the
same 8 states gnclab.linear.linearize_fd uses:
    [Vt, alpha, theta, q, beta, phi, p, r]   (SI units here)
"""

from __future__ import annotations

import math

import numpy as np
from scipy.optimize import least_squares

from build_gnc_trainer import P, cd_static, cl_static

G = 9.80665
KV = 60.0 / (2 * math.pi * P["KV_rpm_per_volt"])


def motor_prop(throttle: float, Va: float, rho: float) -> tuple[float, float]:
    D, tp = P["D_prop"], 2 * math.pi
    a = rho * D ** 5 * P["CQ0"] / tp ** 2
    b = rho * D ** 4 * P["CQ1"] * Va / tp + KV * KV / P["R_motor"]
    c = rho * D ** 3 * P["CQ2"] * Va ** 2 - KV * P["V_max"] * throttle / P["R_motor"] + KV * P["i0"]
    om = max(0.0, (-b + math.sqrt(max(0.0, b * b - 4 * a * c))) / (2 * a))
    T = rho * (D ** 4 * P["CT0"] * om ** 2 / tp ** 2 + D ** 3 * P["CT1"] * Va * om / tp + D ** 2 * P["CT2"] * Va ** 2)
    Q = rho * (D ** 5 * P["CQ0"] * om ** 2 / tp ** 2 + D ** 4 * P["CQ1"] * Va * om / tp + D ** 3 * P["CQ2"] * Va ** 2)
    return T, Q


def derivatives(x: np.ndarray, u: np.ndarray, rho: float) -> np.ndarray:
    """x = [Vt, alpha, theta, q, beta, phi, p, r]; u = [de, da, dr, throttle] (rad, rad, rad, 0..1)."""
    Vt, al, th, q, be, ph, p, r = x
    de, da, dr, thr = u
    m, S, b, c = P["mass"], P["S"], P["b"], P["c"]
    Jx, Jy, Jz, Jxz = P["Jx"], P["Jy"], P["Jz"], P["Jxz"]
    qbar = 0.5 * rho * Vt ** 2
    # body velocities
    uu, vv, ww = Vt * math.cos(al) * math.cos(be), Vt * math.sin(be), Vt * math.sin(al) * math.cos(be)
    # aero coefficients (B&M 4.x), forces in stability axes -> body
    CL = float(cl_static(np.array(al))) + P["CLq"] * c / (2 * Vt) * q + P["CLde"] * de
    CD = float(cd_static(np.array(al))) + P["CDq"] * c / (2 * Vt) * q + P["CDde"] * abs(de)
    Lift, Drag = qbar * S * CL, qbar * S * CD
    fx = -Drag * math.cos(al) + Lift * math.sin(al)
    fz = -Drag * math.sin(al) - Lift * math.cos(al)
    fy = qbar * S * (P["CYb"] * be + P["CYp"] * b / (2 * Vt) * p + P["CYr"] * b / (2 * Vt) * r
                     + P["CYda"] * da + P["CYdr"] * dr)
    l_ = qbar * S * b * (P["Clb"] * be + P["Clp"] * b / (2 * Vt) * p + P["Clr"] * b / (2 * Vt) * r
                         + P["Clda"] * da + P["Cldr"] * dr)
    m_ = qbar * S * c * (P["Cm0"] + P["Cma"] * al + P["Cmq"] * c / (2 * Vt) * q + P["Cmde"] * de)
    n_ = qbar * S * b * (P["Cnb"] * be + P["Cnp"] * b / (2 * Vt) * p + P["Cnr"] * b / (2 * Vt) * r
                         + P["Cnda"] * da + P["Cndr"] * dr)
    T, Qp = motor_prop(thr, uu, rho)
    fx += T
    l_ -= Qp
    # gravity in body axes
    fx += -m * G * math.sin(th)
    fy += m * G * math.cos(th) * math.sin(ph)
    fz += m * G * math.cos(th) * math.cos(ph)
    # translational dynamics
    ud = r * vv - q * ww + fx / m
    vd = p * ww - r * uu + fy / m
    wd = q * uu - p * vv + fz / m
    # rotational dynamics (B&M 3.13, Gamma terms)
    Gam = Jx * Jz - Jxz ** 2
    G1, G2 = Jxz * (Jx - Jy + Jz) / Gam, (Jz * (Jz - Jy) + Jxz ** 2) / Gam
    G3, G4 = Jz / Gam, Jxz / Gam
    G5, G6 = (Jz - Jx) / Jy, Jxz / Jy
    G7, G8 = ((Jx - Jy) * Jx + Jxz ** 2) / Gam, Jx / Gam
    pd_ = G1 * p * q - G2 * q * r + G3 * l_ + G4 * n_
    qd_ = G5 * p * r - G6 * (p * p - r * r) + m_ / Jy
    rd_ = G7 * p * q - G1 * q * r + G4 * l_ + G8 * n_
    # kinematics
    phd = p + (q * math.sin(ph) + r * math.cos(ph)) * math.tan(th)
    thd = q * math.cos(ph) - r * math.sin(ph)
    # wind-axis variables
    Vd = (uu * ud + vv * vd + ww * wd) / Vt
    ald = (uu * wd - ww * ud) / (uu * uu + ww * ww)
    bed = (Vt * vd - vv * Vd) / (Vt * math.sqrt(uu * uu + ww * ww))
    return np.array([Vd, ald, thd, qd_, bed, phd, pd_, rd_])


def trim_level(Va: float, rho: float) -> tuple[np.ndarray, np.ndarray]:
    """Wings-level, straight-and-level trim: solve alpha, de, throttle, beta(~0), da, dr, phi."""
    def res(z):
        al, de, thr, be, da, dr, ph = z
        x = np.array([Va, al, al, 0.0, be, ph, 0.0, 0.0])  # theta = alpha (gamma = 0)
        d = derivatives(x, np.array([de, da, dr, thr]), rho)
        return [d[0], d[1], d[3], d[4], d[6], d[7], d[5]]
    sol = least_squares(res, [0.05, -0.1, 0.7, 0, 0, 0, 0], xtol=1e-14, ftol=1e-14)
    al, de, thr, be, da, dr, ph = sol.x
    return np.array([Va, al, al, 0.0, be, ph, 0.0, 0.0]), np.array([de, da, dr, thr])


def linearize(x0: np.ndarray, u0: np.ndarray, rho: float, h: float = 1e-6):
    n, m = len(x0), len(u0)
    A, B = np.zeros((n, n)), np.zeros((n, m))
    for j in range(n):
        dx = np.zeros(n)
        dx[j] = h * max(1.0, abs(x0[j]))
        A[:, j] = (derivatives(x0 + dx, u0, rho) - derivatives(x0 - dx, u0, rho)) / (2 * dx[j])
    for j in range(m):
        du = np.zeros(m)
        du[j] = h
        B[:, j] = (derivatives(x0, u0 + du, rho) - derivatives(x0, u0 - du, rho)) / (2 * h)
    return A, B
