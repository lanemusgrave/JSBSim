"""Module 02 - reference implementations of the basic frame math.

Conventions (standard aerospace, same as JSBSim):
  NED local frame: x north, y east, z down.
  Body frame: x out the nose, y out the right wing, z out the belly.
  Euler angles: 3-2-1 sequence (yaw psi, then pitch theta, then roll phi).
"""

import numpy as np


def dcm_ned_to_body(phi, theta, psi):
    """C_b/n: rotates a vector expressed in NED into body axes (v_b = C v_n)."""
    cph, sph = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cps, sps = np.cos(psi), np.sin(psi)
    return np.array([
        [cth * cps, cth * sps, -sth],
        [sph * sth * cps - cph * sps, sph * sth * sps + cph * cps, sph * cth],
        [cph * sth * cps + sph * sps, cph * sth * sps - sph * cps, cph * cth],
    ])


def dcm_body_to_ned(phi, theta, psi):
    """C_n/b = C_b/n transposed (rotation matrices are orthonormal)."""
    return dcm_ned_to_body(phi, theta, psi).T


def euler_rates(phi, theta, p, q, r):
    """Euler-angle kinematics: body rates -> (phidot, thetadot, psidot).

    Singular at theta = +/-90 deg (gimbal lock) - one reason sims integrate
    quaternions instead.
    """
    tth = np.tan(theta)
    phidot = p + (q * np.sin(phi) + r * np.cos(phi)) * tth
    thetadot = q * np.cos(phi) - r * np.sin(phi)
    psidot = (q * np.sin(phi) + r * np.cos(phi)) / np.cos(theta)
    return phidot, thetadot, psidot


def alpha_beta(u, v, w):
    """Aerodynamic angles from body-axis *air-relative* velocity."""
    V = np.sqrt(u * u + v * v + w * w)
    return np.arctan2(w, u), np.arcsin(v / V), V


def euler_to_quat(phi, theta, psi):
    """3-2-1 Euler angles -> unit quaternion [q0, q1, q2, q3] (scalar first)."""
    c1, s1 = np.cos(psi / 2), np.sin(psi / 2)
    c2, s2 = np.cos(theta / 2), np.sin(theta / 2)
    c3, s3 = np.cos(phi / 2), np.sin(phi / 2)
    return np.array([
        c3 * c2 * c1 + s3 * s2 * s1,
        s3 * c2 * c1 - c3 * s2 * s1,
        c3 * s2 * c1 + s3 * c2 * s1,
        c3 * c2 * s1 - s3 * s2 * c1,
    ])


def quat_to_euler(q):
    q0, q1, q2, q3 = q
    phi = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2))
    theta = np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1))
    psi = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 ** 2 + q3 ** 2))
    return phi, theta, psi
