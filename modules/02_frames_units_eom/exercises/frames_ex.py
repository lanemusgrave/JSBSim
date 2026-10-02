"""Module 02 exercise: write the frame math yourself and test it on JSBSim data.

Implement the four functions below (no peeking at solutions/frames.py until
you've tried).  Then run this file: it flies a C172 through a rolling pull-up
and checks your functions against JSBSim, sample by sample.

    python modules/02_frames_units_eom/exercises/frames_ex.py

Conventions: NED (x north, y east, z down); body (x nose, y right wing,
z belly); 3-2-1 Euler angles (psi, theta, phi).  All angles in radians.
"""

import numpy as np

from gnclab.cli import parse_args


def dcm_body_to_ned(phi, theta, psi):
    """TODO 1: 3x3 matrix C such that v_ned = C @ v_body.
    Hint: build C_b/n = R_x(phi) R_y(theta) R_z(psi) (NED -> body), then transpose."""
    raise NotImplementedError


def euler_rates(phi, theta, p, q, r):
    """TODO 2: return (phidot, thetadot, psidot) from body rates."""
    raise NotImplementedError


def alpha_beta(u, v, w):
    """TODO 3: return (alpha, beta, V) from body-axis velocity."""
    raise NotImplementedError


def euler_to_quat(phi, theta, psi):
    """TODO 4: unit quaternion [q0, q1, q2, q3] (scalar first) for a 3-2-1 sequence."""
    raise NotImplementedError


# ----------------------------------------------------------------------------
# Test harness (no changes needed below)
# ----------------------------------------------------------------------------
def _check():
    from gnclab import initialize, make_fdm, run
    from gnclab.signals import pulse

    fdm = make_fdm("c172x")
    initialize(fdm, {"lat-geod-deg": 33.67, "h-sl-ft": 4000, "vc-kts": 100, "psi-true-deg": 45})
    fdm.do_trim(1)
    da0, de0 = fdm["fcs/aileron-cmd-norm"], fdm["fcs/elevator-cmd-norm"]

    def pilot(f, t):
        f["fcs/aileron-cmd-norm"] = da0 + pulse(t, 1.0, 1.5, 0.4)
        f["fcs/elevator-cmd-norm"] = de0 + pulse(t, 3.0, 2.0, -0.15)

    cols = {"u": "velocities/u-fps", "v": "velocities/v-fps", "w": "velocities/w-fps",
            "p": "velocities/p-rad_sec", "q": "velocities/q-rad_sec", "r": "velocities/r-rad_sec",
            "phi": "attitude/phi-rad", "theta": "attitude/theta-rad", "psi": "attitude/psi-rad",
            "alpha": "aero/alpha-rad", "beta": "aero/beta-rad",
            "vn": "velocities/v-north-fps", "ve": "velocities/v-east-fps", "vd": "velocities/v-down-fps",
            "phidot": "velocities/phidot-rad_sec", "thetadot": "velocities/thetadot-rad_sec",
            "psidot": "velocities/psidot-rad_sec"}
    df = run(fdm, 8.0, cols, callback=pilot, record_every=10)
    x = {k: df[k].to_numpy() for k in cols}
    results = []

    def test(label, fn):
        try:
            err = fn()
            results.append((label, "PASS" if err < 1e-6 else f"FAIL (max err {err:.3g})"))
        except NotImplementedError:
            results.append((label, "not implemented yet"))

    def t1():
        ned = np.array([dcm_body_to_ned(x["phi"][k], x["theta"][k], x["psi"][k]) @
                        [x["u"][k], x["v"][k], x["w"][k]] for k in range(len(df))])
        jsb = np.c_[x["vn"], x["ve"], x["vd"]]
        return np.max(np.abs(ned - jsb))

    def t2():
        rates = euler_rates(x["phi"], x["theta"], x["p"], x["q"], x["r"])
        jsb = (x["phidot"], x["thetadot"], x["psidot"])
        return max(np.max(np.abs(a - b)) for a, b in zip(rates, jsb))

    def t3():
        a, b, _ = alpha_beta(x["u"], x["v"], x["w"])
        return max(np.max(np.abs(a - x["alpha"])), np.max(np.abs(b - x["beta"])))

    def t4():
        q = euler_to_quat(0.3, -0.2, 2.0)
        # rotating a vector with the quaternion must match the DCM
        q0, qv = q[0], np.asarray(q[1:])
        v = np.array([1.0, 2.0, 3.0])
        # v_ned = q * v_body * q^-1
        v_rot = v + 2 * q0 * np.cross(qv, v) + 2 * np.cross(qv, np.cross(qv, v))
        return max(abs(np.linalg.norm(q) - 1), np.max(np.abs(v_rot - dcm_body_to_ned(0.3, -0.2, 2.0) @ v)))

    test("1 DCM body->NED vs velocities/v-north/east/down", t1)
    test("2 Euler rates vs velocities/*dot-rad_sec", t2)
    test("3 alpha/beta vs aero/alpha-rad, aero/beta-rad", t3)
    test("4 quaternion consistent with DCM", t4)
    for label, res in results:
        print(f"{res:>22s}  {label}")


if __name__ == "__main__":
    parse_args(__doc__)
    _check()
