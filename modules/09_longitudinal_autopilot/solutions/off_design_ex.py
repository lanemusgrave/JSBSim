"""Module 09 exercise - solution: fixed vs re-designed gains across speed."""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import control  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.autopilot import engage_longitudinal  # noqa: E402
from gnclab.linear import linearize_fd  # noqa: E402
from gnclab.metrics import step_metrics  # noqa: E402
from gnclab.trim import trim  # noqa: E402


def trimmed(Va):
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": Va / 0.3048})
    trim(fdm, "full")
    return fdm


def design(Va_mps):
    fdm = trimmed(Va_mps)
    Va = fdm["velocities/vt-fps"]
    lin = linearize_fd(fdm, ["fcs/elevator-cmd-norm", "fcs/throttle-cmd-norm"]).subsystem(
        ["Vt", "Alpha", "Theta", "Q"], ["fcs/elevator-cmd-norm", "fcs/throttle-cmd-norm"])
    A, B = lin.A, lin.B
    a1, a2, a3 = -A[3, 3], -A[3, 1], B[3, 0]
    aV1, aV2 = -A[0, 0], B[0, 1]
    kp_th = 1.0 / np.radians(15.0) * np.sign(a3)
    w_th = np.sqrt(a2 + kp_th * a3)
    kd_th = (2 * 0.75 * w_th - a1) / a3

    def tf_of(row):
        C = np.zeros((1, 4))
        C[0, row] = 1.0
        return control.tf(control.ss(A, B[:, [0]], C, 0.0))

    th, q = tf_of(2), tf_of(3)
    K = float(np.real(control.dcgain(kp_th * th / (1 + kp_th * th + kd_th * q))))
    w_h, z_h = w_th / 60.0, 1.5
    w_V, z_V = 0.35, 0.9
    return {"ap/gains/kp-theta": kp_th, "ap/gains/kd-theta": kd_th,
            "ap/gains/kp-h": 2 * z_h * w_h / (K * Va), "ap/gains/ki-h": w_h ** 2 / (K * Va),
            "ap/gains/kp-v": (2 * z_V * w_V - aV1) / aV2, "ap/gains/ki-v": w_V ** 2 / aV2}


def alt_step(Va, gains=None):
    fdm = trimmed(Va)
    for k, v in (gains or {}).items():
        fdm[k] = v
    engage_longitudinal(fdm)
    h0, da0 = fdm["position/h-sl-ft"], fdm["fcs/aileron-cmd-norm"]

    def cb(f, t):
        if t >= 2.0:
            f["ap/alt-cmd-ft"] = h0 + 100.0
        f["fcs/aileron-cmd-norm"] = da0 - 1.5 * f["attitude/phi-rad"] - 0.3 * f["velocities/p-rad_sec"]

    df = run(fdm, 40.0, {"h": "position/h-sl-ft", "de": "ap/elevator-cmd-norm"}, callback=cb, record_every=6)
    m = step_metrics(df.index, df.h, h0 + 100, h0, t_step=2.0, band=0.10)
    return m.overshoot_pct, m.settling_time, df.de.abs().max()


rows = []
for Va in ((18.0, 32.0) if args.fast else (18.0, 25.0, 32.0)):
    g = design(Va)
    for label, gains in (("fixed 25 m/s gains", None), ("re-designed", g)):
        os_, ts, de = alt_step(Va, gains)
        rows.append({"Va [m/s]": Va, "gains": label, "overshoot %": os_, "settle 10 ft [s]": ts, "max |de_ap|": de,
                     "kp-theta": g["ap/gains/kp-theta"] if gains else -3.82, "kp-h": g["ap/gains/kp-h"] if gains else 0.0103})
print(pd.DataFrame(rows).round(3).to_string(index=False))
