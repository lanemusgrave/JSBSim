"""Module 11 - gain scheduling on dynamic pressure.

Elevator effectiveness scales with qbar, so a fixed-gain pitch loop has a
different loop gain - and different closed-loop dynamics - at every speed.
Scaling the pitch gains by qbar_design / qbar keeps them roughly constant.

1. Linear: at 16, 20, 25, 32, 36 m/s, linearize, close the pitch loop (with the
   40 rad/s actuator) with fixed and with scheduled gains, and compare the
   closed-loop short-period poles and the loop's delay margin.
2. Nonlinear: a 5 deg pitch step at 18 and 32 m/s, fixed vs scheduled.

    python modules/11_robustness/solutions/gain_schedule.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import control  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.controllers import DEFAULT_GAINS, LongitudinalAP  # noqa: E402
from gnclab.linear import linearize_fd  # noqa: E402
from gnclab.metrics import loop_margins  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402
from gnclab.trim import trim  # noqa: E402

KP, KD = DEFAULT_GAINS["kp_theta"], DEFAULT_GAINS["kd_theta"]
act = control.tf([40], [1, 40])


def trimmed(Va):
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": Va / 0.3048})
    trim(fdm, "full")
    return fdm


QBAR0 = trimmed(25.0)["aero/qbar-psf"]
rows = []
for Va in ((18.0, 32.0) if args.fast else (16.0, 20.0, 25.0, 32.0, 36.0)):
    try:
        fdm = trimmed(Va)
    except Exception:
        continue
    lin = linearize_fd(fdm, ["fcs/elevator-cmd-norm"]).subsystem(["Vt", "Alpha", "Theta", "Q"], ["fcs/elevator-cmd-norm"])

    def tf_of(i):
        C = np.zeros((1, 4))
        C[0, i] = 1
        return control.tf(control.ss(lin.A, lin.B, C, 0.0)) * act

    th, q = tf_of(2), tf_of(3)
    for label, k in (("fixed", 1.0), ("scheduled", QBAR0 / fdm["aero/qbar-psf"])):
        L = k * (KP * th + KD * q)
        poles = control.poles(L / (1 + L))
        sp = poles[np.argmax(np.abs(poles.imag))]
        m = loop_margins(L)
        rows.append({"Va [m/s]": Va, "qbar [psf]": fdm["aero/qbar-psf"], "gains": label, "gain x": k,
                     "SP wn [rad/s]": abs(sp), "SP zeta": -sp.real / abs(sp),
                     "PM [deg]": m.phase_margin_deg, "delay margin [ms]": 1000 * m.delay_margin_s})
res = pd.DataFrame(rows)
print(res.round(3).to_string(index=False))
for label in ("fixed", "scheduled"):
    sub = res[res.gains == label]
    print(f"{label:9s}: closed-loop SP wn spans {sub['SP wn [rad/s]'].min():.1f}-{sub['SP wn [rad/s]'].max():.1f} rad/s, "
          f"delay margin {sub['delay margin [ms]'].min():.0f}-{sub['delay margin [ms]'].max():.0f} ms")

runs = {}
for Va in (18.0, 32.0):
    for sched in (False, True):
        fdm = trimmed(Va)
        ap = LongitudinalAP(fdm, schedule=(lambda qb: QBAR0 / qb) if sched else None)
        ap.gains["kp_h"] = ap.gains["ki_h"] = 0.0          # pitch-attitude hold only
        th0 = ap.theta_trim

        def cb(f, t, ap=ap, th0=th0):
            ap.theta_trim = th0 + (np.radians(5) if t >= 1 else 0.0)
            ap(f, t)

        df = run(fdm, 4.0, {"theta_deg": "attitude/theta-deg", "q_dps": "velocities/q-rad_sec",
                            "elevator_rad": "fcs/elevator-pos-rad"}, callback=cb)
        df["q_dps"] = np.degrees(df.q_dps)
        runs[f"{Va:.0f} m/s {'scheduled' if sched else 'fixed'}"] = df
fig, _ = timehistory(runs, ["theta_deg", "q_dps", "elevator_rad"], title="5 deg pitch step: fixed vs qbar-scheduled gains")
save(fig, "11_robustness", "gain_schedule", show=args.show)
