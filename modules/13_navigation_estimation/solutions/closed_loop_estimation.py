"""Module 13 - close the altitude hold on raw sensors vs on estimates.

Same autopilot (gnclab.controllers.LongitudinalAP), three feedback sources:
  truth      : JSBSim states
  raw        : baro altitude (0.6 m noise), gyro q (with bias), "theta" from the
               complementary filter, pitot airspeed
  estimated  : AltitudeKF (baro + accelerometer) altitude, EKF theta, gyro q
Measures altitude-hold accuracy and elevator activity in a 100 ft step + hold.

    python modules/13_navigation_estimation/solutions/closed_loop_estimation.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.controllers import LongitudinalAP  # noqa: E402
from gnclab.estimation import AltitudeKF, AttitudeEKF, ComplementaryAttitude, Sensors  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402
from gnclab.trim import trim  # noqa: E402

FT = 0.3048
T = 30.0 if args.fast else 50.0


def fly(source):
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / FT})
    trim(fdm, "full")
    fdm["fcs/actuators-on"] = 1
    dt = fdm.get_delta_t()
    sens = Sensors(seed=3)
    cf, ekf = ComplementaryAttitude(), AttitudeEKF()
    th0 = fdm["attitude/theta-rad"]
    cf.theta, ekf.x[1] = th0, th0
    akf = AltitudeKF(fdm["position/h-sl-ft"] * FT)
    state = {}

    def feedback(f):
        return state["fb"]

    ap = LongitudinalAP(fdm, feedback=None if source == "truth" else feedback)
    h0 = fdm["position/h-sl-ft"]

    def cb(f, t):
        if t >= 2:
            ap.alt_cmd = h0 + 100
        m = sens.read(f, t)
        _, th_cf = cf.update(m["gyro"], m["accel"], dt)
        phi_e, th_e = ekf.update(m["gyro"], m["accel"], dt, Va=m["Va"])
        h_e, _ = akf.update(m["accel"], phi_e, th_e, m["baro_h"], dt)
        if source == "raw":
            state["fb"] = {"theta": th_cf, "q": m["gyro"][1], "h_ft": m["baro_h"] / FT, "vt_fps": m["Va"] / FT}
        else:
            state["fb"] = {"theta": th_e, "q": m["gyro"][1], "h_ft": h_e / FT, "vt_fps": m["Va"] / FT}
        ap(f, t)

    df = run(fdm, T, {"alt_ft": "position/h-sl-ft", "elevator_deg": "fcs/elevator-pos-rad",
                      "theta_deg": "attitude/theta-deg"}, callback=cb, record_every=2)
    df["elevator_deg"] = np.degrees(df.elevator_deg)
    hold = df.loc[25.0:]
    de_rate = np.diff(hold.elevator_deg.to_numpy()) / (2 * dt)
    return df, {"feedback": source, "alt error RMS in hold [ft]": float(np.sqrt(np.mean((hold.alt_ft - h0 - 100) ** 2))),
                "elevator rate RMS [deg/s]": float(np.sqrt(np.mean(de_rate ** 2)))}


runs, rows = {}, []
for src in ("truth", "raw", "estimated"):
    df, row = fly(src)
    runs[src] = df
    rows.append(row)
print(pd.DataFrame(rows).round(2).to_string(index=False))
fig, _ = timehistory(runs, ["alt_ft", "theta_deg", "elevator_deg"], title="Altitude hold: truth vs raw sensors vs estimates")
save(fig, "13_navigation_estimation", "closed_loop_estimation", show=args.show)
