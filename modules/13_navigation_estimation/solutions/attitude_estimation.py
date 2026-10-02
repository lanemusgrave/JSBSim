"""Module 13 - estimate roll and pitch from a MEMS IMU: complementary filter vs EKF.

The autopilot (truth feedback) flies S-turns: 90 deg right, 90 deg left, with a
climb in between.  Alongside, in Python, a sensor suite (gyro with a constant
bias, noisy accelerometer, pitot) feeds two estimators:
  * a complementary filter that trusts the accelerometer as a "gravity vector"
  * B&M's attitude EKF, whose measurement model includes the turn's
    centripetal acceleration (r*Va, q*Va).

    python modules/13_navigation_estimation/solutions/attitude_estimation.py [--show]
"""

import math

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.autopilot import engage_lateral, engage_longitudinal  # noqa: E402
from gnclab.estimation import AttitudeEKF, ComplementaryAttitude, Sensors, truth  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402
from gnclab.trim import trim  # noqa: E402

fdm = make_fdm("gnc_trainer")
initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048, "psi-true-deg": 0})
trim(fdm, "full")
engage_longitudinal(fdm)
engage_lateral(fdm)
h0 = fdm["position/h-sl-ft"]

sens = Sensors(seed=1)
cf = ComplementaryAttitude(tau=2.0)
ekf = AttitudeEKF()
cf.phi, cf.theta = 0.0, fdm["attitude/theta-rad"]
ekf.x[:] = [0.0, fdm["attitude/theta-rad"]]
dt = fdm.get_delta_t()
log = {k: [] for k in ("t", "phi", "theta", "phi_cf", "theta_cf", "phi_ekf", "theta_ekf")}


def cb(f, t):
    if 5 <= t < 25:
        f["ap/chi-cmd-rad"] = math.radians(90)
    elif t >= 25:
        f["ap/chi-cmd-rad"] = 0.0
    if 15 <= t:
        f["ap/alt-cmd-ft"] = h0 + 100
    m = sens.read(f, t)
    pc, tc = cf.update(m["gyro"], m["accel"], dt)
    pe, te = ekf.update(m["gyro"], m["accel"], dt, Va=m["Va"])
    tr = truth(f)
    for k, v in (("t", t), ("phi", tr["phi"]), ("theta", tr["theta"]), ("phi_cf", pc), ("theta_cf", tc),
                 ("phi_ekf", pe), ("theta_ekf", te)):
        log[k].append(v)


run(fdm, 30.0 if args.fast else 45.0, {"h": "position/h-sl-ft"}, callback=cb)
L = {k: np.array(v) for k, v in log.items()}
turn = (L["t"] > 7) & (L["t"] < 20)
for name in ("cf", "ekf"):
    e_phi = np.degrees(L[f"phi_{name}"] - L["phi"])
    e_th = np.degrees(L[f"theta_{name}"] - L["theta"])
    print(f"{'complementary' if name == 'cf' else 'EKF':14s}: roll error RMS {np.sqrt(np.mean(e_phi ** 2)):5.2f} deg "
          f"(in the turn: max {np.abs(e_phi[turn]).max():5.2f}), pitch error RMS {np.sqrt(np.mean(e_th ** 2)):5.2f} deg")

import pandas as pd  # noqa: E402

df = pd.DataFrame({k: np.degrees(v) for k, v in L.items() if k != "t"}, index=L["t"])
fig, _ = timehistory({"truth": df[["phi", "theta"]],
                      "complementary": df[["phi_cf", "theta_cf"]].set_axis(["phi", "theta"], axis=1),
                      "EKF": df[["phi_ekf", "theta_ekf"]].set_axis(["phi", "theta"], axis=1)},
                     ["phi", "theta"], title="Attitude estimation through S-turns (deg)")
save(fig, "13_navigation_estimation", "attitude_estimation", show=args.show)
