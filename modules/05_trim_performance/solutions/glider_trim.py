"""Module 05 - write your own trim: the glider JSBSim's trim routine can't do.

JSBSim's longitudinal trim pairs u-dot with the THROTTLE.  A glider has no
throttle, so u-dot can only be zeroed by choosing the flight-path angle.  We
solve the equilibrium directly:

    unknowns  x = [alpha, gamma, pitch_trim]       (airspeed V is given)
    residuals r = [u_dot, w_dot, q_dot] = 0

with scipy.optimize.least_squares.  Each evaluation writes the ICs and calls
run_ic(), which runs every JSBSim model once with integration suspended - so
the residuals come from the *real* model (FCS, aero tables, mass properties).

    python modules/05_trim_performance/solutions/glider_trim.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.plotting import save  # noqa: E402

fdm = make_fdm("m04_glider")
initialize(fdm, {"lat-geod-deg": 33, "h-sl-ft": 3000, "vt-fps": 48.0}, engines_running=False)


def residuals(x, V_fps):
    alpha, gamma, trim_cmd = x
    fdm["ic/vt-fps"] = V_fps
    fdm["ic/alpha-rad"] = alpha
    fdm["ic/gamma-rad"] = gamma
    fdm["ic/q-rad_sec"] = 0.0
    fdm["fcs/pitch-trim-cmd-norm"] = trim_cmd
    fdm.run_ic()
    return [fdm["accelerations/udot-ft_sec2"],
            fdm["accelerations/wdot-ft_sec2"],
            10.0 * fdm["accelerations/qdot-rad_sec2"]]  # weight: rad/s^2 are "small"


rows = []
for V_mps in np.arange(10.0, 24.1, 4.0 if args.fast else 1.0):
    V = V_mps / 0.3048
    sol = least_squares(residuals, [0.05, -0.05, 0.0], args=(V,),
                        bounds=([-0.2, -0.6, -1.0], [0.25, 0.2, 1.0]), xtol=1e-12, ftol=1e-12)
    a, g, de = sol.x
    rows.append({"V_mps": V_mps, "alpha_deg": np.degrees(a), "gamma_deg": np.degrees(g),
                 "trim_cmd": de, "elevator_deg": np.degrees(fdm["fcs/elevator-pos-rad"]),
                 "L/D": -1 / np.tan(g), "sink_mps": -V_mps * np.sin(g), "max_resid": np.max(np.abs(sol.fun))})
res = pd.DataFrame(rows)
print(res.round(4).to_string(index=False))
best = res.loc[res["L/D"].idxmax()]
print(f"\nbest glide: L/D {best['L/D']:.1f} at {best.V_mps:.0f} m/s;  "
      f"min sink {res.sink_mps.min():.2f} m/s at {res.V_mps[res.sink_mps.idxmin()]:.0f} m/s")

# Prove it IS an equilibrium: start exactly there and fly 30 s hands-off.
i = res["L/D"].idxmax()
residuals([np.radians(res.alpha_deg[i]), np.radians(res.gamma_deg[i]), res.trim_cmd[i]], res.V_mps[i] / 0.3048)
df = run(fdm, 30.0, {"V": "velocities/vt-fps", "alpha": "aero/alpha-deg", "gamma": "flight-path/gamma-deg"})
print(f"30 s from the trim point: V drift {df.V.max() - df.V.min():.4f} ft/s, "
      f"alpha drift {df.alpha.max() - df.alpha.min():.4f} deg")

fig, ax = plt.subplots(1, 2, figsize=(11, 4.5))
ax[0].plot(res.V_mps, res.sink_mps, "o-")
ax[0].invert_yaxis()
ax[0].set(xlabel="airspeed [m/s]", ylabel="sink [m/s]", title="speed polar from custom trim")
ax[1].plot(res.V_mps, res.elevator_deg, "o-")
ax[1].set(xlabel="airspeed [m/s]", ylabel="trim elevator [deg]", title="elevator to trim")
for a_ in ax:
    a_.grid(alpha=0.3)
fig.tight_layout()
save(fig, "05_trim_performance", "glider_trim", show=args.show)
