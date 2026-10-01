"""Module 04 - fly the m04_glider at fixed elevator settings and recover its polar.

For each elevator setting: release the glider near its glide speed, let the
phugoid die out, and average the last part of the flight.  In a steady glide
    L/D = 1 / tan(-gamma),   CL = 2 W cos(gamma) / (rho V^2 S)
which we compare with the drag polar typed into the XML (CD = 0.020 + 0.0333 CL^2).

    python modules/04_aircraft_anatomy/solutions/glider_polar.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.plotting import save  # noqa: E402

W_N, S = 5.0 * 9.80665, 0.80
CD0, K = 0.020, 0.0333
SLUGFT3_TO_KGM3, FT2M = 515.379, 0.3048

settings = [-0.15, -0.05, 0.05] if args.fast else np.linspace(-0.20, 0.10, 7)
rows = []
for de in settings:
    fdm = make_fdm("m04_glider")
    initialize(fdm, {"lat-geod-deg": 33, "h-sl-ft": 3000, "vt-fps": 45.0}, engines_running=False)
    fdm["fcs/elevator-cmd-norm"] = de
    df = run(fdm, 150.0 if args.fast else 300.0,
             {"V": "velocities/vt-fps", "gamma": "flight-path/gamma-rad", "rho": "atmosphere/rho-slugs_ft3",
              "alpha": "aero/alpha-deg", "h": "position/h-agl-ft"}, record_every=12)
    tail = df.iloc[int(len(df) * 0.6):]           # last 40%: phugoid has decayed
    V = tail.V.mean() * FT2M
    gamma = tail.gamma.mean()
    rho = tail.rho.mean() * SLUGFT3_TO_KGM3
    CL = 2 * W_N * np.cos(gamma) / (rho * V ** 2 * S)
    rows.append({"de_cmd": de, "V_mps": V, "alpha_deg": tail.alpha.mean(), "gamma_deg": np.degrees(gamma),
                 "L/D": 1 / np.tan(-gamma), "CL": CL, "CD": CL * np.tan(-gamma),
                 "sink_mps": -V * np.sin(gamma)})
res = pd.DataFrame(rows)
print(res.round(3).to_string(index=False))

cl = np.linspace(0.2, 1.2, 50)
best = res.loc[res["L/D"].idxmax()]
print(f"\nbest measured L/D = {best['L/D']:.1f} at CL = {best.CL:.2f}, V = {best.V_mps:.1f} m/s; "
      f"theory: L/D_max = {1 / (2 * np.sqrt(CD0 * K)):.1f} at CL* = {np.sqrt(CD0 / K):.2f}")

fig, ax = plt.subplots(1, 2, figsize=(11, 4.5))
ax[0].plot(CD0 + K * cl ** 2, cl, label="XML polar")
ax[0].plot(res.CD, res.CL, "o", label="flown (steady glide)")
ax[0].set_xlabel("CD")
ax[0].set_ylabel("CL")
ax[0].legend()
ax[0].grid(alpha=0.3)
ax[1].plot(res.V_mps, res.sink_mps, "o-")
ax[1].invert_yaxis()
ax[1].set_xlabel("airspeed [m/s]")
ax[1].set_ylabel("sink rate [m/s]")
ax[1].set_title("speed polar")
ax[1].grid(alpha=0.3)
fig.suptitle("m04_glider: drag polar recovered from simulated glides")
fig.tight_layout()
save(fig, "04_aircraft_anatomy", "glider_polar", show=args.show)
