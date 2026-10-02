"""Module 05 - find the glider's stick-fixed neutral point with a CG sweep.

Classic flight-test method: at each CG, trim at several speeds and plot trim
elevator vs CL.  The slope d(delta_e)/dCL is proportional to the static
margin, so it goes to zero at the neutral point.  Extrapolate the slopes vs
CG to zero -> x_NP.

CG is moved with the BALLAST point mass in aircraft/m04_glider (properties
inertia/pointmass-weight-lbs and inertia/pointmass-location-X-inches).

Theory for this model (ARP at x = 0.40 m, Cm_alpha = -0.80, CL_alpha = 5.0
per rad from the table): x_NP ~ x_ARP + (-Cm_alpha/CL_alpha) cbar
                                = 0.40 + 0.16 * 0.27 = 0.443 m

    python modules/05_trim_performance/solutions/neutral_point.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm  # noqa: E402
from gnclab.plotting import save  # noqa: E402
from gnclab.trim import trim_custom  # noqa: E402

M2IN, KG2LB = 39.3701, 2.20462
BALLAST_KG = 1.0
CBAR = 0.27

fig, ax = plt.subplots(1, 2, figsize=(11, 4.5))
cgs, slopes = [], []
for xb in np.linspace(0.10, 0.70, 4 if args.fast else 7):
    fdm = make_fdm("m04_glider")
    fdm["inertia/pointmass-weight-lbs"] = BALLAST_KG * KG2LB
    fdm["inertia/pointmass-location-X-inches"] = xb * M2IN
    initialize(fdm, {"lat-geod-deg": 33, "h-sl-ft": 3000, "vt-fps": 50.0}, engines_running=False)
    x_cg = fdm["inertia/cg-x-in"] / M2IN
    cl, de = [], []
    for V_mps in (12.0, 14.0, 17.0, 20.0):
        sol = trim_custom(fdm, {"ic/vt-fps": V_mps / 0.3048},
                          {"ic/alpha-rad": (0.05, -0.2, 0.25), "ic/gamma-rad": (-0.05, -0.6, 0.2),
                           "fcs/pitch-trim-cmd-norm": (0.0, -1.0, 1.0)})
        qS = fdm["aero/qbar-psf"] * fdm["metrics/Sw-sqft"]
        cl.append(fdm["forces/fwz-aero-lbs"] / qS)
        de.append(np.degrees(fdm["fcs/elevator-pos-rad"]))
    slope = np.polyfit(cl, de, 1)[0]
    cgs.append(x_cg)
    slopes.append(slope)
    ax[0].plot(cl, de, "o-", label=f"CG {x_cg:.3f} m")
    print(f"CG at x = {x_cg:.3f} m ({(x_cg - 0.40) / CBAR:+.3f} cbar from ARP): d(de)/dCL = {slope:+.2f} deg")

fit = np.polyfit(cgs, slopes, 1)
x_np = -fit[1] / fit[0]
print(f"\nstick-fixed neutral point: x_NP = {x_np:.4f} m  (theory ~0.443 m)")
for x_cg, s in zip(cgs, slopes):
    print(f"   CG {x_cg:.3f} m -> static margin {(x_np - x_cg) / CBAR * 100:5.1f} % cbar")

ax[0].set(xlabel="CL", ylabel="trim elevator [deg]", title="trim elevator vs CL at each CG")
ax[0].legend(fontsize="small")
xx = np.linspace(min(cgs), max(x_np, max(cgs)) + 0.01, 20)
ax[1].plot(cgs, slopes, "o")
ax[1].plot(xx, np.polyval(fit, xx), "--")
ax[1].axhline(0, color="k", lw=0.8)
ax[1].axvline(x_np, color="r", lw=0.8)
ax[1].set(xlabel="CG x [m] (structural, aft +)", ylabel="d(delta_e)/dCL [deg]", title=f"neutral point {x_np:.3f} m")
for a_ in ax:
    a_.grid(alpha=0.3)
fig.tight_layout()
save(fig, "05_trim_performance", "neutral_point", show=args.show)
