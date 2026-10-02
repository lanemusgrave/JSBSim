"""Module 14 - equation-error least squares: recover the aero derivatives.

From the flight-test logs (run flight_test.py first):
  measured coefficient  = inertia * smoothed angular acceleration / (qbar S ref)
  regressors            = 1, alpha, (c/2V) q, delta_e        (pitch)   ... etc.
Solve by OLS on the 3-2-1-1 maneuvers, then PREDICT the held-out doublet
maneuvers with the identified model (the only honest test of a model).

Truth (what JSBSim was built from) = Beard & McLain parameters in
modules/07_build_uas_model/solutions/build_gnc_trainer.py.

    python modules/14_system_id/solutions/equation_error.py [--show]
"""

import sys
from pathlib import Path

from gnclab.cli import parse_args

args = parse_args(__doc__)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from gnclab import OUTPUT_DIR  # noqa: E402
from gnclab.plotting import save  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
from flight_test import ensure_logs  # noqa: E402

ensure_logs()
from gnclab.sysid import ols, smooth, smooth_derivative, theil  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "07_build_uas_model" / "solutions"))
from build_gnc_trainer import P  # noqa: E402

LOGS = OUTPUT_DIR / "14_system_id"
PSF = 47.880259
S, b, c, m = P["S"], P["b"], P["c"], P["mass"]
Jx, Jy, Jz, Jxz = P["Jx"], P["Jy"], P["Jz"], P["Jxz"]


def prep(name):
    d = pd.read_csv(LOGS / f"{name}.csv", index_col="t")
    dt = d.index[1] - d.index[0]
    for col in ("alpha", "beta", "p", "q", "r", "V", "ax", "ay", "az", "de", "da", "dr"):
        d[col + "_s"] = smooth(d[col], dt)
    for col in ("p", "q", "r"):
        d[col + "dot"] = smooth_derivative(d[col], dt)
    qS = d.qbar_psf.to_numpy() * PSF * S
    d["Cm"] = Jy * d.qdot / (qS * c)
    d["CL"] = -m * d.az_s * np.cos(d.alpha_s) / qS          # thrust is along x, so az has no thrust term
    d["CY"] = m * d.ay_s / qS
    d["Cl"] = (Jx * d.pdot - Jxz * d.rdot) / (qS * b)
    d["Cn"] = (Jz * d.rdot - Jxz * d.pdot) / (qS * b)
    d["qn"] = c / (2 * d.V_s) * d.q_s
    d["pn"] = b / (2 * d.V_s) * d.p_s
    d["rn"] = b / (2 * d.V_s) * d.r_s
    d["one"] = 1.0
    return d.iloc[25:-25]                                    # drop filter edge effects


MODELS = {
    "Cm": (["one", "alpha_s", "qn", "de_s"], ["Cm0", "Cma", "Cmq", "Cmde"], "long"),
    "CL": (["one", "alpha_s", "qn", "de_s"], ["CL0", "CLa", "CLq", "CLde"], "long"),
    "CY": (["beta_s", "da_s", "dr_s"], ["CYb", "CYda", "CYdr"], "lat"),
    "Cl": (["one", "beta_s", "pn", "rn", "da_s", "dr_s"], ["Cl_0 (prop torque)", "Clb", "Clp", "Clr", "Clda", "Cldr"], "lat"),
    "Cn": (["one", "beta_s", "pn", "rn", "da_s", "dr_s"], ["Cn_0", "Cnb", "Cnp", "Cnr", "Cnda", "Cndr"], "lat"),
}
fit = {"long": prep("long_3211"), "lat": prep("lat_3211")}
val = {"long": prep("long_doublet"), "lat": prep("lat_doublet")}

fig, axes = plt.subplots(len(MODELS), 1, figsize=(10, 2.3 * len(MODELS)), sharex=False)
summary = []
for ax, (y, (cols, names, kind)) in zip(axes, MODELS.items()):
    res = ols(fit[kind][cols].to_numpy(), fit[kind][y].to_numpy(), names, truth=P)
    print(f"\n{y}: R^2 = {res.r2:.3f} on the 3-2-1-1")
    print(res.table.round(4).to_string())
    pred = res.predict(val[kind][cols].to_numpy())
    tic = theil(val[kind][y], pred)
    summary.append((y, res.r2, tic))
    ax.plot(val[kind].index, val[kind][y], lw=0.8, label="measured (held-out doublet)")
    ax.plot(val[kind].index, pred, lw=1.2, label="identified model prediction")
    ax.set_ylabel(y)
    ax.set_title(f"{y}: validation Theil U = {tic:.2f}", fontsize="small")
    ax.grid(alpha=0.3)
axes[0].legend(fontsize="small")
axes[-1].set_xlabel("time [s]")
fig.tight_layout()
save(fig, "14_system_id", "equation_error_validation", show=args.show)
print("\nfit R^2 / validation Theil U (0 = perfect, < 0.3 good):")
for y, r2, tic in summary:
    print(f"   {y}: R^2 {r2:.3f}, Theil U {tic:.3f}")
