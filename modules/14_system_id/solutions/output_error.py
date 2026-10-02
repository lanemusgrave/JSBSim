"""Module 14 - output-error identification of the short period.

Equation error differentiates noisy q and regresses on noisy alpha: biased.
Output error instead SIMULATES a model and adjusts its parameters until the
simulated alpha(t), q(t) match the measured ones (no differentiation, noise
only in the outputs).  Model (short period with measured V, theta, delta_e):
    alpha_dot = q - qbar S CL / (m V) + g cos(theta - alpha) / V
    q_dot     = qbar S c Cm / Iyy
    CL = CL0 + CLa alpha,  Cm = Cm0 + Cma alpha + Cmq (c/2V) q + Cmde de
Start from the equation-error answers; fit on the 3-2-1-1; validate on the doublet.

    python modules/14_system_id/solutions/output_error.py [--show]
"""

import sys
from pathlib import Path

from gnclab.cli import parse_args

args = parse_args(__doc__)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402

from gnclab import OUTPUT_DIR  # noqa: E402
from gnclab.plotting import save  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
from flight_test import ensure_logs  # noqa: E402

ensure_logs()
from gnclab.sysid import theil  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "07_build_uas_model" / "solutions"))
from build_gnc_trainer import P  # noqa: E402

LOGS = OUTPUT_DIR / "14_system_id"
PSF, G = 47.880259, 9.80665
NAMES = ["CL0", "CLa", "Cm0", "Cma", "Cmq", "Cmde"]
START = np.array([0.224, 5.35, 0.0177, -2.64, -31.1, -0.92])   # equation-error results


def load(name):
    d = pd.read_csv(LOGS / f"{name}.csv", index_col="t")
    return d.iloc[100:1200]       # 1 s before to ~10 s after the input


def simulate(theta_p, d, x0):
    CL0, CLa, Cm0, Cma, Cmq, Cmde = theta_p
    t = d.index.to_numpy()
    dt = t[1] - t[0]
    qbar = d.qbar_psf.to_numpy() * PSF
    V, th, de = d.V.to_numpy(), d.theta.to_numpy(), d.de.to_numpy()
    a, q = x0
    out = np.empty((len(t), 2))
    for k in range(len(t)):
        out[k] = a, q
        qS = qbar[k] * P["S"]
        CL = CL0 + CLa * a
        Cm = Cm0 + Cma * a + Cmq * P["c"] / (2 * V[k]) * q + Cmde * de[k]
        ad = q - qS * CL / (P["mass"] * V[k]) + G * np.cos(th[k] - a) / V[k]
        qd = qS * P["c"] * Cm / P["Jy"]
        a, q = a + dt * ad, q + dt * qd
    return out


def residuals(theta_p, d):
    x0 = (d.alpha.iloc[:20].mean(), d.q.iloc[:20].mean())
    sim = simulate(theta_p, d, x0)
    return np.r_[(sim[:, 0] - d.alpha.to_numpy()) / 0.0035, (sim[:, 1] - d.q.to_numpy()) / 0.003]


fit, val = load("long_3211"), load("long_doublet")
sol = least_squares(residuals, START, args=(fit,), x_scale=np.abs(START))
J = sol.jac
cov = np.linalg.inv(J.T @ J) * np.mean(sol.fun ** 2)
table = pd.DataFrame({"equation error": START, "output error": sol.x, "std_err": np.sqrt(np.diag(cov)),
                      "truth": [P[k] for k in NAMES]}, index=NAMES)
table["EE error %"] = 100 * (table["equation error"] - table.truth) / table.truth.abs()
table["OE error %"] = 100 * (table["output error"] - table.truth) / table.truth.abs()
print(table.round(4).to_string())

fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
x0 = (val.alpha.iloc[:20].mean(), val.q.iloc[:20].mean())
for label, th in (("equation-error model", START), ("output-error model", sol.x)):
    sim = simulate(th, val, x0)
    print(f"validation (held-out doublet) Theil U, {label:21s}: alpha {theil(val.alpha, sim[:, 0]):.3f}, "
          f"q {theil(val.q, sim[:, 1]):.3f}")
    ax[0].plot(val.index, np.degrees(sim[:, 0]), label=label)
    ax[1].plot(val.index, np.degrees(sim[:, 1]), label=label)
ax[0].plot(val.index, np.degrees(val.alpha), "k", lw=0.5, alpha=0.6, label="measured")
ax[1].plot(val.index, np.degrees(val.q), "k", lw=0.5, alpha=0.6)
ax[0].set_ylabel("alpha [deg]")
ax[1].set_ylabel("q [deg/s]")
ax[1].set_xlabel("time [s]")
ax[0].legend(fontsize="small")
ax[0].set_title("Held-out doublet: simulated vs measured")
for a_ in ax:
    a_.grid(alpha=0.3)
fig.tight_layout()
save(fig, "14_system_id", "output_error_validation", show=args.show)
