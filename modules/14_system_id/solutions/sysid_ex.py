"""Module 14 exercise A - solution: what makes a good system-ID input?"""

import sys
from pathlib import Path

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "07_build_uas_model" / "solutions"))
from build_gnc_trainer import P  # noqa: E402
from flight_test import maneuver  # noqa: E402

from gnclab.signals import doublet, multistep_3211, step  # noqa: E402
from gnclab.sysid import ols, smooth, smooth_derivative  # noqa: E402

PSF = 47.880259
A = 0.12
inputs = {
    "step (5 s)": lambda t: step(t, 2.0, A) - step(t, 7.0, A),
    "doublet": lambda t: doublet(t, 2.0, 0.35, A),
    "3-2-1-1": lambda t: multistep_3211(t, 2.0, 0.25, A),
}
rows = []
for name, sig in inputs.items():
    d = maneuver(f"ex_{name.split()[0]}", de_sig=sig, T=15.0)
    dt = d.index[1] - d.index[0]
    a, q, de, V = (smooth(d[c], dt) for c in ("alpha", "q", "de", "V"))
    qdot = smooth_derivative(d.q, dt)
    Cm = P["Jy"] * qdot / (d.qbar_psf.to_numpy() * PSF * P["S"] * P["c"])
    X = np.c_[np.ones_like(a), a, P["c"] / (2 * V) * q, de][25:-25]
    res = ols(X, Cm[25:-25], ["Cm0", "Cma", "Cmq", "Cmde"], truth=P)
    corr = np.corrcoef(X[:, 1], X[:, 3])[0, 1]
    rows.append({"input": name, "corr(alpha, de)": corr, "cond(X'X)": np.linalg.cond(X.T @ X),
                 **{f"{k} err %": v for k, v in res.table["error_%"].items()}})
print(pd.DataFrame(rows).round(2).to_string(index=False))
