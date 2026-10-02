"""System-identification tools (Klein & Morelli, "Aircraft System Identification").

* :func:`smooth_derivative` - Savitzky-Golay derivative of a noisy signal
* :func:`measured_coefficients` - flight-log -> measured aero coefficients + regressors
* :func:`ols`               - equation-error ordinary least squares with standard errors
* :func:`frf`               - frequency response + coherence from input/output records
* :func:`theil`             - Theil inequality coefficient (fit quality, 0 = perfect)
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd
from scipy import signal


def smooth_derivative(x, dt: float, window_s: float = 0.25, order: int = 3) -> np.ndarray:
    """d/dt of a noisy, uniformly sampled signal (local polynomial fit)."""
    n = max(order + 2, int(round(window_s / dt)) | 1)
    return signal.savgol_filter(np.asarray(x, float), n, order, deriv=1, delta=dt)


def smooth(x, dt: float, window_s: float = 0.25, order: int = 3) -> np.ndarray:
    n = max(order + 2, int(round(window_s / dt)) | 1)
    return signal.savgol_filter(np.asarray(x, float), n, order)


PSF_TO_PA = 47.880259

# coefficient: (regressor columns, parameter names, maneuver kind) for equation error
EQUATION_ERROR_MODELS = {
    "Cm": (["one", "alpha_s", "qn", "de_s"], ["Cm0", "Cma", "Cmq", "Cmde"], "long"),
    "CL": (["one", "alpha_s", "qn", "de_s"], ["CL0", "CLa", "CLq", "CLde"], "long"),
    "CY": (["beta_s", "da_s", "dr_s"], ["CYb", "CYda", "CYdr"], "lat"),
    "Cl": (["one", "beta_s", "pn", "rn", "da_s", "dr_s"], ["Cl_0 (prop torque)", "Clb", "Clp", "Clr", "Clda", "Cldr"], "lat"),
    "Cn": (["one", "beta_s", "pn", "rn", "da_s", "dr_s"], ["Cn_0", "Cnb", "Cnp", "Cnr", "Cnda", "Cndr"], "lat"),
}


def measured_coefficients(d: pd.DataFrame, g: dict, trim: int = 25) -> pd.DataFrame:
    """Measured aero coefficients and normalized regressors from a flight log.

    ``d``: uniformly sampled log (index t [s]) with alpha, beta, p, q, r, V [SI],
    ax, ay, az [m/s^2 specific force], de, da, dr [rad], qbar_psf.
    ``g``: S, b, c, mass, Jx, Jy, Jz, Jxz (SI), i.e. what you KNOW about the airplane
    (from weighing and swinging it), not what you're identifying.
    Adds ``<col>_s`` (smoothed), ``pdot/qdot/rdot``, ``Cm, CL, CY, Cl, Cn``, ``pn/qn/rn``
    and ``one``; drops ``trim`` samples at each end (filter edge effects).
    """
    d = d.copy()
    dt = d.index[1] - d.index[0]
    for col in ("alpha", "beta", "p", "q", "r", "V", "ax", "ay", "az", "de", "da", "dr"):
        d[col + "_s"] = smooth(d[col], dt)
    for col in ("p", "q", "r"):
        d[col + "dot"] = smooth_derivative(d[col], dt)
    S, b, c, m = g["S"], g["b"], g["c"], g["mass"]
    qS = d.qbar_psf.to_numpy() * PSF_TO_PA * S
    d["Cm"] = g["Jy"] * d.qdot / (qS * c)
    d["CL"] = -m * d.az_s * np.cos(d.alpha_s) / qS          # thrust is along x, so az has no thrust term
    d["CY"] = m * d.ay_s / qS
    d["Cl"] = (g["Jx"] * d.pdot - g["Jxz"] * d.rdot) / (qS * b)
    d["Cn"] = (g["Jz"] * d.rdot - g["Jxz"] * d.pdot) / (qS * b)
    d["qn"] = c / (2 * d.V_s) * d.q_s
    d["pn"] = b / (2 * d.V_s) * d.p_s
    d["rn"] = b / (2 * d.V_s) * d.r_s
    d["one"] = 1.0
    return d.iloc[trim:-trim]


@dataclass
class OLSResult:
    table: pd.DataFrame        # estimate, std error, (optional) truth
    r2: float
    residual_std: float

    def predict(self, X: np.ndarray) -> np.ndarray:
        return X @ self.table["estimate"].to_numpy()


def ols(X: np.ndarray, y: np.ndarray, names, truth: dict | None = None) -> OLSResult:
    """Solve y = X theta in the least-squares sense.

    Standard errors use s^2 (X'X)^-1, which UNDER-estimates the true error when
    residuals are coloured (they nearly always are in flight data) - Klein &
    Morelli sec. 5.2 give a correction; here they are an optimistic lower bound.
    """
    X, y = np.asarray(X, float), np.asarray(y, float)
    theta, *_ = np.linalg.lstsq(X, y, rcond=None)
    res = y - X @ theta
    n, p = X.shape
    s2 = res @ res / max(n - p, 1)
    cov = s2 * np.linalg.inv(X.T @ X)
    table = pd.DataFrame({"estimate": theta, "std_err": np.sqrt(np.diag(cov))}, index=list(names))
    if truth:
        table["truth"] = [truth.get(k, np.nan) for k in names]
        table["error_%"] = 100 * (table.estimate - table.truth) / table.truth.abs()
    r2 = 1 - res.var() / y.var()
    return OLSResult(table, float(r2), float(np.sqrt(s2)))


def frf(u, y, fs: float, nperseg: int):
    """H(f) = Puy / Puu (H1 estimator) and coherence gamma^2(f)."""
    f, Puu = signal.welch(u, fs=fs, nperseg=nperseg)
    _, Puy = signal.csd(u, y, fs=fs, nperseg=nperseg)
    _, coh = signal.coherence(u, y, fs=fs, nperseg=nperseg)
    return f, Puy / Puu, coh


def theil(y_meas, y_model) -> float:
    """Theil inequality coefficient: 0 perfect, < 0.3 is usually 'good' for flight data."""
    y_meas, y_model = np.asarray(y_meas), np.asarray(y_model)
    num = np.sqrt(np.mean((y_meas - y_model) ** 2))
    return float(num / (np.sqrt(np.mean(y_meas ** 2)) + np.sqrt(np.mean(y_model ** 2))))
