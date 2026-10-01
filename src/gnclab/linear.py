"""Linearize a trimmed JSBSim model and analyze its dynamic modes.

``jsbsim.FGLinearization(fdm)`` numerically perturbs every state and input
around the *current* (trimmed!) flight condition and returns

    xdot = A x + B u
    y    = C x + D u

with states ``Vt, Alpha, Theta, Q, Rpm0, Beta, Phi, P, Psi, R, Latitude,
Longitude, Alt`` (units: ft/s, rad, rad, rad/s, rpm, rad, rad, rad/s, rad,
rad/s, rad, rad, ft) and inputs ``ThtlCmd, DaCmd, DeCmd, DrCmd`` (normalized
-1..1 commands, throttle 0..1).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import jsbsim
import numpy as np
import pandas as pd

LONG_STATES = ("Vt", "Alpha", "Theta", "Q")
LAT_STATES = ("Beta", "Phi", "P", "R")


@dataclass
class LinearModel:
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    D: np.ndarray
    x_names: tuple[str, ...]
    u_names: tuple[str, ...]
    y_names: tuple[str, ...]
    x_units: tuple[str, ...] = ()
    u_units: tuple[str, ...] = ()

    def subsystem(self, states: Sequence[str], inputs: Sequence[str]) -> "LinearModel":
        """Extract a decoupled subsystem, e.g. longitudinal or lateral.

        Outputs are taken equal to the selected states (C = I, D = 0).
        """
        xi = [self.x_names.index(s) for s in states]
        ui = [self.u_names.index(u) for u in inputs]
        A = self.A[np.ix_(xi, xi)]
        B = self.B[np.ix_(xi, ui)]
        n = len(xi)
        xu = tuple(self.x_units[i] for i in xi) if self.x_units else ()
        uu = tuple(self.u_units[i] for i in ui) if self.u_units else ()
        return LinearModel(A, B, np.eye(n), np.zeros((n, len(ui))), tuple(states),
                           tuple(inputs), tuple(states), xu, uu)

    def to_control(self):
        """Return a ``control.StateSpace`` object (python-control)."""
        import control

        return control.ss(self.A, self.B, self.C, self.D,
                          inputs=list(self.u_names), outputs=list(self.y_names),
                          states=list(self.x_names))

    def as_frame(self, which: str = "A") -> pd.DataFrame:
        """Pretty-print a matrix with row/column labels."""
        if which == "A":
            return pd.DataFrame(self.A, index=self.x_names, columns=self.x_names)
        if which == "B":
            return pd.DataFrame(self.B, index=self.x_names, columns=self.u_names)
        raise ValueError(which)


def linearize(fdm: jsbsim.FGFDMExec) -> LinearModel:
    """Linearize about the current state (trim first!)."""
    lin = jsbsim.FGLinearization(fdm)
    return LinearModel(
        np.array(lin.system_matrix), np.array(lin.input_matrix),
        np.array(lin.output_matrix), np.array(lin.feedforward_matrix),
        tuple(lin.x_names), tuple(lin.u_names), tuple(lin.y_names),
        tuple(lin.x_units), tuple(lin.u_units),
    )


def modes(A: np.ndarray, x_names: Sequence[str] | None = None,
          min_abs: float = 1e-6) -> pd.DataFrame:
    """Eigen-analysis of ``A``.

    Returns one row per mode (complex pairs collapsed) with natural frequency
    ``wn`` [rad/s], damping ratio ``zeta``, damped ``period`` [s], time to
    half amplitude (time to *double* if ``stable`` is False), and the state
    with the largest eigenvector component.  Eigenvector components have mixed
    units, so treat ``dominant_state`` as a hint, not a proof.
    """
    eigvals, eigvecs = np.linalg.eig(A)
    rows = []
    for k, lam in enumerate(eigvals):
        if abs(lam) < min_abs or lam.imag < -1e-12:
            continue  # skip integrator-like zeros and the conjugate twin
        wn = abs(lam)
        zeta = -lam.real / wn
        period = 2 * np.pi / lam.imag if lam.imag > 1e-12 else np.inf
        t_half = np.log(2) / abs(lam.real) if abs(lam.real) > 1e-12 else np.inf
        dominant = ""
        if x_names is not None:
            v = np.abs(eigvecs[:, k])
            dominant = x_names[int(np.argmax(v))]
        rows.append({
            "eig_real": lam.real, "eig_imag": lam.imag, "wn": wn, "zeta": zeta,
            "period_s": period, "stable": bool(lam.real < 0),
            "t_half_or_double_s": t_half,
            "dominant_state": dominant,
        })
    df = pd.DataFrame(rows)
    return df.sort_values("wn", ascending=False).reset_index(drop=True)
