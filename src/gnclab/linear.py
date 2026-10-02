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


# ---------------------------------------------------------------------------
# Finite-difference linearization (works for any vehicle, including gliders)
# ---------------------------------------------------------------------------
FD_STATES = ("Vt", "Alpha", "Theta", "Q", "Beta", "Phi", "P", "R")
_IC = {"Vt": "ic/vt-fps", "Alpha": "ic/alpha-rad", "Theta": "ic/theta-rad", "Q": "ic/q-rad_sec",
       "Beta": "ic/beta-rad", "Phi": "ic/phi-rad", "P": "ic/p-rad_sec", "R": "ic/r-rad_sec"}


def _state_derivative(fdm) -> np.ndarray:
    """xdot for FD_STATES, from the accelerations JSBSim computes in run_ic()."""
    u, v, w = fdm["velocities/u-aero-fps"], fdm["velocities/v-aero-fps"], fdm["velocities/w-aero-fps"]
    ud, vd, wd = (fdm["accelerations/udot-ft_sec2"], fdm["accelerations/vdot-ft_sec2"],
                  fdm["accelerations/wdot-ft_sec2"])
    V = np.sqrt(u * u + v * v + w * w)
    Vd = (u * ud + v * vd + w * wd) / V
    alpha_d = (u * wd - w * ud) / (u * u + w * w)
    beta_d = (V * vd - v * Vd) / (V * np.sqrt(u * u + w * w))
    return np.array([Vd, alpha_d, fdm["velocities/thetadot-rad_sec"], fdm["accelerations/qdot-rad_sec2"],
                     beta_d, fdm["velocities/phidot-rad_sec"], fdm["accelerations/pdot-rad_sec2"],
                     fdm["accelerations/rdot-rad_sec2"]])


def linearize_fd(fdm: jsbsim.FGFDMExec, inputs: Sequence[str],
                 x_steps: Sequence[float] = (0.5, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3),
                 u_step: float = 1e-3) -> LinearModel:
    """Central-difference linearization about the CURRENT (trimmed) state.

    States: ``Vt [ft/s], Alpha, Theta [rad], Q [rad/s], Beta, Phi [rad], P, R [rad/s]``.
    ``inputs`` are property names written before ``run_ic()``, e.g.
    ``["fcs/elevator-cmd-norm", "fcs/aileron-cmd-norm", "fcs/rudder-cmd-norm"]``.

    Each perturbation re-initializes the model with ``run_ic()`` (one pass of
    every model, no integration) and reads the accelerations.  Assumption: the
    FCS is *static* between command and surface (gains, sums, limits).  An
    actuator with a lag will not respond within one pass - perturb its output
    property instead, or use ``jsbsim.FGLinearization`` (needs an engine).

    The model is left re-initialized at the original trim point.
    """
    # read the *current* state (not possibly stale ic/ values)
    x0 = {"Vt": fdm["velocities/vt-fps"], "Alpha": fdm["aero/alpha-rad"], "Theta": fdm["attitude/theta-rad"],
          "Q": fdm["velocities/q-rad_sec"], "Beta": fdm["aero/beta-rad"], "Phi": fdm["attitude/phi-rad"],
          "P": fdm["velocities/p-rad_sec"], "R": fdm["velocities/r-rad_sec"]}
    u0 = {p: fdm[p] for p in inputs}
    keep = {k: fdm[k] for k in ("ic/h-sl-ft", "ic/psi-true-rad", "ic/lat-geod-rad", "ic/long-gc-rad")}

    def f(x: dict, u: dict) -> np.ndarray:
        for k, val in keep.items():
            fdm[k] = val
        for s in FD_STATES:
            fdm[_IC[s]] = x[s]
        for p, val in u.items():
            fdm[p] = val
        fdm.run_ic()
        return _state_derivative(fdm)

    n, m = len(FD_STATES), len(inputs)
    A, B = np.zeros((n, n)), np.zeros((n, m))
    for j, s in enumerate(FD_STATES):
        h = x_steps[j]
        xp, xm = dict(x0), dict(x0)
        xp[s] += h
        xm[s] -= h
        A[:, j] = (f(xp, u0) - f(xm, u0)) / (2 * h)
    for j, p in enumerate(inputs):
        up, um = dict(u0), dict(u0)
        up[p] += u_step
        um[p] -= u_step
        B[:, j] = (f(x0, up) - f(x0, um)) / (2 * u_step)
    f(x0, u0)  # restore
    return LinearModel(A, B, np.eye(n), np.zeros((n, m)), FD_STATES, tuple(inputs), FD_STATES,
                       ("ft/s", "rad", "rad", "rad/s", "rad", "rad", "rad/s", "rad/s"), tuple("norm" for _ in inputs))
