"""Trimming: find controls + attitude that zero the accelerations.

JSBSim's built-in trim (``fdm.do_trim(mode)``) is a Newton-like search that
adjusts controls (throttle, elevator, aileron, rudder) and states (alpha,
theta, phi...) one axis at a time.  Modes:

====  ============  ==========================================================
mode  name          what is trimmed
====  ============  ==========================================================
0     longitudinal  udot, wdot, qdot with throttle, elevator (pitch) and alpha
1     full          all six accelerations (adds phi/beta/aileron/rudder)
2     ground        aircraft sitting on its gear (pitch + altitude)
3     pullup        steady pull-up at ``ic/targetNlf``
4     custom        user-defined axes (not exposed here)
5     turn          steady coordinated turn at ``ic/targetNlf``
====  ============  ==========================================================

The trim target is defined by the ``ic/...`` properties set *before* calling
``run_ic()`` (airspeed, altitude, flight-path angle ``ic/gamma-deg``...).
"""

from __future__ import annotations

import jsbsim

MODES = {"longitudinal": 0, "full": 1, "ground": 2, "pullup": 3, "turn": 5}

SUMMARY = {
    "V_kts": "velocities/vtrue-kts",
    "alt_ft": "position/h-sl-ft",
    "alpha_deg": "aero/alpha-deg",
    "theta_deg": "attitude/theta-deg",
    "phi_deg": "attitude/phi-deg",
    "beta_deg": "aero/beta-deg",
    "gamma_deg": "flight-path/gamma-deg",
    "elevator_cmd": "fcs/elevator-cmd-norm",
    "pitch_trim_cmd": "fcs/pitch-trim-cmd-norm",
    "elevator_rad": "fcs/elevator-pos-rad",
    "aileron_cmd": "fcs/aileron-cmd-norm",
    "rudder_cmd": "fcs/rudder-cmd-norm",
    "throttle": "fcs/throttle-cmd-norm[0]",
    "nz_g": "accelerations/Nz",
}


class TrimError(RuntimeError):
    """Raised when JSBSim's trim routine does not converge."""


def trim(fdm: jsbsim.FGFDMExec, mode: str | int = "full") -> dict[str, float]:
    """Trim the aircraft and return a summary of the trimmed state.

    Raises :class:`TrimError` with a helpful message if the trim fails
    (typically: target speed below stall / above max thrust, or bad ICs).
    """
    m = MODES[mode] if isinstance(mode, str) else int(mode)
    try:
        fdm.do_trim(m)
    except jsbsim.TrimFailureError as exc:  # pragma: no cover - depends on ICs
        raise TrimError(
            f"Trim (mode {m}) failed: {exc}. Check that the requested speed/"
            "altitude/flight path angle is achievable with the available thrust "
            "and control power."
        ) from exc
    return trim_summary(fdm)


def trim_summary(fdm: jsbsim.FGFDMExec) -> dict[str, float]:
    """Read the properties in :data:`SUMMARY` (skips any the model lacks)."""
    out = {}
    for key, prop in SUMMARY.items():
        try:
            out[key] = fdm[prop]
        except (KeyError, jsbsim.BaseError):  # pragma: no cover
            continue
    return out


def trim_custom(
    fdm: jsbsim.FGFDMExec,
    fixed: dict[str, float],
    unknowns: dict[str, tuple[float, float, float]],
    residuals: tuple[str, ...] = ("accelerations/udot-ft_sec2",
                                  "accelerations/wdot-ft_sec2",
                                  "accelerations/qdot-rad_sec2"),
    weights: tuple[float, ...] | None = None,
) -> dict[str, float]:
    """Solve for an equilibrium with scipy instead of JSBSim's trim routine.

    ``fixed``    : property -> value written before every evaluation
                   (e.g. ``{"ic/vt-fps": 48, "ic/h-sl-ft": 3000}``).
    ``unknowns`` : property -> (initial guess, lower bound, upper bound), e.g.
                   ``{"ic/alpha-rad": (0.05, -0.2, 0.3), "ic/gamma-rad": (-0.05, -0.5, 0.2),
                   "fcs/pitch-trim-cmd-norm": (0, -1, 1)}``.
    ``residuals``: properties driven to zero (default: u-dot, w-dot, q-dot).

    Each evaluation writes the values, calls ``run_ic()`` (which runs every
    model once with integration suspended) and reads the accelerations.
    Works for vehicles JSBSim's trim cannot handle (gliders, odd controls).
    Body rates are zeroed, so this is for wings-level, non-turning equilibria.
    Returns the solution plus ``cost`` (sum of squared weighted residuals).
    """
    from scipy.optimize import least_squares

    names = list(unknowns)
    x0 = [unknowns[n][0] for n in names]
    lo = [unknowns[n][1] for n in names]
    hi = [unknowns[n][2] for n in names]
    w = weights or tuple(10.0 if "rad_sec2" in r else 1.0 for r in residuals)

    def f(x):
        for prop, value in fixed.items():
            fdm[prop] = value
        for prop in ("ic/p-rad_sec", "ic/q-rad_sec", "ic/r-rad_sec"):
            fdm[prop] = 0.0
        for prop, value in zip(names, x):
            fdm[prop] = value
        fdm.run_ic()
        return [fdm[r] * wi for r, wi in zip(residuals, w)]

    sol = least_squares(f, x0, bounds=(lo, hi), xtol=1e-12, ftol=1e-12)
    f(sol.x)  # leave the fdm at the solution
    out = dict(zip(names, map(float, sol.x)))
    out["cost"] = float(2 * sol.cost)
    if out["cost"] > 1e-6:
        raise TrimError(f"custom trim did not converge (cost {out['cost']:.3g}); "
                        "check bounds/initial guess or whether the point is achievable")
    return out
