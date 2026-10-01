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
