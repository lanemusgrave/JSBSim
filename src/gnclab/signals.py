"""Test-input signals used for flight test and system identification.

All functions take time ``t`` (scalar or numpy array) and return the signal
value(s).  Times are in seconds, amplitudes in whatever unit you are driving.
"""

from __future__ import annotations

import numpy as np


def step(t, t0: float = 1.0, amp: float = 1.0):
    """0 before ``t0``, ``amp`` after."""
    return np.where(np.asarray(t) >= t0, amp, 0.0) + 0.0


def pulse(t, t0: float, width: float, amp: float = 1.0):
    t = np.asarray(t)
    return np.where((t >= t0) & (t < t0 + width), amp, 0.0) + 0.0


def doublet(t, t0: float = 1.0, width: float = 1.0, amp: float = 1.0):
    """+amp for ``width`` s, then -amp for ``width`` s.

    Pick ``width`` near 1/(2 f) for the mode you want to excite: e.g. ~0.5-1 s
    for a light-aircraft short period.
    """
    return pulse(t, t0, width, amp) - pulse(t, t0 + width, width, amp)


def multistep_3211(t, t0: float = 1.0, dt_unit: float = 0.3, amp: float = 1.0):
    """The classic 3-2-1-1 multistep: +3u, -2u, +1u, -1u (u = ``dt_unit``).

    Richer in frequency content than a doublet; a workhorse of flight-test
    system identification (Morelli & Klein, ch. 9).
    """
    s = pulse(t, t0, 3 * dt_unit, amp)
    s = s - pulse(t, t0 + 3 * dt_unit, 2 * dt_unit, amp)
    s = s + pulse(t, t0 + 5 * dt_unit, dt_unit, amp)
    s = s - pulse(t, t0 + 6 * dt_unit, dt_unit, amp)
    return s


def chirp(t, t0: float, duration: float, f0: float, f1: float, amp: float = 1.0,
          log: bool = True, taper: float = 0.0):
    """Frequency sweep from ``f0`` to ``f1`` Hz over ``duration`` seconds.

    ``log=True`` gives an exponential sweep (equal time per decade) which is
    the usual choice for frequency-domain identification.  ``taper`` (s)
    applies a raised-cosine fade-in/out to avoid a step at the start/end.
    """
    t = np.asarray(t, dtype=float)
    tau = np.clip(t - t0, 0.0, duration)
    if log:
        k = (f1 / f0) ** (1.0 / duration)
        phase = 2 * np.pi * f0 * (k ** tau - 1.0) / np.log(k)
    else:
        phase = 2 * np.pi * (f0 * tau + 0.5 * (f1 - f0) / duration * tau ** 2)
    y = amp * np.sin(phase)
    if taper > 0:
        w = np.ones_like(tau)
        a = tau < taper
        w[a] = 0.5 * (1 - np.cos(np.pi * tau[a] / taper))
        b = tau > duration - taper
        w[b] = 0.5 * (1 - np.cos(np.pi * (duration - tau[b]) / taper))
        y = y * w
    active = (t >= t0) & (t <= t0 + duration)
    return np.where(active, y, 0.0)
