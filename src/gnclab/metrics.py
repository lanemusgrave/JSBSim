"""Performance metrics for responses and loops.

These are the numbers that go on a requirements matrix or a flight-test
summary: rise time, overshoot, settling time, tracking error, and stability
margins (gain, phase and *delay* margin).
"""

from __future__ import annotations

from dataclasses import dataclass, asdict

import numpy as np


@dataclass
class StepMetrics:
    rise_time: float          # 10% -> 90% of the commanded change [s]
    overshoot_pct: float      # peak beyond the final command, % of the change
    settling_time: float      # last time outside +/- band of the command [s]
    steady_state_error: float  # mean error over the last 10% of the record
    peak: float

    def as_dict(self):
        return asdict(self)


def step_metrics(t, y, y_cmd: float, y0: float | None = None, t_step: float = 0.0,
                 band: float = 0.05) -> StepMetrics:
    """Metrics of a response ``y(t)`` to a step from ``y0`` to ``y_cmd`` at ``t_step``.

    ``band`` is the settling band as a fraction of the step size (5% default;
    use 0.02 for the 2% criterion).
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    m = t >= t_step
    t, y = t[m] - t_step, y[m]
    if y0 is None:
        y0 = y[0]
    dy = y_cmd - y0
    if abs(dy) < 1e-12:
        raise ValueError("step size is zero")
    frac = (y - y0) / dy  # 0 -> 1 normalized response
    t10 = t[np.argmax(frac >= 0.1)] if np.any(frac >= 0.1) else np.nan
    t90 = t[np.argmax(frac >= 0.9)] if np.any(frac >= 0.9) else np.nan
    rise = t90 - t10
    peak_frac = frac.max()
    overshoot = max(0.0, (peak_frac - 1.0) * 100.0)
    outside = np.abs(frac - 1.0) > band
    idx = np.nonzero(outside)[0]
    if len(idx) == 0:
        settling = 0.0
    elif idx[-1] == len(t) - 1:
        settling = np.nan  # never settled within the record
    else:
        settling = t[idx[-1] + 1]
    n_tail = max(1, len(y) // 10)
    sse = float(np.mean(y_cmd - y[-n_tail:]))
    return StepMetrics(float(rise), float(overshoot), float(settling), sse,
                       float(y0 + peak_frac * dy))


def rms(x) -> float:
    x = np.asarray(x, dtype=float)
    return float(np.sqrt(np.mean(x ** 2)))


@dataclass
class Margins:
    gain_margin_db: float
    phase_margin_deg: float
    w_gain_crossover: float    # rad/s, where |L| = 1 (phase margin measured here)
    w_phase_crossover: float   # rad/s, where angle(L) = -180 deg
    delay_margin_s: float      # extra pure delay that would destabilize the loop

    def as_dict(self):
        return asdict(self)


def loop_margins(L) -> Margins:
    """Classical margins of an open-loop transfer function ``L`` (python-control).

    Delay margin = PM [rad] / w_gc: a pure delay ``tau`` adds phase lag
    ``w*tau`` without changing gain, so the loop goes unstable when
    ``w_gc * tau`` eats the whole phase margin.  This is the number that
    tells you how much computation/transport/sensor latency you can afford.
    """
    import control

    gm, pm, wpc, wgc = control.margin(L)
    gm_db = 20 * np.log10(gm) if np.isfinite(gm) and gm > 0 else np.inf
    dm = np.deg2rad(pm) / wgc if np.isfinite(wgc) and wgc > 0 and np.isfinite(pm) else np.inf
    return Margins(float(gm_db), float(pm), float(wgc), float(wpc), float(dm))
