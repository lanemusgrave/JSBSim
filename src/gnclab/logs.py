"""Flight-test log toolkit (Module 16).

Real logs are messy: several recorders, each with its own clock and rate,
jitter, gaps, and no labels saying where the test points are.  These are the
first five things you do with every log:

* :func:`resample`          put a log on a uniform time grid (interpolating numeric columns)
* :func:`estimate_offset`   find the clock offset between two recorders by cross-correlation
* :func:`steps`             find command steps (test-point starts)
* :func:`intervals`         turn a boolean condition (saturated, engaged, ...) into start/end/duration rows
* :func:`activity_windows`  find maneuvers in a long log from control-surface activity
"""

from __future__ import annotations

import numpy as np
import pandas as pd


def resample(df: pd.DataFrame, rate_hz: float, t0: float | None = None, t1: float | None = None) -> pd.DataFrame:
    """Linear interpolation of every numeric column onto a uniform grid.

    The index must be time [s] and increasing.  Never interpolate across big
    gaps without knowing they're there: check ``np.diff(df.index).max()`` first.
    """
    t = df.index.to_numpy(dtype=float)
    t0 = t[0] if t0 is None else t0
    t1 = t[-1] if t1 is None else t1
    grid = np.arange(t0, t1 + 0.5 / rate_hz, 1.0 / rate_hz)
    num = df.select_dtypes("number")
    out = {c: np.interp(grid, t, num[c].to_numpy(dtype=float)) for c in num}
    return pd.DataFrame(out, index=pd.Index(grid, name=df.index.name or "t"))


def estimate_offset(a: pd.Series, b: pd.Series, max_lag_s: float = 30.0, rate_hz: float = 100.0) -> tuple[float, float]:
    """Offset ``d`` [s] such that ``b(t + d) ~= a(t)``, i.e. add ``d`` to a's time base to get b's.

    Both series are resampled to ``rate_hz``, de-meaned and normalized; the
    peak of their cross-correlation (refined with a parabola through the
    three samples around it) gives the offset.  Returns ``(d, peak_corr)``;
    a peak below ~0.5 means the signals don't share enough features to trust it.
    Correlate on a *busy* signal (rates, surface commands), not on a slow one.
    """
    def grid(s):
        r = resample(s.to_frame("x"), rate_hz)["x"]
        x = r.to_numpy() - r.mean()
        return r.index.to_numpy(), x / (np.linalg.norm(x) or 1.0)

    ta, xa = grid(a)
    tb, xb = grid(b)
    c = np.correlate(xb, xa, mode="full")             # lag L: b[n + L] vs a[n]
    lags = np.arange(-len(xa) + 1, len(xb))
    keep = np.abs(lags / rate_hz + tb[0] - ta[0]) <= max_lag_s
    c, lags = c[keep], lags[keep]
    i = int(np.argmax(c))
    frac = 0.0
    if 0 < i < len(c) - 1:
        y0, y1, y2 = c[i - 1], c[i], c[i + 1]
        den = y0 - 2 * y1 + y2
        frac = 0.5 * (y0 - y2) / den if den != 0 else 0.0
    lag_s = (lags[i] + frac) / rate_hz
    # b sample index n+L sits at tb[0] + (n+L)/rate, a sample n at ta[0] + n/rate
    return float(tb[0] - ta[0] + lag_s), float(c[i])


def steps(cmd: pd.Series, min_jump: float) -> pd.DataFrame:
    """Rows (t, before, after, jump) for every jump of at least ``min_jump`` in a command channel."""
    v = cmd.to_numpy(dtype=float)
    d = np.diff(v)
    idx = np.nonzero(np.abs(d) >= min_jump)[0] + 1
    t = cmd.index.to_numpy()
    return pd.DataFrame({"t": t[idx], "before": v[idx - 1], "after": v[idx], "jump": d[idx - 1]})


def intervals(mask: pd.Series, min_duration: float = 0.0) -> pd.DataFrame:
    """Rows (start, end, duration) for every run of ``True`` in a boolean series."""
    m = mask.to_numpy(dtype=bool)
    t = mask.index.to_numpy(dtype=float)
    edges = np.diff(np.r_[0, m.astype(int), 0])
    starts, ends = np.nonzero(edges == 1)[0], np.nonzero(edges == -1)[0] - 1
    rows = [(t[s], t[e], t[e] - t[s]) for s, e in zip(starts, ends) if t[e] - t[s] >= min_duration]
    return pd.DataFrame(rows, columns=["start", "end", "duration"])


def activity_windows(df: pd.DataFrame, cols: list[str], window_s: float = 1.0, threshold: float | None = None,
                     merge_gap_s: float = 3.0, min_duration: float = 1.0) -> pd.DataFrame:
    """Find maneuvers: windows where any of ``cols`` is 'busy'.

    Busy = rolling standard deviation over ``window_s`` above ``threshold``
    (default: 5x the median rolling std of that column, a robust 'quiet' level).
    Windows closer than ``merge_gap_s`` are merged.  Returns start, end,
    duration and ``axis`` (the column with the most activity).
    """
    dt = float(np.median(np.diff(df.index.to_numpy())))
    n = max(3, int(round(window_s / dt)))
    busy = pd.Series(False, index=df.index)
    stds = {}
    for c in cols:
        s = df[c].rolling(n, center=True, min_periods=1).std().fillna(0.0)
        thr = threshold if threshold is not None else 5 * max(s.median(), 1e-12)
        stds[c] = s / thr
        busy |= s > thr
    win = intervals(busy)
    merged = []
    for _, w in win.iterrows():
        if merged and w.start - merged[-1][1] <= merge_gap_s:
            merged[-1][1] = w.end
        else:
            merged.append([w.start, w.end])
    rows = []
    for s, e in merged:
        if e - s < min_duration:
            continue
        seg = slice(s, e)
        axis = max(cols, key=lambda c: stds[c].loc[seg].mean())
        rows.append((s, e, e - s, axis))
    return pd.DataFrame(rows, columns=["start", "end", "duration", "axis"])
