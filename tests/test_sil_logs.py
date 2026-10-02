"""SIL harness and log toolkit (Module 16)."""

import numpy as np
import pandas as pd
import pytest

from gnclab.logs import activity_windows, estimate_offset, intervals, resample, steps
from gnclab.sil import SILConfig, in_process_reference, pack, run_sil, unpack


def test_packets_round_trip():
    kind, f = unpack(pack("SENS", 7, 0.25, 0.1, -0.02, 330.0, 82.0, 430.0, 82.0))
    assert kind == "SENS" and f[0] == 7 and f[4] == 330.0


def test_lockstep_sil_matches_in_process():
    """The harness (processes, sockets, packing) must add nothing."""
    cfg = SILConfig(duration=6.0)
    ref, sil = in_process_reference(cfg), run_sil(cfg)
    assert np.array_equal(ref.alt_ft.to_numpy(), sil.alt_ft.to_numpy())
    assert (sil.age_frames == 0).all()


def test_lockstep_delay_is_applied():
    s = run_sil(SILConfig(duration=2.0, delay_frames=3))
    assert (s.age_frames.iloc[5:] == 3).all()


def test_estimate_offset_recovers_a_shift():
    rng = np.random.default_rng(1)
    t = np.arange(0, 40, 0.02)
    x = np.convolve(rng.normal(size=t.size), np.ones(15) / 15, "same")
    a = pd.Series(x, index=t)
    b = pd.Series(x, index=t + 57.3)                 # same signal, clock 57.3 s ahead
    d, peak = estimate_offset(a, b, max_lag_s=100)
    assert d == pytest.approx(57.3, abs=0.01) and peak > 0.95


def test_intervals_steps_resample():
    t = np.arange(0, 10, 0.1)
    m = pd.Series((t > 2) & (t < 4), index=t)
    iv = intervals(m)
    assert len(iv) == 1 and iv.start[0] == pytest.approx(2.1) and iv.end[0] == pytest.approx(3.9)
    st = steps(pd.Series(np.where(t >= 5, 10.0, 0.0), index=t), 1.0)
    assert len(st) == 1 and st.t[0] == pytest.approx(5.0)
    r = resample(pd.DataFrame({"x": t}, index=t), 20.0)
    assert np.allclose(r.x.to_numpy(), r.index.to_numpy())


def test_activity_windows_finds_bursts():
    rng = np.random.default_rng(0)
    t = np.arange(0, 60, 0.01)
    de = rng.normal(0, 0.001, t.size)
    de[(t > 10) & (t < 13)] += 0.05 * np.sign(np.sin(2 * np.pi * t[(t > 10) & (t < 13)]))
    w = activity_windows(pd.DataFrame({"de": de, "da": rng.normal(0, 0.001, t.size)}, index=t), ["de", "da"])
    assert len(w) == 1 and 9 < w.start[0] < 11 and 12 < w.end[0] < 14 and w.axis[0] == "de"
