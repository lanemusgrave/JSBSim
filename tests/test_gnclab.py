"""Unit tests for the gnclab helpers (fast; run on every push)."""

import numpy as np
import pytest

from gnclab import make_fdm, initialize, run
from gnclab.linear import linearize, modes, LONG_STATES
from gnclab.metrics import step_metrics, rms, loop_margins
from gnclab.signals import doublet, multistep_3211, chirp, step
from gnclab.trim import trim


@pytest.fixture(scope="module")
def c172_trimmed():
    fdm = make_fdm("c172x")
    initialize(fdm, {"h-sl-ft": 5000, "vc-kts": 100, "gamma-deg": 0})
    summary = trim(fdm, "full")
    return fdm, summary


def test_trim_c172_cruise(c172_trimmed):
    _, s = c172_trimmed
    assert abs(s["gamma_deg"]) < 0.1
    assert 0.3 < s["throttle"] < 1.0
    assert -2 < s["alpha_deg"] < 6
    assert abs(s["nz_g"] - 1.0) < 0.02


def test_run_returns_dataframe():
    fdm = make_fdm("c172x")
    initialize(fdm, {"h-sl-ft": 3000, "vc-kts": 90})
    df = run(fdm, 1.0, ["velocities/vc-kts", "position/h-sl-ft"])
    assert df.index[0] == 0.0
    assert df.index[-1] == pytest.approx(1.0, abs=fdm.get_delta_t())
    assert list(df.columns) == ["velocities/vc-kts", "position/h-sl-ft"]


def test_linearization_modes_c172(c172_trimmed):
    fdm, _ = c172_trimmed
    lin = linearize(fdm)
    lon = lin.subsystem(LONG_STATES, ["DeCmd", "ThtlCmd"])
    m = modes(lon.A, lon.x_names)
    osc = m[m.eig_imag > 0]
    assert len(osc) == 2  # short period + phugoid
    sp, ph = osc.iloc[0], osc.iloc[1]
    assert 2.0 < sp.wn < 10.0 and 0.3 < sp.zeta < 1.0       # short period
    assert 0.05 < ph.wn < 0.5 and 0.0 < ph.zeta < 0.3       # phugoid
    assert lon.to_control().nstates == 4


def test_signals():
    t = np.arange(0, 10, 0.01)
    d = doublet(t, 1.0, 0.5, 2.0)
    assert d.max() == 2.0 and d.min() == -2.0 and abs(d.sum()) < 1e-9
    m = multistep_3211(t, 1.0, 0.2, 1.0)
    assert abs(m.sum() * 0.01 - (3 - 2 + 1 - 1) * 0.2) < 0.02
    c = chirp(t, 1.0, 8.0, 0.1, 2.0, taper=0.5)
    assert np.all(np.abs(c) <= 1.0) and c[t < 1.0].max() == 0.0
    assert step(0.5, 1.0) == 0.0 and step(1.5, 1.0) == 1.0


def test_step_metrics_second_order():
    import control
    zeta, wn = 0.5, 2.0
    sys = control.tf([wn**2], [1, 2 * zeta * wn, wn**2])
    t = np.linspace(0, 15, 3001)
    t, y = control.step_response(sys, t)
    m = step_metrics(t, y, 1.0, 0.0, band=0.02)
    expected_os = 100 * np.exp(-np.pi * zeta / np.sqrt(1 - zeta**2))
    assert m.overshoot_pct == pytest.approx(expected_os, rel=0.02)
    assert m.settling_time == pytest.approx(4 / (zeta * wn), rel=0.25)
    assert abs(m.steady_state_error) < 1e-3
    assert rms([3, 4, 3, 4]) == pytest.approx(np.sqrt(12.5))


def test_delay_margin():
    import control
    L = control.tf([1], [1, 0])  # integrator: wgc = 1 rad/s, PM = 90 deg
    mg = loop_margins(L)
    assert mg.phase_margin_deg == pytest.approx(90, abs=0.5)
    assert mg.delay_margin_s == pytest.approx(np.pi / 2, rel=0.01)
