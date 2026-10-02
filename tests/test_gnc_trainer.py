"""Numerical sanity checks for the gnc_trainer model (Module 07)."""

import sys
from pathlib import Path

import numpy as np
import pytest

from gnclab import initialize, make_fdm
from gnclab.linear import linearize_fd, modes
from gnclab.trim import trim

SOL = Path(__file__).resolve().parents[1] / "modules" / "07_build_uas_model" / "solutions"


@pytest.fixture(scope="module")
def trimmed():
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048})
    trim(fdm, "full")
    return fdm


def test_trim_25mps(trimmed):
    assert 2.5 < trimmed["aero/alpha-deg"] < 3.5
    assert 0.7 < trimmed["fcs/throttle-cmd-norm[0]"] < 0.85
    assert abs(trimmed["accelerations/Nz"] - 1) < 0.01


def test_matches_reference_model(trimmed):
    sys.path.insert(0, str(SOL))
    import reference_model as ref

    rho = trimmed["atmosphere/rho-slugs_ft3"] * 515.379
    x0, u0 = ref.trim_level(25.0, rho)
    S = np.diag([0.3048, 1, 1, 1, 1, 1, 1, 1])
    lin = linearize_fd(trimmed, ["fcs/elevator-cmd-norm", "fcs/aileron-cmd-norm", "fcs/rudder-cmd-norm"])
    ev_jsb = np.sort_complex(np.linalg.eigvals(S @ lin.A @ np.linalg.inv(S)))
    ev_ref = np.sort_complex(np.linalg.eigvals(ref.linearize(x0, u0, rho)[0]))
    big = np.abs(ev_ref) > 0.05
    assert np.allclose(ev_jsb[big], ev_ref[big], rtol=0.02, atol=0.02)
