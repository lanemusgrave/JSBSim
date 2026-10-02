"""Monte Carlo helpers and a small requirements regression (Module 15)."""

import math

import pytest

from gnclab.montecarlo import (REQUIREMENTS, evaluate, pass_rate_ci, requirements_flight, run_cases,
                               runs_for_confidence, sample_cases, sensitivity, unusual)


def test_sampling_is_reproducible_and_case0_is_nominal():
    a, b = sample_cases(20, seed=7), sample_cases(20, seed=7)
    assert a.equals(b)
    assert not a.equals(sample_cases(20, seed=8))
    assert a.loc[0, "scale_Cma"] == 1.0 and a.loc[0, "wind_mps"] == 0.0 and a.loc[0, "turb_w20_fps"] == 0.0
    assert a["scale_CD"].min() >= 0.6                      # truncation applied


def test_statistics():
    assert runs_for_confidence(0.01) == 299                # the "rule of three": ~3/p
    p, lo, hi = pass_rate_ci(300, 300)
    assert p == 1.0 and hi == 1.0 and lo == pytest.approx(0.025 ** (1 / 300), rel=1e-6)   # exact for k = n
    p, lo, hi = pass_rate_ci(5, 10)
    assert lo < 0.5 < hi


def test_nominal_case_meets_every_requirement():
    r = requirements_flight(sample_cases(1).iloc[0].to_dict())
    assert r["status"] == "ok"
    res = evaluate(__import__("pandas").DataFrame([r]))
    failed = [k for k in REQUIREMENTS if not res.loc[0, k]]
    assert not failed, f"nominal case fails {failed}: {r}"


def test_dispersions_reach_the_model():
    nom = sample_cases(1).iloc[0].to_dict()
    weak = requirements_flight(dict(nom, scale_Clda=0.5))
    base = requirements_flight(nom)
    assert weak["chi_overshoot_deg"] > base["chi_overshoot_deg"] + 0.5   # less aileron power -> more overshoot


def test_small_monte_carlo_regression():
    """12 dispersed cases, serial.  If a change to the autopilot or model makes
    any of these fail, the regression catches it before a full Monte Carlo."""
    res = evaluate(run_cases(sample_cases(12, seed=2026), workers=1, progress=False))
    assert (res.status == "ok").all()
    assert res["pass"].all(), res.loc[~res["pass"], list(REQUIREMENTS)]
    assert math.isfinite(sensitivity(res, "chi_overshoot_deg").iloc[0])
    assert "sig" in unusual(res.iloc[1])
