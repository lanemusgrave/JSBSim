"""Capstone mission regression (Module 18)."""

import pandas as pd

from gnclab.mission import MISSION_REQUIREMENTS, fly_mission
from gnclab.montecarlo import evaluate, sample_cases


def test_nominal_mission_with_fillets_meets_every_requirement():
    nom = sample_cases(1).iloc[0].to_dict()
    r = fly_mission(dict(nom, wind_mps=5.0, wind_from_deg=270.0, turb_w20_fps=10.0), manager="fillet")
    assert r["status"] == "ok"
    res = evaluate(pd.DataFrame([r]), MISSION_REQUIREMENTS)
    failed = [k for k in MISSION_REQUIREMENTS if not res.loc[0, k]]
    assert not failed, f"{failed}: {r}"
