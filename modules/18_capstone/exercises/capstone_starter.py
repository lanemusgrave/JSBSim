"""Module 18 capstone: starter.  Fill in each step; see the README for the brief.

Everything you need already exists:
  gnclab.mission      fly_mission(case, manager="line"|"fillet"), MISSION_REQUIREMENTS, MISSION_WPS
  gnclab.montecarlo   sample_cases, DEFAULT_DISPERSIONS, Normal, run_cases, evaluate, pass_rate_ci, sensitivity
  gnclab.sysid        measured_coefficients, EQUATION_ERROR_MODELS, ols
  modules/14_system_id/solutions/flight_test.py   maneuver(..., props=..., module="18_capstone")
  modules/07_build_uas_model/solutions/build_gnc_trainer.py   P (design parameters, mass properties)

Rules: do not open solutions/as_built.py.  To fly the as-built airplane, import AS_BUILT
from it and pass it through (fly_mission(dict(case, **AS_BUILT)); props for maneuver()),
without printing it.  Parallel runs spawn processes: keep work inside main().

    python modules/18_capstone/exercises/capstone_starter.py
"""

import sys
from pathlib import Path

from gnclab.cli import parse_args

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "14_system_id" / "solutions"))
sys.path.insert(0, str(HERE.parents[1] / "07_build_uas_model" / "solutions"))
sys.path.insert(0, str(HERE.parent / "solutions"))

WEATHER = {"wind_mps": 5.0, "wind_from_deg": 270.0, "turb_w20_fps": 10.0}


def main():
    args = parse_args(__doc__)
    # 1. Design check: fly_mission(nominal case + WEATHER) and evaluate against MISSION_REQUIREMENTS.
    #    Fix what fails (hint: Module 12).

    # 2. Flight test: fly long and lat 3-2-1-1s on the as-built airplane, identify Cm, Cl, Cn.

    # 3. Model update: build a dispersion dict centred on the identified scales. Choose sigma honestly.

    # 4. Monte Carlo: design model vs updated model, fillet manager, forecast weather (W20 5-15 ft/s).
    #    Fly the "first flight" (as-built, forecast weather, 3 seeds) and place it in each prediction.

    # 5. Memo: fill in ../memo_template.md with your numbers and judgment.


if __name__ == "__main__":
    main()
