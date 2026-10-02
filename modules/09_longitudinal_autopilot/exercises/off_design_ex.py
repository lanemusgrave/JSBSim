"""Module 09 exercise: how good are 25 m/s gains at 18 and 32 m/s?

TODO 1: turn the gain formulas of solutions/design_longitudinal.py into a
        function design(Va_mps) -> dict of the six ap/gains/... values
        (linearize at Va, compute a_th*, a_V*, the inner loop, the REAL pitch-loop
        DC gain, then the altitude and airspeed loops).
TODO 2: for Va in (18, 25, 32) m/s fly the 100 ft altitude step twice:
        (a) with the default 25 m/s gains in the XML, (b) with design(Va) gains
        written into the ap/gains/... properties after trimming.
        Keep the Python wing leveler from verify_longitudinal.py.
TODO 3: tabulate overshoot and settling time.  Which speed suffers most with
        fixed gains, and why (think dynamic pressure and elevator power)?

    python modules/09_longitudinal_autopilot/exercises/off_design_ex.py
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

# TODO 1

# TODO 2

# TODO 3
