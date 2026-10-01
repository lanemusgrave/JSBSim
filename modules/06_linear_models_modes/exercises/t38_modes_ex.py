"""Module 06 exercise: how do the T-38's modes change across the envelope?

TODO 1: for each flight condition below, trim (mode "full"), linearize with
        gnclab.linear.linearize, and extract short period, phugoid, Dutch roll,
        roll and spiral (wn, zeta, period / time constant).
TODO 2: put them in one pandas table and print it.
TODO 3: answer in comments:
   a) The short-period frequency should scale roughly with V (at constant
      altitude) and with sqrt(rho) (at constant TAS).  Does it?  Why?
   b) Phugoid period ~ sqrt(2) * pi * V / g.  Check it.
   c) Which condition has the worst Dutch-roll damping?  Would you want a yaw
      damper?  (MIL-F-8785C Level 1 wants zeta_DR >= 0.19 for this class.)

    python modules/06_linear_models_modes/exercises/t38_modes_ex.py
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

CONDITIONS = [  # (name, altitude ft, KCAS)
    ("low/slow", 5000.0, 200.0),
    ("low/fast", 5000.0, 400.0),
    ("high/slow", 25000.0, 200.0),
    ("high/fast", 25000.0, 330.0),
]

# TODO 1-2

# TODO 3
