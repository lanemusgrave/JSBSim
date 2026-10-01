"""Module 01 exercise: your own loop, a jet you know.

Write the JSBSim loop yourself (raw API, like solutions/hand_loop.py) for the
bundled T-38 at 10,000 ft and 300 KCAS:

TODO 1: create the FGFDMExec, load "T38", set the ICs (position first!),
        run_ic(), set engines running, trim with do_trim(1).
TODO 2: at t = 2 s apply a 1 s aileron pulse of +0.2 (normalized) on top of the
        trimmed aileron command, then return to trim.
TODO 3: log phi [deg], p [deg/s], beta [deg], r [deg/s] and total thrust (find
        the thrust property names with fdm.query_property_catalog("thrust")).
TODO 4: plot them and save to outputs/01_first_simulation/t38_roll.png.
TODO 5: answer in a comment: what bank angle did you end with, and why doesn't
        the airplane roll back to wings level by itself?

Run:  python modules/01_first_simulation/exercises/t38_roll.py --show
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import jsbsim  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

jsbsim.FGJSBBase().debug_lvl = 0

# TODO 1
fdm = None

# TODO 2-3: the loop
log = {"t": [], "phi": [], "p": [], "beta": [], "r": [], "thrust": []}

# TODO 4: plot

# TODO 5: answer
