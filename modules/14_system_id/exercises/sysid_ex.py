"""Module 14 exercises.

A. Excitation design.  Using maneuver() from solutions/flight_test.py, fly three
   elevator inputs of similar peak amplitude: a single STEP (hold 5 s), a DOUBLET
   and a 3-2-1-1.  Identify the Cm derivatives from each (equation error, as in
   solutions/equation_error.py) and compare: errors vs truth, and the correlation
   between the alpha and delta_e regressors.  Which input is best, and why?

B. Noise.  Double every NOISE entry in flight_test.py and re-run equation and
   output error.  Which method degrades more?  Why (think errors-in-variables)?

C. Lateral output error.  Write an output-error fit for the Dutch roll
   (states beta, p, r, phi) and compare Cnr and Clp with the equation-error answers.

    python modules/14_system_id/exercises/sysid_ex.py
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

# TODO A

# TODO B

# TODO C
