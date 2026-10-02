"""Module 10 exercises.

A. Read the C172's autopilot (jsbsim data folder: aircraft/c172x/c172ap.xml),
   the RM Case Study 3 "wing leveler" and "heading hold".  Draw its block
   diagram (paper is fine) and list three differences from ours (structure,
   wrap-around handling, anti-windup, feedback signals).

B. Sideslip feedback.  Copy aircraft/gnc_trainer/fcs/gnc_autopilot.xml to
   modules/10_lateral_autopilot/exercises/fcs/gnc_autopilot.xml and change the
   yaw channel to   dr = kr * washout(r) - kbeta * beta   (aero/beta-rad as feedback).
   Fly the 90 deg course change with kbeta = 0, 1, 2 (set ap/gains/kbeta from Python)
   with systems_dir="modules/10_lateral_autopilot/exercises/fcs".
   How much does peak |beta| drop?  What happens to the Dutch roll?

C. Bank limit.  Raise ap/phi-max-rad to 45 deg and repeat the course change.
   How much faster is the turn, and what does it cost the altitude hold (load
   factor 1/cos(phi))?

    python modules/10_lateral_autopilot/exercises/lateral_ex.py
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

# TODO B

# TODO C
