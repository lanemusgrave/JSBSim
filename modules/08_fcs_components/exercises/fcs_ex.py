"""Module 08 exercises.

A. Characterize an actuator from data (a preview of system ID).
   Fly aircraft/m08_fcs_rig with a large step and a small step on rig/input
   and, from rig/act-realistic alone, estimate:
     - the rate limit [units/s]   (slope of the large-step response)
     - the lag bandwidth [rad/s]  (small step: 63% rise time = 1/bandwidth)
     - the transport delay [s]    (time before the output first moves)
     - the travel limits
   Compare with the XML.  Why do you need BOTH a small and a large step?

B. Find the latency-induced stability limit in the NONLINEAR sim.
   Using modules/08_fcs_components/solutions/fcs as systems_dir, with
   actuators on and ap/q-delay-on = 1, sweep ap/kq upward from 0.5 and find
   the smallest gain at which a doublet leaves a sustained oscillation
   (e.g. |q| > 5 deg/s between t = 4 and 8 s).  Compare with the linear
   prediction printed by solutions/pitch_damper.py (~0.97).

C. Copy solutions/fcs/gnc_autopilot.xml to exercises/fcs/gnc_autopilot.xml and add
   a lag_filter (c1 = 20 rad/s) on the q feedback (a typical gyro noise filter).
   With sensors on, how much does it cut elevator activity from noise, and
   what does it do to the damping?  (systems_dir="modules/08_fcs_components/exercises/fcs")

    python modules/08_fcs_components/exercises/fcs_ex.py
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

# TODO A

# TODO B

# TODO C
