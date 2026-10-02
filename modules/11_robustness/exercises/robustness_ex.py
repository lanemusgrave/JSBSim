"""Module 11 exercises.

A. Back-calculation anti-windup.  Subclass gnclab.controllers.LongitudinalAP
   and replace the altitude integrator's "freeze" logic with back-calculation:
       int_h += (e_h + (thc - thc_raw) / (kp_h * Tt)) * dt,   Tt ~ 1/ki_h ... sqrt(Ti*Td)
   (the saturation error bleeds the integrator back).  Compare with freezing and
   with no anti-windup on the 400 ft climb from solutions/antiwindup.py.

B. Latency budget.  At a 25 Hz controller frame, what is the largest output
   latency before the pitch loop limit-cycles, with sensors and actuators on?
   (Hint: a frame-based controller can only add latency in whole frames, and
   with sensors on there's a noise floor - compare against the 0-latency run.)
   Express it as a "latency budget" a flight-software engineer could design to.

C. Schedule in XML.  In your own copy of gnc_autopilot.xml, make the pitch gains
   ap/gains/kp-theta and kd-theta scale with 7.92 / aero/qbar-psf using an
   <fcs_function>, clipped to [0.5, 2].  Re-run gain_schedule.py's nonlinear
   part against the XML autopilot (engage_longitudinal(altitude=False) and step
   ap/theta-cmd-ext-rad).

    python modules/11_robustness/exercises/robustness_ex.py
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

# TODO A

# TODO B

# TODO C
