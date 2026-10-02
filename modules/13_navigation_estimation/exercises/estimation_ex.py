"""Module 13 exercises.

A. Gyro bias.  gnclab.estimation.Sensors adds a constant gyro bias
   (0.004, -0.003, 0.002 rad/s).  Extend AttitudeEKF to 5 states
   [phi, theta, b_p, b_q, b_r] (biases as random walks, subtracted from the
   gyro).  Fly solutions/attitude_estimation.py's S-turns and report the bias
   estimates after 60 s and the attitude error RMS vs the 2-state EKF.
   Which biases converge, and why is b_r the hardest?

B. Tuning.  Halve and double R (accelerometer noise) and Q (process noise) of
   the 2-state EKF.  Plot roll error vs time for each.  Which way trades
   noise for lag?

C. Wind.  With GPS ground speed/course (Sensors.gps) and pitot airspeed plus
   heading, estimate the horizontal wind vector in a 6 m/s crosswind
   (hint: V_ground = V_air + V_wind; fly an orbit so the problem is observable).

    python modules/13_navigation_estimation/exercises/estimation_ex.py
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

# TODO A

# TODO B

# TODO C
