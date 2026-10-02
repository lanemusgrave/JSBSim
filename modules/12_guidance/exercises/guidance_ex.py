"""Module 12 exercises.

A. Fillets (B&M Algorithm 6).  The WaypointManager switches legs AT each
   waypoint, so a 90 deg corner overshoots by about one turn radius
   (R = V^2 / (g tan(phi_max)) ~ 110 m here).  Write a FilletManager that
   flies each corner as an orbit segment of radius R tangent to both legs:
     - on leg i, switch to the fillet when crossing the half-plane at
       z1 = w_i - (R / tan(rho/2)) q_{i-1}      (rho = angle between legs)
     - fly the orbit centred at c = w_i - (R / sin(rho/2)) (q_{i-1} - q_i)/|q_{i-1} - q_i|
       with direction lam = sign(q_{i-1,n} q_{i,e} - q_{i-1,e} q_{i,n})
     - leave the orbit when crossing the half-plane at z2 = w_i + (R / tan(rho/2)) q_i
   Fly solutions/waypoint_mission.py's mission with it; compare corner overshoot.

B. Tuning.  Sweep k_path (0.005 ... 0.2) and chi_inf (30 ... 90 deg) for the
   150 m-offset line capture.  What happens when k_path is too large?  Relate
   it to the course loop bandwidth (Module 10).

C. (Advanced, no reference solution) Total Energy Control (TECS).  Replace the
   altitude-from-pitch / speed-from-throttle split with: throttle controls the
   total specific energy rate  E_dot/(mgV) = gamma + V_dot/g, pitch controls the
   energy distribution  gamma - V_dot/g.  Subclass gnclab.controllers.LongitudinalAP.
   Compare a combined climb + speed change with the classic loops.

    python modules/12_guidance/exercises/guidance_ex.py
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

# TODO A

# TODO B
