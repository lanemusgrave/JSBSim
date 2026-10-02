"""Module 15 exercises.

A. Conditional requirements.  Every MC-3 failure in solutions/monte_carlo.py
   happened in turbulence with W20 > 15 ft/s.  Run a STRATIFIED Monte Carlo:
   fix turb_w20_fps to each bin (0-10, 10-15, 15-20, 20-25 ft/s, sampled
   uniformly within the bin) with everything else dispersed, ~60 runs per bin.
   Report MC-3's pass rate and 95 % interval per bin.  Then decide whether
   this is a design problem or a requirement problem, and rewrite MC-3 so
   that it states its conditions (and has a separate turbulence criterion).

B. How many runs?  Your program office wants "P(fail) < 0.5 % at 95 %
   confidence".  How many failure-free runs is that?  If one run fails, how
   many runs does the claim need?  (Hint: Clopper-Pearson upper bound;
   gnclab.montecarlo.pass_rate_ci.)

C. Gyro bias.  gnc_sensors.xml has sensors/gyro-bias-{p,q,r}-rad_sec
   (case keys gyro_bias_p/q/r).  Sweep each axis alone from 0 to 1 rad/s.
   Which requirement is most sensitive, and where is its cliff?  Explain
   each axis from the autopilot structure, then ask what the sensor model
   is NOT modelling.

The worker processes are spawned (as on Windows): keep all work inside main()
behind the if __name__ == "__main__": guard.

    python modules/15_monte_carlo_vv/exercises/mc_ex.py
"""

from gnclab.cli import parse_args


def main():
    args = parse_args(__doc__)
    # TODO A: build cases with sample_cases(...), overwrite turb_w20_fps per bin,
    #         run_cases(...), evaluate(...), pass_rate_ci(...) per bin

    # TODO B

    # TODO C


if __name__ == "__main__":
    main()
