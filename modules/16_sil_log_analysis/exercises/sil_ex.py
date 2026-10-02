"""Module 16 exercises.

A. Average vs worst-case latency.  Fly the default card in real time with
   three flight computers: constant 20 ms compute (SILConfig(compute_ms=20)),
   jittery 0-40 ms (jitter_ms=40, same mean), and constant 30 ms.  Compare the
   command-age distribution (log column age_frames) and the pitch-rate RMS in
   altitude hold.  Which statistic of compute time does the loop care about?

B. Log toolkit.  From outputs/16_sil_log_analysis/plant_log.csv (written by
   solutions/log_analysis.py), find every interval where the airspeed is more
   than 3 ft/s from its command for over 1 s (gnclab.logs.intervals).  Which
   test points cause them, and does any violate R-L2?

C. (Open-ended) Move the lateral autopilot into the flight-computer process:
   extend the SENS/CMD_ packets (phi, p, r, chi; aileron, rudder), port
   the roll/course/yaw-damper law from gnc_autopilot.xml to Python, and
   validate it against the XML in lockstep before you trust it.

The SIL spawns a process: keep work inside main() behind the __main__ guard.

    python modules/16_sil_log_analysis/exercises/sil_ex.py
"""

from gnclab.cli import parse_args


def main():
    args = parse_args(__doc__)
    # TODO A: run_sil(SILConfig(mode="realtime", duration=25, compute_ms=..., jitter_ms=...))

    # TODO B

    # TODO C


if __name__ == "__main__":
    main()
