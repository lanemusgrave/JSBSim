"""Module 16 exercise A (solution): average vs worst-case latency.

Two flight computers with the SAME mean compute time (20 ms of a 25 ms
frame): one constant, one jittery (uniform 0-40 ms).  Real time, 100 ft step.
Which one does the control loop care about?

    python modules/16_sil_log_analysis/solutions/sil_ex.py [--show] [--fast]
"""

import numpy as np

from gnclab.cli import parse_args
from gnclab.sil import SILConfig, run_sil


def main():
    args = parse_args(__doc__)
    T = 4.0 if args.fast else 25.0
    cases = {"constant 20 ms": dict(compute_ms=20.0), "jitter 0-40 ms": dict(jitter_ms=40.0),
             "constant 30 ms": dict(compute_ms=30.0)}
    print(f"{'flight computer':16s} {'command age [frames]: share of frames':45s} {'q RMS':>8s}")
    for name, kw in cases.items():
        s = run_sil(SILConfig(duration=T, mode="realtime", **kw))
        ages = s.age_frames.value_counts(normalize=True).sort_index()
        dist = "  ".join(f"{int(a)}: {100 * p:4.1f}%" for a, p in ages.items())
        q = np.degrees(s.loc[0.6 * T:, "q_rad_sec"].std())          # altitude hold after the step
        print(f"{name:16s} {dist:45s} {q:6.2f}°/s")


if __name__ == "__main__":
    main()
