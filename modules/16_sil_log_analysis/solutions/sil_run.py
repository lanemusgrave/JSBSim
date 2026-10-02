"""Module 16 - fly the longitudinal autopilot as separate flight software over UDP.

1. Validate the harness: lockstep SIL must reproduce the in-process controller
   exactly.  Then the sockets, processes and packing add nothing.
2. Transport delay (lockstep, deterministic): find the cliff and compare it
   with Module 11's linear delay margin (~90 ms).
3. Real time: what latency and jitter does the loop actually see?  And what
   happens when the flight computer is slower than its frame?

    python modules/16_sil_log_analysis/solutions/sil_run.py [--show] [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np

from gnclab.cli import parse_args
from gnclab.plotting import save
from gnclab.sil import SILConfig, in_process_reference, run_sil

MODULE = "16_sil_log_analysis"


def hold_stats(s, t0=15.0, t1=30.0):
    seg = s.loc[t0:t1]
    return np.degrees(seg.q_rad_sec.std()), np.degrees(seg.elevator_rad.std())


def main():
    args = parse_args(__doc__)
    T = 30.0

    # 1. validation
    cfg = SILConfig(duration=T)
    ref = in_process_reference(cfg)
    sil = run_sil(cfg)
    print("1. lockstep SIL vs in-process controller (same law, same 40 Hz frame):")
    for c in ("alt_ft", "theta_rad", "elevator_rad", "throttle"):
        print(f"   {c:13s} max |difference| = {np.abs(ref[c].to_numpy() - sil[c].to_numpy()).max():.3g}")

    # 2. transport delay
    print("\n2. transport delay (lockstep, 40 Hz frame = 25 ms):")
    print(f"   {'delay':>6s} {'added':>7s} {'q RMS in hold':>14s} {'elevator RMS':>13s} {'alt overshoot':>14s}")
    runs = {}
    delays = (0, 2, 3) if args.fast else (0, 1, 2, 3, 4, 5)
    for d in delays:
        s = run_sil(SILConfig(duration=T, delay_frames=d))
        runs[d] = s
        q, de = hold_stats(s)
        over = (s.alt_ft - s.alt_cmd_ft).loc[2:T].max()
        print(f"   {d:6d} {25 * d:5d} ms {q:11.2f} °/s {de:10.2f} °  {over:11.1f} ft")

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 7))
    for d in (0, 2, 3):
        s = runs[d]
        axes[0].plot(s.index, s.alt_ft, label=f"+{25 * d} ms")
        axes[1].plot(s.index, np.degrees(s.q_rad_sec))
        axes[2].plot(s.index, np.degrees(s.elevator_rad))
    axes[0].plot(s.index, s.alt_cmd_ft, "k--", lw=0.8, label="command")
    for ax, lab in zip(axes, ("altitude [ft]", "q [deg/s]", "elevator [deg]")):
        ax.set_ylabel(lab)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize="small")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("SIL transport delay: altitude looks fine while the pitch loop limit-cycles")
    fig.tight_layout()
    save(fig, MODULE, "sil_delay", show=args.show)

    # 3. real time
    T_rt = 3.0 if args.fast else 20.0
    print(f"\n3. real time ({T_rt:.0f} s of wall clock per run):")
    rt = run_sil(SILConfig(duration=T_rt, mode="realtime"))
    ages = {int(k): int(v) for k, v in rt.age_frames.value_counts().sort_index().items()}
    print("   command age [frames]:", ages, f"| frame overruns: {(rt.overrun_ms > 0).sum()}"
          f" (max {rt.overrun_ms.max():.1f} ms)")
    lk = run_sil(SILConfig(duration=T_rt, delay_frames=1))
    print(f"   realtime vs lockstep with delay_frames=1: max |alt difference| "
          f"{np.abs(rt.alt_ft.to_numpy() - lk.alt_ft.to_numpy()).max():.3g} ft")
    T_slow = 2.0 if args.fast else 8.0
    for drain in (False, True):
        s = run_sil(SILConfig(duration=T_slow, mode="realtime", compute_ms=30.0, drain=drain))
        print(f"   controller takes 30 ms per 25 ms frame, drain={drain!s:5s}: command age at "
              f"t = 1, 2, ... s: {s.age_frames.iloc[40::40].tolist()} frames")


if __name__ == "__main__":
    main()
