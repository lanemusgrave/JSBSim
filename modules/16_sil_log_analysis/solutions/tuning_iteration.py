"""Module 16 - one flight-test tuning iteration, on the SIL rig.

Situation: the new servo bus adds 50 ms of transport delay (2 frames at
40 Hz).  The test card is flown in light turbulence (W20 = 10 ft/s) and
scored against two requirements:
  R-L1  altitude step overshoot <= 10 %, settles (+/-10 ft) within 30 s
  R-Q   pitch-rate RMS in altitude hold <= 3 deg/s     (ride quality / servo wear)

Iteration 0 is the baseline; each later iteration is a hypothesis from the
previous log, re-flown on the rig.  Lockstep, same seed: any change in the
numbers comes from the gains, not from the noise.

    python modules/16_sil_log_analysis/solutions/tuning_iteration.py [--show] [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import welch

from gnclab.cli import parse_args
from gnclab.controllers import DEFAULT_GAINS
from gnclab.metrics import step_metrics
from gnclab.plotting import save
from gnclab.sil import SILConfig, run_sil

ITERATIONS = [
    ("0 baseline", 1.0, 1.0, "Module 09 gains"),
    ("1 scale pitch loop", 0.65, 0.65, "less gain -> more delay margin"),
    ("2 cut kd more than kp", 0.8, 0.5, "delay hurts the q (derivative) path most"),
]


def card(t, h0, v0):
    return h0 + (100.0 if t >= 2.0 else 0.0), v0


def fly(kp_scale, kd_scale, T, delay_frames=2, w20=10.0):
    g = dict(DEFAULT_GAINS)
    g["kp_theta"] *= kp_scale
    g["kd_theta"] *= kd_scale
    s = run_sil(SILConfig(duration=T, delay_frames=delay_frames, gains=g, turbulence_w20_fps=w20, card=card))
    h0 = s.alt_ft.iloc[0]
    m = step_metrics(s.index, s.alt_ft, h0 + 100, h0, t_step=2.0, band=0.10)
    q_rms = np.degrees(s.loc[15.0:, "q_rad_sec"].std())
    return s, m, q_rms


def verdict(m, q_rms):
    r_l1 = m.overshoot_pct <= 10.0 and m.settling_time <= 30.0
    return r_l1, q_rms <= 3.0


def main():
    args = parse_args(__doc__)
    T = 25.0 if args.fast else 40.0
    print(f"{'iteration':24s} {'kp':>5s} {'kd':>5s} {'overshoot':>10s} {'settle':>7s} {'q RMS':>9s}  R-L1  R-Q   rationale")
    runs = {}
    for name, kp, kd, why in ITERATIONS:
        s, m, q = fly(kp, kd, T)
        r1, rq = verdict(m, q)
        runs[name] = s
        print(f"{name:24s} {kp:5.2f} {kd:5.2f} {m.overshoot_pct:9.1f}% {m.settling_time:6.1f}s {q:6.2f}°/s  "
              f"{'PASS' if r1 else 'FAIL'}  {'PASS' if rq else 'FAIL'}  {why}")

    # Diagnose from the log: where is the q energy?  (the evidence for iteration 2)
    fig, axes = plt.subplots(2, 1, figsize=(9, 7))
    for name, s in runs.items():
        q = s.loc[15.0:, "q_rad_sec"].to_numpy()
        f, P = welch(np.degrees(q), fs=40.0, nperseg=min(256, len(q)))
        axes[0].plot(s.index, s.alt_ft, label=name)
        axes[1].semilogy(f, P, label=name)
    axes[0].set(xlabel="time [s]", ylabel="altitude [ft]", title="Tuning iterations (50 ms extra delay, W20 = 10 ft/s)")
    axes[1].set(xlabel="frequency [Hz]", ylabel="q PSD [(deg/s)^2/Hz]", title="Pitch-rate spectrum in hold: "
                "the delay-induced peak near 3 Hz")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize="small")
    fig.tight_layout()
    save(fig, "16_sil_log_analysis", "tuning_iteration", show=args.show)

    # Regression check: the new gains must not break the original configuration.
    _, kp, kd, _ = ITERATIONS[-1]
    print("\nRe-check the candidate where the old gains were verified, and one step beyond:")
    for d, w in ((0, 0.0), (0, 10.0), (3, 10.0)):
        _, m, q = fly(kp, kd, T, delay_frames=d, w20=w)
        r1, rq = verdict(m, q)
        print(f"   delay {25 * d:3d} ms, W20 {w:4.1f}: overshoot {m.overshoot_pct:4.1f} %, q RMS {q:5.2f} deg/s  "
              f"R-L1 {'PASS' if r1 else 'FAIL'}  R-Q {'PASS' if rq else 'FAIL'}")


if __name__ == "__main__":
    main()
