"""Module 16 - analyze flight-test logs with gnclab.logs.

Part 1: a real-time SIL "flight" of a test card (+100 ft, +10 ft/s, -100 ft)
in light turbulence produces two logs, as on an aircraft:
  plant_log.csv  "data acquisition": truth at 40 Hz, sim-time clock
  fc_log.csv     "flight computer": what it measured and commanded, its OWN
                 clock (seconds since it booted), jittered, no frame numbers
We align the two clocks by cross-correlation, find the test points from the
command channels, find saturation events, and score each test point
against the requirements.

Part 2: the Module 14 system-ID logs joined into one long "flight": find
the maneuvers automatically from control-surface activity.

    python modules/16_sil_log_analysis/solutions/log_analysis.py [--show] [--fast]
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from gnclab.cli import parse_args
from gnclab.logs import activity_windows, estimate_offset, intervals, resample, steps
from gnclab.metrics import step_metrics
from gnclab.plotting import outdir, save
from gnclab.sil import SILConfig, run_sil

MODULE = "16_sil_log_analysis"
REPO = Path(__file__).resolve().parents[3]


def card(t, h0, v0):
    alt = h0 + (100.0 if 2.0 <= t < 50.0 else 0.0)
    return alt, v0 + (10.0 if t >= 30.0 else 0.0)


# test point -> (channel, settling band, requirement checks)
LIMITS = {"alt_cmd_ft": {"overshoot_pct": 10.0, "settling_time": 30.0, "band": 10.0},
          "vt_cmd_fps": {"overshoot_pct": 20.0, "settling_time": 20.0, "band": 1.5}}
RESPONSE = {"alt_cmd_ft": "alt_ft", "vt_cmd_fps": "vt_fps"}


def part1(args, out):
    T = 8.0 if args.fast else 70.0
    fc_path = out / "fc_log.csv"
    print(f"Part 1: flying the test card in real time ({T:.0f} s)...")
    # The flight computer was "powered on" a while before the card started; its
    # log clock counts from then.  We pretend not to know the 123.456 s.
    plant = run_sil(SILConfig(duration=T, mode="realtime", card=card, turbulence_w20_fps=8.0,
                              controller_log=str(fc_path), fc_clock_offset_s=123.456))
    plant.to_csv(out / "plant_log.csv")

    # --- load as you would from disk
    plant = pd.read_csv(out / "plant_log.csv", index_col="t")
    fc = pd.read_csv(fc_path, index_col="t_fc")
    dts = np.diff(fc.index.to_numpy())
    print(f"  plant log: {len(plant)} rows, uniform 40 Hz | flight-computer log: {len(fc)} rows, "
          f"interval {1000 * dts.mean():.1f} ms mean, {1000 * dts.min():.1f}..{1000 * dts.max():.1f} ms")

    # --- 1. clocks: fc time + d = plant time.  Correlate a busy signal both logs share.
    d, peak = estimate_offset(fc.q_meas, plant.q_meas, max_lag_s=300.0)
    print(f"  clock offset: plant time = flight-computer time {d:+.3f} s (correlation peak {peak:.2f}; "
          f"emulated power-on offset 123.456 s)")
    fc_on_plant = resample(fc.set_axis(fc.index + d), 40.0, plant.index[0], plant.index[-1])
    both = plant.join(fc_on_plant, rsuffix="_fc", how="inner")
    err = np.degrees((both.q_meas - both.q_meas_fc).abs())
    print(f"  after alignment: |q (plant's copy) - q (fc's copy)| median {err.median():.3f} deg/s, "
          f"max {err.max():.2f} deg/s (jitter + interpolation)")

    # --- 2. test points from the command channels
    tps = []
    for ch in LIMITS:
        for _, s in steps(plant[ch], min_jump=1.0).iterrows():
            tps.append((s.t, ch, s.before, s.after))
    tps.sort()
    ends = [t for t, *_ in tps[1:]] + [plant.index[-1]]

    # --- 3. events
    sat_thr = intervals(plant.throttle >= 0.999, min_duration=0.1)
    sat_de = intervals(plant.de_cmd.abs() >= 0.99, min_duration=0.05)
    print(f"  throttle saturated: {len(sat_thr)} intervals, {sat_thr.duration.sum():.1f} s total"
          + "".join(f"\n    {r.start:6.2f} - {r.end:6.2f} s" for _, r in sat_thr.iterrows()))
    print(f"  elevator command saturated: {len(sat_de)} intervals")

    # --- 4. score each test point
    print(f"\n  {'t [s]':>6s} {'test point':22s} {'overshoot':>10s} {'settle':>8s} {'final err':>10s}  verdict")
    rows = []
    for (t0, ch, before, after), t1 in zip(tps, ends):
        seg = plant.loc[t0 - 1.0:t1]
        y = seg[RESPONSE[ch]]
        lim = LIMITS[ch]
        m = step_metrics(seg.index, y, after, y.loc[t0:].iloc[0], t_step=t0, band=lim["band"] / abs(after - before))
        ok = m.overshoot_pct <= lim["overshoot_pct"] and m.settling_time <= lim["settling_time"]
        name = f"{ch.split('_')[0]} {after - before:+.0f}"
        rows.append(dict(t=t0, test_point=name, overshoot_pct=m.overshoot_pct, settle_s=m.settling_time,
                         final_err=m.steady_state_error, passed=ok))
        print(f"  {t0:6.1f} {name:22s} {m.overshoot_pct:9.1f}% {m.settling_time:7.1f}s {m.steady_state_error:+9.2f}"
              f"  {'PASS' if ok else 'FAIL'}")
    pd.DataFrame(rows).to_csv(out / "test_points.csv", index=False)

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 8))
    axes[0].plot(plant.index, plant.alt_ft, label="altitude")
    axes[0].plot(plant.index, plant.alt_cmd_ft, "k--", lw=0.8, label="command")
    axes[1].plot(plant.index, plant.vt_fps)
    axes[1].plot(plant.index, plant.vt_cmd_fps, "k--", lw=0.8)
    axes[2].plot(plant.index, np.degrees(plant.q_meas), label="plant log")
    axes[2].plot(fc_on_plant.index, np.degrees(fc_on_plant.q_meas), lw=0.8, label="fc log, aligned")
    axes[3].plot(plant.index, plant.throttle)
    for _, r in sat_thr.iterrows():
        axes[3].axvspan(r.start, r.end, color="r", alpha=0.2)
    for t0, *_ in tps:
        for ax in axes:
            ax.axvline(t0, color="g", lw=0.8, ls=":")
    for ax, lab in zip(axes, ("alt [ft]", "Vt [ft/s]", "q [deg/s]", "throttle")):
        ax.set_ylabel(lab)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize="small")
    axes[2].legend(fontsize="small")
    axes[-1].set_xlabel("plant time [s]")
    fig.suptitle("Test card log: test points (green), throttle saturation (red), aligned flight-computer log")
    fig.tight_layout()
    save(fig, MODULE, "test_card_log", show=args.show)


def part2(args, out):
    sys.path.insert(0, str(REPO / "modules" / "14_system_id" / "solutions"))
    from flight_test import NAMES, ensure_logs

    ensure_logs()
    src = REPO / "outputs" / "14_system_id"
    frames, truth, t0 = [], [], 0.0
    for name in NAMES:
        df = pd.read_csv(src / f"{name}.csv", index_col="t")
        df.index = df.index + t0
        frames.append(df)
        truth.append((name, t0, df.index[-1]))
        t0 = df.index[-1] + 0.01
    flight = pd.concat(frames)
    print(f"\nPart 2: {len(NAMES)} maneuvers joined into one {flight.index[-1]:.0f} s log; finding them blind:")
    found = activity_windows(flight, ["de", "da", "dr"], window_s=1.0, merge_gap_s=3.0)
    axis_name = {"de": "longitudinal", "da": "lateral", "dr": "lateral"}
    for _, w in found.iterrows():
        actual = next((n for n, a, b in truth if a <= (w.start + w.end) / 2 <= b), "?")
        print(f"  {w.start:6.1f} - {w.end:6.1f} s ({w.duration:4.1f} s)  most active {w.axis} -> "
              f"{axis_name[w.axis]:12s} | actually: {actual}")


def main():
    args = parse_args(__doc__)
    out = outdir(MODULE)
    part1(args, out)
    part2(args, out)


if __name__ == "__main__":
    main()
