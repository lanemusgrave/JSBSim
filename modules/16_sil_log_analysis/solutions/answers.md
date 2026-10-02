# Module 16: answers

**A. Average vs worst-case latency** ([`sil_ex.py`](sil_ex.py)). With a constant
20 ms compute time, every command makes its frame boundary (age 1, the same as
a 1 ms controller). With 0–40 ms jitter, the mean is the same but about half the frames
miss the boundary (age 2, sometimes 3), and the loop behaves like the lockstep
run with **two** frames of delay (q RMS 4.09 vs 4.06°/s). A constant
30 ms is worse still. The loop sees the **deadline-miss rate and the worst case**,
not the mean. That's why flight software is specified and tested on
worst-case execution time (WCET) and frame overruns, and why a "fast on
average" processor with OS hiccups can be worse than a slow deterministic one.

**B. Airspeed-error intervals.**
```python
plant = pd.read_csv("outputs/16_sil_log_analysis/plant_log.csv", index_col="t")
iv = intervals((plant.vt_fps - plant.vt_cmd_fps).abs() > 3.0, min_duration=1.0)
```
On the 70 s card (W20 = 8 ft/s) this finds five intervals:

| interval [s] | duration | cause (from `steps` on the command channels) |
|---|---|---|
| 3.2–9.0 | 5.8 s | +100 ft climb: airspeed traded for altitude until the throttle catches up |
| 10.5–16.2 | 5.7 s | leveling off: the excess energy comes back as airspeed |
| 30.0–32.9 | 2.8 s | the +10 ft/s step itself (expected: the error *starts* at 10 ft/s) |
| 51.2–56.8 | 5.6 s | −100 ft descent: the airplane speeds up |
| 58.4–66.3 | 7.9 s | after the descent: slow energy recovery |

None violates R-L2. R-L2 is about the speed step (settled in 3.6 s ≤ 20 s), but
four of the five are **altitude-speed coupling**. That is what a TECS
(Module 12, exercise C) is designed to remove. The point is to attribute each
interval to its test point instead of eyeballing the plot.

**C. Lateral loops in the flight computer.** The work is in the plumbing: extend
the `FMT` entries, port `gnc_autopilot.xml`'s roll PD, course PI (with the
atan2 wrap!) and washout yaw damper, switch the XML lateral loops off in
`_plant_fdm`, then **validate in lockstep against the XML** before anything else.
Expect a one-frame timing difference at step edges, as in Module 11.
The washout filter must be discretized at 40 Hz, not 120 Hz.

## Self-check

- **Proving the rig:** in lockstep with no added delay, the external controller
  receives exactly the in-process measurements and its command is applied at
  exactly the same frame, so the trajectories must match bit-for-bit. A
  one-frame mismatch means the command is applied a frame late (or early),
  usually an off-by-one in the queue or in when SENS is sampled.
- **Built-in frame:** the plant sends SENS k and immediately steps without
  waiting. To remove it, wait for CMD k up to a budget (e.g. 10 ms) before
  stepping. That's "lockstep with a timeout": it costs real-time margin and needs a
  policy for late commands (hold the last one, and count the miss).
- **Two 20 ms computers:** see A.
- **Clock alignment:** correlate a high-bandwidth signal recorded by both
  (rates, surface commands, accelerations). Check the correlation peak (near 1)
  and overlay the aligned signals: residuals should be at the noise and
  interpolation level. If the clocks also *drift*, align separate segments and fit
  offset + rate.
- **Why "scale both" failed:** scaling kp lowered the pitch loop's stiffness,
  so the altitude loop overshot (less separation between inner and outer loop). The
  delay's phase loss ωτ is largest at high frequency, where the kd·q path
  dominates. Cutting kd alone recovers most of the margin at little cost in
  low-frequency stiffness.
