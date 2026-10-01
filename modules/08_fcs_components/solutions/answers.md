# Module 08: answers

**A. Actuator characterization.** Large step: rate limit 2.0 /s (the slope),
travel ±0.8, delay 0.050 s (time before the first motion). Small step (0.02,
far below the rate limit): 63% rise 0.033 s after the delay, so the lag
bandwidth is about 30 rad/s. You need both because **the two effects hide each other**.
A large step is rate-limited the whole way, so you never see the lag. A small
step never reaches the rate limit. Real actuator acceptance tests use
small-amplitude frequency sweeps (bandwidth/phase) **and** large-amplitude
steps (rate, travel, saturation recovery) for exactly this reason.

**B. Nonlinear stability limit.** Sustained oscillation from Kq ≈ 0.90–0.95,
slightly below the linear 0.97. The sim has JSBSim's one-frame input delay (8 ms)
plus the actuator's rate limit on top of the 0.1 s you added, and the
3rd-order Padé is an approximation. Once unstable, the oscillation doesn't grow
forever: it saturates in a **limit cycle** bounded by the damper's ±0.5 authority
clip and the elevator travel. Limit cycles like this one, sustained by
saturation and delay, are what pilot-induced and autopilot-induced oscillations look like
in flight test.

**C. Gyro noise filter.** A 20 rad/s first-order filter on q roughly halves the
RMS elevator activity caused by gyro noise (with the 0.0025 rad/s noise in
`gnc_sensors.xml`, the activity is small either way). It adds about
atan(ω/20) of phase lag at frequency ω: about 27° at the 10 rad/s short-period
frequency. So the damping you get at Kq = 0.5 drops, and the delay-induced
instability gain moves lower. The rule of thumb is to keep noise filters at
least 3–5× above the loop crossover, or include them in the design model from
the start.

**Self-check.**
- `<delay>` only works on actuator, sensor and switch in JSBSim 1.3.1. On other
  components it is parsed and silently ignored.
- Deadband subtracts: an input of 1.0 through a 0.2-wide deadband gives 0.9.
  Hysteresis leaves an offset after a reversal.
- A rate limit acts like a frequency-dependent gain reduction **plus phase
  lag** (the triangle in the sine plot lags the input). That is the mechanism of
  rate-limited PIO.
