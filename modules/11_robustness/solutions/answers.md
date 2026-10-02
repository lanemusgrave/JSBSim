# Module 11: answers

**A. Back-calculation.** 400 ft climb overshoot: none 237 ft, clamping 7 ft,
back-calculation −0.9 ft. Back-calculation keeps the integrator consistent
with the *saturated* output, so when the error shrinks the PI output is already
at the right value; clamping stops integration but keeps whatever was stored.
Both are standard. Clamping is easier to verify, which matters for certification.

**B. Latency budget at 25 Hz.** One frame (40 ms) of output latency is
acceptable; two frames (80 ms) limit-cycle. Including the half-frame
sample-and-hold, the effective delay limit is about 60–100 ms, consistent with
the 89 ms linear delay margin (which doesn't include sensor lag). A latency
budget written as a requirement: "Sensor sample to surface command ≤ 40 ms at a
≥ 25 Hz frame rate (pitch loop delay margin ≥ 50% of total delay)".

**C. Schedule in XML.** An `<fcs_function name="ap/sched">` computing
min(2, max(0.5, 7.92 / aero/qbar-psf)) and two `pure_gain`s multiplying the
design kp, kd by it. Compare with the C172's `scheduled_gain` component (table
lookup vs altitude), which is another way to write the same idea.

**Self-check.**
- *Why does a lower frame rate act like delay?* A zero-order hold delays the
  signal by T/2 on average (phase lag ωT/2).
- *Why can't you filter noise for free?* A first-order filter at ω<sub>f</sub> costs
  atan(ω<sub>c</sub>/ω<sub>f</sub>) of phase at crossover ω<sub>c</sub>. Here a 5 Hz filter costs 26° and
  27 ms of delay margin. Cheap when you have margin, expensive when you don't.
- *What's the risk of not scheduling?* At 32 m/s the fixed-gain pitch loop's
  delay margin falls from 89 to 55 ms; at low speed it's sluggish and closer to
  saturation.
