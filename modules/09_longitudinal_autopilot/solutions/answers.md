# Module 09: answers

**Off-design exercise.** Representative results (100 ft altitude step):

| Va | gains | overshoot | settle (10 ft) | max \|δe_ap\| |
|---|---|---|---|---|
| 18 m/s | fixed (25 m/s) | 8.9 % | 15.5 s | 0.97 |
| 18 m/s | re-designed | 6.5 % | 13.5 s | 0.96 |
| 25 m/s | design point | 5.0 % | 6.3 s | 0.70 |
| 32 m/s | fixed | 1.2 % | 4.9 s | 0.47 |

- **Low speed suffers most.** Elevator power scales with q̄ ∝ V², so at 18 m/s
  each unit of elevator buys about half the pitching moment it does at 25 m/s. The pitch
  loop is slower and the climb uses nearly **all** the elevator. Re-designing helps the
  altitude loop a little, but the real constraint is *authority*: the autopilot
  is close to saturation. A requirement of "no saturation at V<sub>min</sub>" would
  force a lower θ<sub>max</sub> or a softer climb at low speed.
- At high speed the fixed gains are effectively *higher* relative to the plant,
  and the loop is faster and better damped; watch the delay margin instead.
- This is the motivation for **gain scheduling on q̄** (Module 11).

**Self-check.**
- *Why must the inner loop be about 10× faster than the outer?* So the outer loop can
  treat it as a (near) unity gain; then the two designs decouple.
- *Why did the textbook DC-gain formula mislead us?* The 2nd-order pitch
  approximation ignores the speed and phugoid coupling, and the true DC gain was
  0.73 vs 0.38 predicted. Always check the design on the full model.
- *Why does altitude-hold need a wing leveler on this airframe?* It's spiral
  unstable. Bank grows, lift tilts, and no pitch command holds altitude in a
  60° bank.
- *Where does the noise on θ<sub>c</sub> come from?* Baro noise (2 ft) × k<sub>p,h</sub> (0.0103 rad/ft)
  ≈ 1.2°. Fixes: filter the altitude feedback, blend with vertical speed from
  the IMU/GPS (complementary filter, Module 13), or lower k<sub>p,h</sub>.
