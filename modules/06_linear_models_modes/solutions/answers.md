# Module 06: answers

**Exercise 1 (T-38).** Representative output of `t38_modes_ex.py`:

| | low/slow | low/fast | high/slow | high/fast |
|---|---|---|---|---|
| KTAS | 215 | 427 | 293 | 471 |
| SP ω [rad/s] | 1.29 | 2.41 | 1.20 | 1.83 |
| DR ζ | 0.166 | 0.195 | 0.128 | 0.149 |

- (a) Doubling KCAS at 5,000 ft (q̄ ×4) raises ω<sub>sp</sub> ×1.87, close to the ×2
  you'd expect from ω ∝ √q̄ ∝ V. At the same KCAS (about the same q̄), ω<sub>sp</sub> is about the
  same at 5k and 25k: it is *dynamic pressure* that sets the aerodynamic
  stiffness. Damping falls with altitude (lower density means less aerodynamic
  damping relative to inertia).
- (b) √2 π V/g gives 50 vs 55 s, 100 vs 118 s and so on, within 10–20%.
- (c) high/slow has ζ<sub>DR</sub> ≈ 0.13, below the 0.19 Level 1 value, so a yaw damper
  is justified. (Note that the bundled T-38 is an approximate model; its short
  period is softer than the real airplane's. Don't use it for real T-38 numbers.)

**Exercise 2 (spiral).** With the state order [β, φ, p, r], L<sub>β</sub> = A[p, β],
L<sub>r</sub> = A[p, r], N<sub>β</sub> = A[r, β], N<sub>r</sub> = A[r, r]. The product L<sub>β</sub>N<sub>r</sub> − N<sub>β</sub>L<sub>r</sub>
is positive (spiral stable) for C<sub>lβ</sub> = −0.10 and negative for −0.06. Dimensional
derivatives keep the same sign structure as the coefficients, scaled by
q̄Sb/I.

**Exercise 3.** θ/δ<sub>e</sub> rolls off past the short-period frequency (≈10 rad/s
for the glider) with phase heading to −180° and beyond. φ/δ<sub>a</sub> is about an
integrator times a first-order lag (−90° at low frequency, −180° well above
1/τ<sub>roll</sub>). Both are easy plants for proportional loops, until you add
actuator lag and delay (Module 11).

**Self-check.**
- Linearization is a Taylor series about f(x₀, u₀) = 0. If the point isn't an
  equilibrium, the zeroth-order term doesn't vanish and the "linear model"
  describes a point that's accelerating away.
- A[Q, Alpha] = M<sub>α</sub>. It must be negative.
- Sluggish roll response: poor roll-mode time constant (MIL-F-8785C wants
  τ<sub>R</sub> ≤ 1.0–1.4 s depending on class/category). Pilots call it "slow to roll in,
  hard to stop".
- Input timing / delay: one frame, a zero-order hold, actuator lag, or
  hysteresis. Then the trim point.
