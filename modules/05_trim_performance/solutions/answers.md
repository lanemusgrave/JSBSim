# Module 05: answers

**Exercise 1 (T-38).** See `t38_trim_ex.py`. Max level speed at 15,000 ft is about
452 KCAS (M 0.88) at full throttle. Drag is lowest around 175–185 KCAS (α ≈ 12–13°).
Below that you're on the back side of the drag curve: slowing down takes *more*
thrust, so speed becomes unstable for a pitch-for-path pilot. That's why power
approaches fly path with pitch and speed with throttle (or use an auto-throttle).

**Exercise 2 (turn trim, sketch).** In a steady coordinated level turn,
ψ̇ = g tan φ / V and the body rates are p = −ψ̇ sin θ, q = ψ̇ sin φ cos θ,
r = ψ̇ cos φ cos θ. Unknowns: α, pitch trim, and (for a glider) γ. φ is
given. Residuals: u̇, ẇ, q̇. For a full solution also solve β, aileron and rudder
against v̇, ṗ, ṙ. Write the rates into `ic/p,q,r-rad_sec` before each `run_ic()`.
JSBSim's mode 5 does exactly this; use it to check your answer on the C172.

**Exercise 3.** A 2 g pull-up and a 60° bank turn (n = 1/cos 60° = 2) at the same
speed and weight both need C<sub>L</sub> = 2W/(q̄S), so trim α is nearly the same. The
small difference comes from the different pitch rates (q = g(n−1)/V in the
pull-up vs. q = ψ̇ sin φ ≈ g tan φ sin φ / V in the turn) acting through C<sub>Lq</sub>
and C<sub>mq</sub>, and from power effects. Same n means same α: the rule behind
"g-limits are α-limits" in fighters.

**Exercise 4.** At full throttle, sweep KCAS from 60 to 90 and, at each speed,
increase `gamma-deg` until the trim fails. Rate of climb = V sin γ<sub>max</sub>, and it
peaks near the bundled model's V<sub>y</sub> (around 70–75 KCAS at 4000 ft).

**Self-check.**
- Linearization is a Taylor expansion of f(x, u) around a point where
  f = 0. Away from equilibrium the "constant" term doesn't vanish and the
  model drifts.
- Add a gamma fallback (trim u̇ with γ), or write a custom solver
  (`gnclab.trim.trim_custom`).
- Stable: more nose-down trim at higher speed. (dδ<sub>e</sub>/dV > 0 in the model's
  sign convention, meaning trailing edge down.)
- A bigger tail moves the neutral point aft, so static margin grows. Burning
  fuel from an aft tank moves the CG forward, so static margin also grows.
  The reverse (forward tank burn) erodes it.
