# Equations cheat sheet

The few equations the curriculum leans on, with the module where each is used.
Conventions: body axes x fwd, y right, z down; angles in rad; c̄ chord, b span, S wing area.

## Kinematics and frames (02)

- α = atan2(w, u), β = asin(v / V), V = √(u² + v² + w²) (air-relative velocities)
- γ = θ − α (wings level, β = 0); χ = ψ + crab angle; crab ≈ asin(V<sub>w,⊥</sub>/V<sub>a</sub>)
- Euler rates: φ̇ = p + (q sinφ + r cosφ) tanθ, θ̇ = q cosφ − r sinφ, ψ̇ = (q sinφ + r cosφ)/cosθ
- Coordinated turn: φ = atan(V χ̇ / g) = atan(V<sub>g</sub>² / (g R)); load factor n = 1/cosφ

## Forces and moments (04, 07)

- Lift = q̄ S C<sub>L</sub>, q̄ = ½ ρ V²; drag polar C<sub>D</sub> = C<sub>D0</sub> + C<sub>L</sub>²/(π e AR)
- C<sub>m</sub> = C<sub>m0</sub> + C<sub>mα</sub> α + C<sub>mq</sub> (c̄/2V) q + C<sub>mδe</sub> δ<sub>e</sub>; moment = q̄ S c̄ C<sub>m</sub>
- Static margin = (x<sub>NP</sub> − x<sub>CG</sub>)/c̄ ≈ −C<sub>mα</sub>/C<sub>Lα</sub> (about the CG)
- Accelerometer = specific force = (total force − gravity)/m. In steady flight ≈ −g in body axes

## Trim and performance (05)

- Level flight: L = W, T = D; power required P = D V; best range at max L/D; best endurance at max C<sub>L</sub><sup>3/2</sup>/C<sub>D</sub> (prop)
- Stall speed V<sub>s</sub> = √(2W / (ρ S C<sub>L,max</sub>))

## Modes (06)

- Short period: ω<sub>n</sub>² ≈ Z<sub>α</sub>M<sub>q</sub>/V − M<sub>α</sub>, 2ζω<sub>n</sub> ≈ −(M<sub>q</sub> + M<sub>α̇</sub> + Z<sub>α</sub>/V)
- Phugoid (Lanchester): ω<sub>n</sub> ≈ √2 g / V, ζ ≈ 1/(√2 L/D)
- Roll subsidence: τ<sub>r</sub> ≈ −1/L<sub>p</sub>; Dutch roll ω<sub>n</sub>² ≈ N<sub>β</sub> + (Y<sub>β</sub>N<sub>r</sub>)/V; spiral stable if L<sub>β</sub>N<sub>r</sub> > L<sub>r</sub>N<sub>β</sub>
- Eigenvalue λ = σ ± jω: ω<sub>n</sub> = |λ|, ζ = −σ/ω<sub>n</sub>, time to half = ln2/(−σ)

## Control (09–11)

- Loop L(s) = C(s)P(s); closed loop T = L/(1 + L); crossover |L(jω<sub>c</sub>)| = 1
- Phase margin PM = 180° + ∠L(jω<sub>c</sub>); **delay margin = PM [rad] / ω<sub>c</sub>** (the most useful margin in practice)
- A pure delay τ costs phase ω τ (grows with frequency); Padé(1): e<sup>−sτ</sup> ≈ (1 − sτ/2)/(1 + sτ/2)
- Successive loop closure: each outer loop ≥ 5–10× slower than the inner one (B&M ch. 6)
- 2nd-order step: overshoot = exp(−πζ/√(1−ζ²)); t<sub>s,2%</sub> ≈ 4/(ζω<sub>n</sub>); t<sub>r</sub> ≈ 1.8/ω<sub>n</sub>
- Discrete: zero-order hold ≈ delay of T/2; a controller at rate f adds ≈ 1/f of compute + T/2

## Guidance (12)

- Straight line: χ<sub>c</sub> = χ<sub>q</sub> − χ<sub>∞</sub> (2/π) atan(k<sub>path</sub> e<sub>py</sub>), e<sub>py</sub> = −sinχ<sub>q</sub>(p<sub>n</sub>−r<sub>n</sub>) + cosχ<sub>q</sub>(p<sub>e</sub>−r<sub>e</sub>)
- Orbit: χ<sub>c</sub> = φ + λ[π/2 + atan(k<sub>orbit</sub> (d − ρ)/ρ)]

## Estimation (13)

- KF predict: x⁻ = A x + B u, P⁻ = A P Aᵀ + Q. Correct: K = P⁻Hᵀ(HP⁻Hᵀ + R)⁻¹, x = x⁻ + K(y − Hx⁻), P = (I − KH)P⁻
- Attitude EKF accel model: f<sub>x</sub> = g sinθ, f<sub>y</sub> = rV − g cosθ sinφ, f<sub>z</sub> = −qV − g cosθ cosφ

## System ID (14)

- Equation error: θ̂ = (XᵀX)⁻¹Xᵀy, cov = s²(XᵀX)⁻¹ (optimistic with coloured residuals)
- Output error: min Σ (y − ŷ(θ))ᵀ R⁻¹ (y − ŷ(θ)) by nonlinear least squares
- FRF H = S<sub>uy</sub>/S<sub>uu</sub>; coherence γ² = |S<sub>uy</sub>|²/(S<sub>uu</sub>S<sub>yy</sub>), trust where γ² > 0.6
- Theil U = rms(y − ŷ)/(rms y + rms ŷ): 0 perfect, < 0.3 good

## Statistics (15)

- Zero failures in n runs → P(fail) < 1 − (1 − C)<sup>1/n</sup> ≈ 3/n at C = 95% ("rule of three")
- Clopper–Pearson interval for k passes in n: Beta quantiles (`gnclab.montecarlo.pass_rate_ci`)
- Boxcar of N samples at dt: delay (N−1)dt/2, nulls at k/(N dt) (17)
