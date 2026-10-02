# Module 15: answers

**A. Stratified Monte Carlo** ([`mc_ex.py`](mc_ex.py)). MC-3 passes 100% for
W20 ≤ 15 ft/s (95% CI lower bound 94% with 60 runs per bin) and degrades
above that: 96.7%, 98.3%, then 86.7% for 25–35 ft/s. The altitude loop isn't broken;
gusts move the airplane, and a 5 s mean error of 5 ft is simply not achievable in
moderate turbulence with a 0.05 Hz altitude loop. A sensible rewrite:

> **MC-3a** In W20 ≤ 15 ft/s, the mean altitude error over any 5 s window after
> settling shall be ≤ 5 ft.
> **MC-3b** In W20 ≤ 25 ft/s, the 95th percentile of the 5 s mean altitude
> error across the dispersion set shall be ≤ 7 ft.

Requirements belong to whoever owns the mission (systems engineering and the
customer), not to GNC. GNC brings the data: "this is what the design achieves
vs turbulence, and this is what it would take to do better" (a higher
altitude-loop bandwidth costs elevator activity and noise; Modules 11 and 13).

The stratified run also surfaced **case 43**: a calm-air MC-8 failure (elevator
saturated for 1.8 s) from −2.3σ Cmδe, −2.3σ CL and a 1.3 kg payload. Less
lift means more α; less elevator power means more elevator per unit of α; and the
heavier airplane needs both. To catch it earlier: (1) add a *corner* test
(all three at −2σ/max together) to the regression; (2) run more cases; or (3)
use importance sampling or an optimizer to search the tails. Fixes include
lowering the pitch-loop gain toward saturation, a θ-command rate limit in the
altitude loop, or a tighter weight-and-balance limit.

**B. Runs needed.** "P(fail) < 0.5% at 95%": with zero failures,
n ≥ ln(0.05)/ln(0.995) = **598**. With one failure, find n where the one-sided
95% Clopper–Pearson upper bound on P(fail) falls below 0.5%: **947 runs**. Each
failure costs a lot of runs, which is why you fix failures rather than
out-sample them. (One-sided 95% equals the upper end of a two-sided 90% interval.)

**C. Gyro bias** (now `sensors/gyro-bias-{p,q,r}-rad_sec` in `gnc_sensors.xml`):

| axis | effect | why |
|---|---|---|
| p | **none** up to 1 rad/s | the roll loop is P-only on φ (kd_phi = 0), so nothing reads p |
| r | **none** up to 1 rad/s | the yaw damper's **washout** blocks DC, so a constant bias never reaches the rudder |
| q | MC-3 fails at **0.7 rad/s (40°/s)** | kd_θ·b is a constant elevator offset; the altitude integrator trims it out, until it can't |

Read this result with suspicion. These margins are huge because **our sensor
model's AHRS attitude is truth plus noise**: a gyro bias never corrupts φ or θ.
In a real AHRS (Module 13) a q bias tilts the θ estimate unless the
accelerometer correction removes it, and *that* error goes straight into the
pitch loop. A Monte Carlo can only find failures its models can produce. That
is NASA-STD-7009's "credibility" point: the summary must state what the
simulation does *not* model (here: AHRS dynamics, GPS dropouts, actuator
failures, structural modes, propwash, ground effect).

## Self-check

- **300/300:** P(fail) < 1% at 95% confidence, *for these dispersions, this
  test card and these models*. Not "it never fails", and not "it works for
  conditions you didn't disperse".
- **Nominal first:** if the nominal case fails, the dispersions tell you
  nothing. **Seeds:** without per-case seeds, every case sees the same noise
  and turbulence; with them, any failure can be re-flown exactly.
- **A driver that makes no physical sense:** suspect the plumbing. A property
  name typo (JSBSim creates a new, unused property silently), a multiplier on
  the wrong term, or a units error.
- **Sweeps vs sampling:** sweeps find the cliff for one parameter but miss
  combinations (case 43). Sampling finds combinations but rarely reaches 4σ
  tails. Use both, plus targeted corner cases.
- **Change the requirement** when it doesn't state its conditions, or demands something
  physics won't allow (MC-3 in heavy turbulence). Change the design when it
  fails inside the conditions that matter. Either way, the requirement owner signs off.
