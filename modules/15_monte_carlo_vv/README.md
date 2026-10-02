# Module 15: V&V with Monte Carlo and requirements testing

**Time:** about 6 h · **Reading:** NASA-STD-7009A *Standard for Models and Simulations* (skim §4 on credibility); Hanson & Beard, "Applying Monte Carlo Simulation to Launch Vehicle Design and Requirements Verification" (NASA, *J. Spacecraft & Rockets* 2012), which is the clearest short treatment of pass-rate statistics; MIL-F-8785C §3.7 (turbulence model, for what `atmosphere/turbulence/milspec/*` means)

## Objectives

1. Turn the Module 09/10 requirements into **testable, disturbance-aware** criteria.
2. **Disperse** what you don't know (mass and CG, aero derivatives, thrust,
   wind, turbulence, actuators, sensor noise) and run hundreds of flights in
   parallel, in a way that works on Windows.
3. Report **pass rates with confidence intervals**, find what **drives** each
   failure, and measure the **margin to failure**.
4. Write a **flight-readiness summary** that a test director can act on.

Here you stop asking "does it work?" and start asking "how sure are we, and where does it break?"

---

## Refresher

**What a Monte Carlo can claim.** With *n* runs and *k* passes, the pass
probability has a binomial confidence interval (Clopper–Pearson is exact). The
headline rule: **zero failures in n runs gives P(fail) < 3/n at 95%** (the
"rule of three"). 300 clean runs support "P(fail) < 1%", not "never fails".

$$
(1-p)^n \le 1 - C \;\Rightarrow\; n \ge \frac{\ln(1-C)}{\ln(1-p)} \approx \frac{3}{p}\quad(C=95\%)
$$

**Dispersions vs scenarios.** Dispersions are things you can't control or don't
know (aero uncertainty, payload, wind). Scenarios are what you *test* (the
altitude step and the course change). Run every scenario across the
dispersions, and always fly the **nominal case first**.

**Sensitivity.** The rank correlation (Spearman) between each dispersion and each
metric tells you which uncertainty matters. That tells you where to spend wind-tunnel
or system-ID money (Module 14), and where you need a margin.

## What was added for this module

- **The model** ([generator](../07_build_uas_model/solutions/build_gnc_trainer.py)):
  `uncertainty/<coef>-scale` multipliers (default 1) on CL, CD, Cmα, Cmq, Cmδe,
  Clβ, Clp, Clδa, Cnβ, Cnr and thrust. The actuator `<lag>` and `<rate_limit>`
  now read the properties `fcs/actuator-bw-rad_sec` and `fcs/actuator-rate-rad_sec`
  (JSBSim accepts a property name wherever it accepts a number in those
  elements). `gnc_sensors.xml` gained `sensors/gyro-bias-{p,q,r}-rad_sec`. All
  default to nominal, so the model is unchanged and every earlier test still passes.
- **[`src/gnclab/montecarlo.py`](../../src/gnclab/montecarlo.py)**:
  - Sampling: `Normal`/`Uniform` dispersions, `sample_cases` (reproducible; case 0 is nominal) and `apply_dispersions`.
  - The test card and requirements: `requirements_flight` and `REQUIREMENTS`.
  - Execution: `run_cases`, a spawn-based process pool.
  - Analysis: `evaluate`, `pass_rate_ci`, `runs_for_confidence`, `sensitivity` and `unusual`.

### The test card and requirements

One 65 s flight per case, with realistic actuators and sensors and the full
autopilot: a +100 ft altitude step at 2 s, then a +90° course change at 35 s.
Metrics are judged on **truth**.

| ID | Requirement | Origin |
|---|---|---|
| MC-1 | altitude step overshoot ≤ 15 ft | R-L1 |
| MC-2 | altitude within ±10 ft by 30 s | R-L1 |
| MC-3 | mean altitude error over 28–33 s ≤ 5 ft | R-L1 |
| MC-4 | course overshoot ≤ 10° | R-A2 |
| MC-5 | course within ±5° over the last 8 s | R-A2 |
| MC-6 | \|β\| ≤ 5° in the turn | R-A3 |
| MC-7 | altitude within ±30 ft in the turn | R-A4 |
| MC-8 | elevator command saturated ≤ 0.5 s in total | R-L4 |
| MC-9 | airspeed never below 18 m/s | stall margin |

The R-L/R-A limits from Modules 09–10 were written for calm air
("final error < 2 ft"). In turbulence they have to be restated: average over
a window, or bound a statistic. That restating is the job, not a fudge.

> 🐛 **Gotchas found while building this module**
> - **Turbulence silently off:** `atmosphere/turbulence/milspec/severity` must be
>   non-zero even at low altitude, where it isn't used. An index of 0 disables
>   the whole model, with no warning. `turbulence_on()` sets it.
> - **Sensor and turbulence seeds:** `simulation/randomseed` seeds the sensor noise;
>   `atmosphere/randomseed` seeds the turbulence. Set both per case, or your
>   "300 runs" contain 300 copies of the same noise.
> - **Spawned workers re-import your script** (always on Windows, and here on
>   every OS on purpose). Code outside `if __name__ == "__main__":` runs once
>   per worker, and a script piped in on stdin or typed at a REPL can't be
>   re-imported at all (`BrokenProcessPool`). Use `workers=1` there.

---

## Walkthrough

### 1. The Monte Carlo: [`solutions/monte_carlo.py`](solutions/monte_carlo.py)

300 cases in about 18 s on 3 workers (JSBSim flies this 65 s card in about 0.2 s):

```text
req        pass    rate          95 % CI     worst   limit  description
MC-1   300/300   100.0%  [ 98.8, 100.0] %     13.36    15.0  altitude step overshoot <= 15 ft (R-L1)
MC-2   300/300   100.0%  [ 98.8, 100.0] %     28.10    30.0  altitude within +/-10 ft by 30 s (R-L1)
MC-3   295/300    98.3%  [ 96.2,  99.5] %      6.46     5.0  mean altitude error over 28-33 s <= 5 ft (R-L1)
MC-4   300/300   100.0%  [ 98.8, 100.0] %      5.29    10.0  course overshoot <= 10 deg (R-A2)
 ...
ALL    295/300    98.3%  [ 96.2,  99.5] %

Top drivers (Spearman rank correlation with the metric):
  MC-3 alt_err_end_ft     turb_w20_fps +0.49  scale_Clb +0.16  scale_thrust -0.16
  MC-4 chi_overshoot_deg  scale_Clda -0.72  scale_Clp +0.60  turb_w20_fps +0.16
  MC-6 beta_max_deg       turb_w20_fps +0.55  scale_Cnb -0.53  scale_Cnr +0.28
  MC-9 vt_min_mps         scale_thrust +0.57  turb_w20_fps -0.57  payload_kg -0.24

throttle hit 100 % during the climb in 70 % of runs
```

How to read it:

- **All five failures are MC-3, and all are in turbulence with W20 > 16 ft/s**
  (`drivers.png`, left). The autopilot is fine; the requirement doesn't say what
  happens in turbulence. Exercise A settles it.
- **The drivers make physical sense**, which is your check on the simulation.
  Course overshoot is driven by **aileron power and roll damping**, the roll-mode
  time constant τ<sub>r</sub> ∝ C<sub>lp</sub>/C<sub>lδa</sub> from Module 10's design model.
  Sideslip is driven by turbulence and **C<sub>nβ</sub>**. If a "driver" made no
  physical sense, you'd suspect a bug in the dispersion plumbing.
- **Look beyond pass/fail.** The throttle saturates in 70% of climbs. No
  requirement fails, but a 100 ft step at 25 m/s is *thrust-limited*. Bigger
  steps or weaker motors will trade airspeed for altitude (see MC-9's
  driver). That's a finding for the summary.
- `worst` is the number to brief, not the mean. MC-2's worst case is 28.1 s
  against a 30 s limit: passing, but barely.

### 2. Margin to failure: [`solutions/margin_to_failure.py`](solutions/margin_to_failure.py)

A Monte Carlo samples the likely; a sweep finds the cliff. One parameter at a time, all others nominal, calm air:

```text
parameter       nominal  first failure   sigma  failed requirements
scale_Clda        1.000          0.300     4.7  MC-4
scale_Cnb         1.000          0.200     4.0  MC-6
scale_Cmde        1.000          0.450     3.7  MC-7,MC-8
payload_x_m       0.400          1.300       -  MC-1,MC-8,MC-9   (2 kg payload)
scale_thrust      1.000  none in sweep (to 0.3)
turb_w20_fps      0.000         70.000       -  MC-6
act_rate_dps    250.000          5.000       -  MC-4,MC-5,MC-7,MC-8
```

- Every cliff sits at **3.7σ or more**, so the design is robust to each uncertainty
  *alone*. Elevator power has the least margin.
- **Payload aft:** 2 kg at x = 1.0 m puts the CG near the estimated neutral point
  (static margin ≈ −C<sub>mα</sub>/C<sub>Lα</sub> ≈ 0.49 c̄). The airplane is statically **unstable**
  from there to 1.3 m, and **the pitch loop hides it**. It fails only when the
  elevator saturates. An autopilot that stabilizes an unstable airframe is fine
  until it saturates or loses a sensor. That's a weight-and-balance limit
  for the operating handbook.
- **Thrust:** the B&M motor has huge excess power, so even 30% thrust still trims
  (throttle 0.997). The climb then sags to 18.6 m/s, right at MC-9's limit.
- **Actuator rate** doesn't matter down to 8°/s for these small maneuvers. That
  would change in turbulence, or with larger commands.

### 3. Stratify the condition: [`solutions/mc_ex.py`](solutions/mc_ex.py) (exercise A)

```text
 W20 [ft/s]   MC-3 pass          95 % CI   all reqs  |alt err| p95 [ft]
       0-10      100.0%  [ 94.0, 100.0] %      98.3%                1.47
      10-15      100.0%  [ 94.0, 100.0] %     100.0%                2.60
      15-20       96.7%  [ 88.5,  99.6] %      96.7%                4.65
      20-25       98.3%  [ 91.1, 100.0] %      98.3%                3.99
      25-35       86.7%  [ 75.4,  94.1] %      80.0%                6.95

Failures in turbulence below 15 ft/s (1):
  case 43 [MC-8] W20 3.4: scale_CL -0.11 (-2.3 sig), scale_Cmde -0.34 (-2.3 sig), payload_kg 1.3 (87 % of range)
```

Two lessons:

1. MC-3 holds in W20 ≤ 15 ft/s. Above that it needs a separate, statistical
   criterion (e.g. "95th percentile of |error| ≤ 7 ft in W20 ≤ 25 ft/s").
2. **The stratified run found a calm-air failure that the 300-run Monte Carlo
   missed.** Case 43 saturates the elevator for 1.8 s, from a *combination* of
   −2.3σ elevator power, −2.3σ lift and a heavy payload. Each one alone passes
   easily (the elevator-power cliff alone is at 0.45, not 0.66).
   One-at-a-time sweeps can't find this; sampling can. You need both.

---

## Exercises ([`exercises/mc_ex.py`](exercises/mc_ex.py))

- **A. Stratified Monte Carlo** on turbulence; rewrite MC-3 with conditions.
  Solution above.
- **B. How many runs** for "P(fail) < 0.5% at 95%"? (598 with zero failures; 947 if one fails.)
- **C. Gyro bias** margin to failure. Solution in `mc_ex.py`: p and r biases have
  *no* effect and q fails only at 0.7 rad/s. [`answers.md`](solutions/answers.md)
  explains why that result should make you *distrust* the sensor model.

## Self-check

- 300 runs, 0 failures. What can you claim, and what can't you?
- Why fly the nominal case first? Why always seed every random source per case?
- A driver ranks top for a metric but makes no physical sense. What do you suspect?
- What does a one-at-a-time sweep miss that random sampling catches, and the other way round?
- When is a requirement failure a reason to change the requirement rather than the design?
  Who has to agree?

## Done when

- [ ] You can explain every row of the requirement table, including `worst`
- [ ] You can explain case 43 to a colleague, and say which test you'd add to catch it earlier
- [ ] You've written a one-page readiness summary from `outputs/15_monte_carlo_vv/summary.md`:
  scope, pass rates with intervals, failures and their drivers, margins,
  operating limits (CG, turbulence), and open items
