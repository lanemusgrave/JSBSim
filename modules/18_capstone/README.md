# Module 18: Capstone, from design model to flight readiness

**Time:** about 10 h · **Reading:** your own Modules 12–17 notes; skim a real flight-readiness review checklist (e.g. NASA's *Flight Readiness Review* entry in NPR 7123.1, or your company's FRR template)

## The brief

The first `gnc_trainer` airframe has been built. Before its first autonomous flight, the
program wants a **flight-readiness recommendation** for the mission below, with evidence.

- **Mission:** a 600 m box at 25 m/s: climb 330 → 430 ft on the first leg, descend to 360 ft on the
  third, return to the start and loiter. Forecast wind 5 m/s from 270°, light
  turbulence (W20 5–15 ft/s).
- **Stack:** waypoint guidance (10 Hz) → XML course/roll/yaw-damper loops on the sensor
  models; Python longitudinal autopilot (40 Hz) on attitude-EKF and altitude-KF
  estimates from noisy, biased sensors. Realistic actuators. All of it is in
  [`gnclab/mission.py`](../../src/gnclab/mission.py).
- **Requirements** (`MISSION_REQUIREMENTS`):

| ID | Requirement |
|---|---|
| MR-1 | all waypoints reached within the mission time |
| MR-2 | \|cross-track\| ≤ 30 m on every leg after a 15 s capture |
| MR-3 | mean \|cross-track\| ≤ 8 m after capture |
| MR-4 | \|altitude error\| ≤ 40 ft once 30 s past a command change |
| MR-5 | airspeed never below 18 m/s |
| MR-6 | \|bank\| ≤ 35° |
| MR-7 | pitch-estimate error RMS ≤ 2° (navigation health) |

- **The catch:** the airplane that was built is not the design model. Its differences are
  in [`solutions/as_built.py`](solutions/as_built.py). **Don't open it.** You get
  to fly it, in a flight test, and find out.

## What to do

Work in [`exercises/capstone_starter.py`](exercises/capstone_starter.py). Each step reuses a module:

| Step | Do | Uses |
|---|---|---|
| 1. Design check | fly the mission on the design model in the forecast weather; fix what fails | 12, 15 |
| 2. Flight test | fly 3-2-1-1s on the as-built airplane; identify Cm, Cl, Cn derivatives | 14 |
| 3. Model update | re-centre the uncertainty model on the identified values, with honest σ | 14, 15 |
| 4. Monte Carlo | the mission on the design model and on the updated model; compare with the first flight | 15 |
| 5. Memo | a flight-readiness memo ([`memo_template.md`](memo_template.md)) | all |

The as-built airplane is flown with `fly_mission(dict(case, **AS_BUILT))`. The "first
flight" is that, in the forecast weather. In real life you'd get one flight; here it's
three seeds, so you can see the scatter.

---

## Reference solution: [`solutions/capstone.py`](solutions/capstone.py)

About 2 minutes (`--fast`: 30 s). Writes `outputs/18_capstone/flight_readiness_memo.md`,
two Monte Carlo CSVs and the plots.

### 1. Design check

```text
line  : max |xtrack|  36.0 m, mean  6.3 m, done at 109.3 s -> FAIL MR-2
fillet: max |xtrack|  15.4 m, mean  7.6 m, done at  92.6 s -> PASS
```

The first finding comes before any flight. Switching legs at the waypoint overshoots
every 90° corner by about a turn radius. Fillets from Module 12 fix it, and they're the configuration flown from here on.

### 2. Flight test and identification

```text
            design  identified  scale  sigma (EE accuracy)     (actual, revealed at the debrief)
Cma         -2.740      -2.593  0.946                 0.05      1.00
Cmq        -38.210     -23.193  0.607                 0.20      0.80   <- EE biased low, as in Module 14
Cmde        -0.990      -0.721  0.728                 0.08      0.80
Clp         -0.510      -0.441  0.865                 0.05      0.90
Clda         0.170       0.125  0.733                 0.05      0.75
Cnb          0.073       0.083  1.138                 0.05      1.20
Cnr         -0.095      -0.083  0.879                 0.10      1.00
```

The flight test found the big surprises: elevator power −27%, aileron power −27%,
a bigger fin. It also inherited Module 14's known biases (Cmq, Cnr). So the
σ used for each identified derivative is **the accuracy equation error achieved in
Module 14**, not the OLS standard error, which is optimistic.

### 3–4. Monte Carlo and first flight

```text
req    design model   updated
MR-3            48%       78%   mean |cross-track| <= 8 m after capture
MR-6            82%      100%   |bank| <= 35 deg
(all others 100 % in both)

metric                   flight    design-model MC         updated-model MC
xtrack_max_m              17.00   median  17.85 (p 40)   median  15.83 (p 77)
elevator_rms_deg           2.83   median   2.42 (p 78)   median   2.97 (p 32)
theta_est_err_rms_deg      1.34   median   1.44 (p 10)   median   1.40 (p 25)
```

- **System ID retired a risk.** With handbook uncertainty, 18% of missions bank past 35°.
  The drivers are low roll damping and *high* aileron power (C<sub>lp</sub> −0.52,
  C<sub>lδa</sub> +0.33). The as-built airplane has 27% *less* aileron power, so on the updated model
  every mission stays below 35° (worst 34.1°). Uncertainty you can't reduce has to be covered by margin; uncertainty you
  *can* reduce, with an afternoon of 3-2-1-1s, should be.
- **The updated model predicts the flight better.** The clearest case is elevator
  activity: the flight sits at p78 of the design-model prediction and at the
  middle (p32) of the updated one. A weaker elevator needs more deflection, which
  is exactly what the identification said.
- **MR-3 is the open item.** Mean cross-track sits right at the 8 m limit: 78% pass
  on the updated model, and the flight's 7.8 m is a pass with no margin. Is it a
  design problem (tune k<sub>path</sub>, χ<sub>∞</sub>, the course loop) or a requirement written without
  turbulence in mind (Module 15)? That's what the memo has to answer. With
  the rule in the generated memo, the recommendation is **NO-GO until MR-3 is
  closed**: the honest answer, and the realistic one.

See `outputs/18_capstone/prediction_vs_flight.png` after you run it: the histograms are the two
predictions, and the black lines are the three first-flight runs.

---

## Deliverable: the memo

Start from the generated `outputs/18_capstone/flight_readiness_memo.md` or the
blank [`memo_template.md`](memo_template.md). Write the judgment sections:
recommendation and rationale, findings, operating limits, and what the analysis
does *not* cover. Then close MR-3, either way, and re-run.

## Self-check

- Which numbers in your memo would change if the first flight were in 10 m/s of wind? Which
  analysis would you re-run, and how long would it take?
- Your updated model passes 100% on MR-6, but the design model didn't. Would you still brief MR-6? How?
- The identification said Cmq scale 0.61; the truth was 0.80. Did that error matter for any
  requirement? How would you know without the answer key?
- A reviewer asks "how do you know the simulation is right?" List your evidence (Modules 07, 11, 14, 16).

## Done when

- [ ] Your mission passes the design check (and you know why the first version didn't)
- [ ] Your identified derivatives and their σ are defended, including the ones you didn't trust
- [ ] Your memo has a recommendation, evidence, limits and open items, and closes MR-3
- [ ] You can brief it in 5 minutes, and answer "what would make you change your mind?"
