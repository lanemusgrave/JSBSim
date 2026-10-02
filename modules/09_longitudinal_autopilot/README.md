# Module 09: Longitudinal autopilot

**Time:** about 6 h · **Reading:** B&M ch. 5 (transfer-function models) and ch. 6 (successive loop closure), §6.4 longitudinal; any controls text on root locus, Bode and PI/PD design (FPE ch. 5–6, Ogata); RM Case Study 3 for how the C172's altitude hold is built in XML

## Objectives

1. Design pitch-attitude, altitude and airspeed loops by **successive loop
   closure**, using the plant numbers from *your* JSBSim model.
2. Verify the design on the **full** linear model with all loops closed:
   closed-loop poles, steps, and gain, phase and **delay margins** at each loop break.
3. Implement it in JSBSim XML with engage logic, bumpless transfer,
   saturation and **integrator anti-windup**.
4. Verify against written requirements in the nonlinear sim, ideal *and* with
   realistic actuators and sensors.

---

## Refresher: successive loop closure (B&M §6.1)

```text
 h_c ─►(+)─► PI_h ──► θ_c ─►(+)─► PD_θ ──► δe ─► [actuator] ─► airframe ─┬─► θ, q
        ▲−                     ▲−                                         │
        └──────── h ◄──────────┴──────────────── θ ◄──────────────────────┘
 V_c ─►(+)─► PI_V ──► δt ─► motor ─► airframe ─► V          (throttle path)
        ▲−─────────────────────────────────────┘
```

Close the fastest loop first, then treat it as a unity-ish block in the next
loop out, with **bandwidth separation** W = ω<sub>inner</sub>/ω<sub>outer</sub> ≈ 5–15 or more. With
θ/δ<sub>e</sub> ≈ a<sub>θ3</sub>/(s² + a<sub>θ1</sub>s + a<sub>θ2</sub>):

$$
k_{p\theta} = \frac{\delta_{e,max}}{e_{\theta,max}}\operatorname{sign}(a_{\theta3}),\quad
\omega_\theta=\sqrt{a_{\theta2}+k_{p\theta}a_{\theta3}},\quad
k_{d\theta}=\frac{2\zeta_\theta\omega_\theta-a_{\theta1}}{a_{\theta3}},\quad
K_{\theta,DC}=\frac{k_{p\theta}a_{\theta3}}{a_{\theta2}+k_{p\theta}a_{\theta3}}
$$

$$
\text{altitude (}h/\theta \approx V/s\text{):}\quad k_{ih}=\frac{\omega_h^2}{K_{\theta,DC}V},\; k_{ph}=\frac{2\zeta_h\omega_h}{K_{\theta,DC}V}
\qquad
\text{airspeed (}V/\delta_t \approx a_{V2}/(s+a_{V1})\text{):}\quad k_{iV}=\frac{\omega_V^2}{a_{V2}},\; k_{pV}=\frac{2\zeta_V\omega_V-a_{V1}}{a_{V2}}
$$

The first gain is chosen from **saturation**: a 15° pitch error commands full
elevator. That is how actuator limits enter the design from the start.

---

## Walkthrough

### 1. Design: [`solutions/design_longitudinal.py`](solutions/design_longitudinal.py)

```text
plant: a_th1 = 5.07, a_th2 = 95.6, a_th3 = -15.1 (per elevator norm);  a_V1 = 0.29, a_V2 = 30.2
pitch: kp = -3.82, kd = -0.896  ->  w = 12.4 rad/s
       pitch-loop DC gain: 2nd-order approximation 0.376, full 4-state model 0.733   <- !
loop at elevator (inner)    PM 84 deg  wc 16.4 rad/s  delay margin   90 ms
loop at theta_c (altitude)  GM 19.9 dB PM 60 deg  wc 0.66 rad/s  delay margin 1585 ms
loop at throttle (airspeed) PM 76 deg  wc 0.33 rad/s
```

Three design lessons came out of building this, all of them real:

1. **The approximation's DC gain was wrong by 2×** (0.38 vs 0.73). Designing
   the altitude loop on the approximation gave 20° phase margin and 60%
   overshoot. *Always* re-check with the full model; the script uses the real
   DC gain.
2. **You can't judge throttle → airspeed with the altitude loop open.** Open
   loop, more thrust means more *climb*, not more speed (V/δ<sub>t</sub> has almost no DC gain).
   Only with altitude held does throttle control speed. So the check builds the
   **full closed loop in state space** (`closed_loop()`) and breaks one loop at
   a time *with the others closed* to measure margins. That's the standard
   "loop-at-a-time" margin practice.
3. **PI zeros cause overshoot.** No choice of (W<sub>h</sub>, ζ<sub>h</sub>) got the linear altitude
   step under about 15% (see the sweep table below). The fix is
   operational rather than just gains: a capture zone with the integrator frozen outside
   it, plus a θ-command limit, so large altitude changes are flown as climbs.

| W_h, ζ_h | PM | linear OS |
|---|---|---|
| 25, 0.9 | 36° | 45 % |
| 40, 1.5 | 46° | 28 % |
| **60, 1.5** | **60°** | **15 %** ← chosen |

### 2. Implementation: [`aircraft/gnc_trainer/fcs/gnc_autopilot.xml`](../../aircraft/gnc_trainer/fcs/gnc_autopilot.xml)

Read the "LONGITUDINAL" section. Things to notice:

- **Gains are properties** (`ap/gains/kp-theta`...), so you can change them from
  Python without editing XML: tuning, scheduling and Monte Carlo all rely on this.
- **Incremental commands around trim.** Pitch trim holds the trim elevator, the
  autopilot adds `ap/elevator-cmd-norm`. θ<sub>c</sub> = θ<sub>trim</sub> + increment.
  [`gnclab.autopilot.engage_longitudinal`](../../src/gnclab/autopilot.py)
  captures θ<sub>trim</sub>, altitude and speed at engagement, so there is **no bump**.
- **Anti-windup** with the PID `<trigger>`: −1 resets the integrator when the
  mode is off, +1 freezes it outside the ±60 ft capture zone or when θ<sub>c</sub> or the
  total throttle saturates.
- **Derivative on measurement.** The pitch loop uses −k<sub>d</sub>·q, not d(error)/dt, so
  a θ<sub>c</sub> step doesn't kick the elevator.

### 3. Verification: [`solutions/verify_longitudinal.py`](solutions/verify_longitudinal.py)

Requirements R-L1…R-L4 are flown ideal and realistic. They all pass:

```text
[realistic] altitude step: overshoot 6.6 %, settle 6.2 s, final err -0.03 ft | speed step: overshoot 4.0 %,
            settle 4.5 s | coupling 4.6 ft | max |de_ap| 0.91
```

> 🐛 **The first run failed every requirement.** The airplane climbed, then
> descended into the ground with full up-elevator. The cause wasn't the pitch loop:
> `gnc_trainer` is **spiral unstable** (Module 07), so without lateral control it
> rolled off into a steep bank. Until Module 10 builds the roll autopilot, the
> verification holds the wings level with a two-line **Python-side wing leveler**
> in the sim callback (δ<sub>a</sub> = −1.5φ − 0.3p). Lesson: never test one axis
> of an autopilot on an airframe whose other axes are unstable.

Look at `alt_step.png`. With sensors on, the θ command is noisy: 2 ft of
baro noise × k<sub>p,h</sub> ≈ 1.2° of θ<sub>c</sub> noise, which drives the elevator. That's the
motivation for the estimators in Module 13.

---

## Exercises

1. **[`exercises/off_design_ex.py`](exercises/off_design_ex.py):** wrap the
   design in a `design(Va)` function. Compare fixed 25 m/s gains against
   re-designed gains at 18 and 32 m/s. Which speed suffers, and why?
2. Copy `aircraft/gnc_trainer/fcs/gnc_autopilot.xml` to
   `modules/09_longitudinal_autopilot/exercises/fcs/` and add a `lag_filter` on
   the altitude feedback (try 2 rad/s). Fly it with
   `make_fdm("gnc_trainer", systems_dir=...)`. How much does it cut elevator
   noise, and what does it do to the altitude-loop phase margin? (Add the same
   lag to `h_de` in the design script to check.)
3. Change the altitude loop to **PI with proportional on measurement**
   (θ<sub>c</sub> = −k<sub>p</sub>h + k<sub>i</sub>∫(h<sub>c</sub>−h)). What happens to the overshoot?

## Self-check

- What sets the first (innermost) gain in successive loop closure?
- Why is the pitch loop's DC gain less than one, and who fixes the error?
- What does "loop broken at the elevator with the others closed" mean, and why
  is it different from the single-loop margin?
- Give two anti-windup strategies and say which one this autopilot uses.

## Done when

- [ ] You can re-derive the six gains by hand from the A and B matrices
- [ ] `verify_longitudinal.py` passes all requirements and you can explain each margin
- [ ] You can explain all three design lessons and the spiral-mode bug to a colleague
