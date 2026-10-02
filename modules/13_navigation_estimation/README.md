# Module 13: Navigation and state estimation

**Time:** about 6 h · **Reading:** B&M ch. 7 (sensors) and ch. 8 (state estimation: low-pass filters, attitude EKF §8.6, GPS smoothing §8.7); for the Kalman filter itself, any short treatment (Simon, *Optimal State Estimation* ch. 5 and 13; or the Labbe *Kalman and Bayesian Filters in Python* notebooks, free online)

## Objectives

1. Model a small-UAS sensor suite from JSBSim truth: gyro (noise + **bias**),
   accelerometer (**specific force**), baro, pitot, GPS.
2. Build and compare attitude estimators: a complementary filter and B&M's attitude EKF.
3. Fuse baro and accelerometer into a smooth altitude.
4. **Close the autopilot on estimated states** and quantify what estimation buys.

---

## Refresher

**Kalman filter** (discrete, linearized for the EKF):

$$
\begin{aligned}
\text{predict:}\;& \hat x^- = f(\hat x, u)\,,\quad P^- = A P A^T + Q \\
\text{correct:}\;& K = P^-H^T(HP^-H^T+R)^{-1},\quad \hat x = \hat x^- + K\,(y - h(\hat x^-)),\quad P = (I-KH)P^-
\end{aligned}
$$

Q says how much you distrust the *model*; R how much you distrust the *sensor*.

**Accelerometers measure specific force**, f = (non-gravitational force)/m, not
acceleration. In steady straight flight f ≈ −g<sub>body</sub>, which looks like "gravity",
so tilt = atan2 of its components. In a **coordinated turn** f stays along body
−z, so the accelerometer says "wings level" at any bank angle. B&M's attitude EKF
fixes this with a model that includes the turn:

$$
f_x = g\sin\theta,\qquad f_y = rV_a - g\cos\theta\sin\phi,\qquad f_z = -qV_a - g\cos\theta\cos\phi
$$

Where JSBSim gives you each signal (see [`gnclab/estimation.py`](../../src/gnclab/estimation.py) `truth()`):
specific force = `forces/fb[xyz]-total-lbs` / mass. Recall from Module 02 that
"total" excludes gravity, which is exactly what an accelerometer senses.

## The code

[`src/gnclab/estimation.py`](../../src/gnclab/estimation.py): `Sensors`,
`ComplementaryAttitude`, `AttitudeEKF` (2 states, φ and θ), `AltitudeKF`
(h and ḣ from the accelerometer rotated with the attitude estimate, corrected
by baro). The Python autopilot gained a `feedback=` hook so it can fly on any
estimate.

---

## Walkthrough

### 1. Attitude through S-turns: [`solutions/attitude_estimation.py`](solutions/attitude_estimation.py)

```text
complementary : roll error RMS 13.24 deg (in the turn: max 25.04), pitch error RMS 4.09 deg
EKF           : roll error RMS  0.89 deg (in the turn: max  1.48), pitch error RMS 1.16 deg
```

Look at `attitude_estimation.png`. The complementary filter's roll estimate
**decays toward zero during every turn**, pulled there by the accelerometer. A
naive AHRS like this can "tell" an autopilot that the wings are level while
banked 30°, and that is a real failure mode in cheap autopilots. The EKF tracks
through the turn because its model explains the turn's acceleration.

### 2. Closing the loop on estimates: [`solutions/closed_loop_estimation.py`](solutions/closed_loop_estimation.py)

```text
 feedback   alt error RMS in hold [ft]   elevator rate RMS [deg/s]
    truth                        0.35                        0.01
      raw                        0.59                       41.91
estimated                        0.26                        1.55
```

Raw baro altitude and complementary-filter pitch feed **42°/s RMS** of
elevator motion into the servos: noise, heat, wear and power. That was the
problem visible in Module 09's plots. With the KF altitude and EKF pitch the
autopilot holds altitude as well as on truth, with a 27× reduction in actuator
activity. That is what estimation is for.

---

## Exercises ([`exercises/estimation_ex.py`](exercises/estimation_ex.py))

- **A.** Add **gyro-bias states** to the EKF (5 states). Which biases converge,
  and why is yaw-rate bias the hardest? Solution:
  [`solutions/estimation_ex.py`](solutions/estimation_ex.py)
  (roll RMS 0.89° → 0.51°).
- **B.** Tune Q and R; describe the noise-vs-lag trade.
- **C.** Estimate the **wind** from GPS ground velocity, pitot airspeed and
  heading, flying an orbit. Why is an orbit needed?

## Self-check

- What does an accelerometer measure at rest on the ground? In free fall? In a 60° coordinated turn?
- In your own words, what do Q and R mean, and what happens if you get them backwards?
- Why does the altitude KF need the attitude estimate?
- Name one failure mode that switching from truth to estimates can introduce
  into a control loop that was stable on truth.

## Done when

- [ ] You can derive the accelerometer model for steady flight and for a coordinated turn
- [ ] Your bias EKF converges in turns and you can explain b<sub>r</sub>
- [ ] You can explain the 27× elevator-activity reduction to a colleague
