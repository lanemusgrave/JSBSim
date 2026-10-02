# Module 13: answers

**A. Gyro-bias EKF** (90 s of alternating 90° turns):

| | b<sub>p</sub> | b<sub>q</sub> | b<sub>r</sub> [deg/s] |
|---|---|---|---|
| true | 0.229 | −0.172 | 0.115 |
| estimated | 0.158 | −0.213 | 0.218 |

Second-half RMS error: roll 0.89° → 0.51°, pitch 0.56° → 0.47°.
- b<sub>p</sub> and b<sub>q</sub> are observable through the attitude. A wrong roll rate makes
  φ drift and the accelerometer's lateral/vertical components disagree with
  the model. b<sub>q</sub> also enters directly through −qV<sub>a</sub> in the f<sub>z</sub> model.
- **b<sub>r</sub> is weakly observable.** It only shows up through rV<sub>a</sub> in f<sub>y</sub> and
  through the Euler kinematics when banked. In straight flight it's almost
  invisible, so it takes turns (excitation) to estimate. Real AHRS units add a
  magnetometer or GPS course for heading and yaw-rate bias.
- Q for the biases is a tuning knob: too small and they never move; too large
  and they soak up model error (sideslip, u̇, gusts) as fake "bias".

**B. Tuning.** Larger R (trusting the accelerometer less) means smoother but
lagging estimates that drift with gyro bias. Smaller R means faster
correction but more noise, and more corruption from the model's unmodeled
accelerations (u̇, gusts). Q works the other way round. The classic trade
is noise vs lag, the same as the filter corner in Module 11.

**C. Wind.** Ground velocity from GPS: V<sub>g</sub>[cos χ, sin χ]. Air velocity:
V<sub>a</sub>[cos ψ, sin ψ] (no sideslip). Wind = difference. With noisy heading and a
constant-wind assumption, a least-squares fit over an orbit (heading
excites all directions) recovers the wind to about 0.5 m/s. In straight flight,
wind along the track and an airspeed scale-factor error are indistinguishable,
which is why calibration and wind-estimation maneuvers are flown as orbits or boxes.

**Self-check.**
- *Why does a complementary filter fail in a coordinated turn?* The
  accelerometer measures specific force, which stays along body −z in a
  coordinated turn, so the "gravity vector" says wings level.
- *What fixes it?* Model the turn: the EKF's f<sub>y</sub> = rV<sub>a</sub> − g cos θ sin φ
  includes the centripetal term (needs airspeed). Alternatives: GPS-aided INS,
  or gating the accel correction during turns.
- *Why did the raw-sensor altitude hold drive the elevator at 42°/s RMS?* Baro
  noise × k<sub>p,h</sub> plus the noisy complementary-filter θ, both straight into the
  elevator. The baro+accelerometer KF gives a smooth h (and ḣ), and the
  EKF gives a smooth θ.
