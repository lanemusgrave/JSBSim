# Module 02: Frames, units and the equations of motion

**Time:** about 4 h · **Reading:** RM §2.2 *Frames of Reference*, §2.3 *Units*;
RM *Formulation Manual* §2 *Equations of Motion* (pp. 97–105); refresher: S&L
ch. 1.3–1.4 and 2.1–2.5, or B&M ch. 2–3

## Objectives

1. Name the frames JSBSim uses and know which property lives in which frame.
2. Write the body ↔ NED rotation, the Euler-angle kinematics, α/β/γ and the
   quaternion yourself, and check them against JSBSim to machine precision.
3. Re-derive the 6-DOF force and moment equations and check one of them on
   simulator data.
4. Know JSBSim's two big gotchas: the **structural frame** in the XML and
   **English units** everywhere.

---

## Refresher

### Frames

| Frame | Origin / axes | Where you meet it in JSBSim |
|---|---|---|
| **ECI** (inertial) | Earth centre; non-rotating | where Newton's law holds (`velocities/eci-velocity-mag-fps`) |
| **ECEF** | Earth centre; rotates at ω⊕ = 7.292×10⁻⁵ rad/s | positions (`position/lat-geod-deg`, `long-gc-deg`, `radius-to-vehicle-ft`) |
| **Local NED** | at the vehicle; x North, y East, z Down (tangent plane) | `velocities/v-north-fps`, `v-east-fps`, `v-down-fps`; Euler angles are body relative to NED |
| **Body** | at the CG; x nose, y right wing, z belly | `velocities/u,v,w-fps`, `p,q,r-rad_sec`, `forces/fbx...` |
| **Stability** | body rotated by α about y | stability derivatives (Cmα, Clp…) are often defined here |
| **Wind** | x along the relative wind | lift and drag (`forces/fwx...`) |
| **Structural** ⚠️ | arbitrary origin (often the nose); **x aft, y right, z up**, **inches** | every `<location>` in an aircraft XML file: CG, aero reference point, gear, engines |

The structural frame is a JSBSim (and aircraft-industry) convention: "fuselage
station / butt line / waterline". It is **not** the body frame. JSBSim converts
it for you, but when you type a CG location or a thrust line into XML (Module
04/07) you are in inches, x aft, z up.

### Rotations

3-2-1 Euler sequence (ψ yaw, θ pitch, φ roll). NED → body:

$$
C_{b/n} = R_x(\phi)R_y(\theta)R_z(\psi) =
\begin{bmatrix}
c\theta c\psi & c\theta s\psi & -s\theta\\
s\phi s\theta c\psi - c\phi s\psi & s\phi s\theta s\psi + c\phi c\psi & s\phi c\theta\\
c\phi s\theta c\psi + s\phi s\psi & c\phi s\theta s\psi - s\phi c\psi & c\phi c\theta
\end{bmatrix},
\qquad C_{n/b} = C_{b/n}^T
$$

Euler-angle kinematics (singular at θ = ±90°: gimbal lock):

$$
\dot\phi = p + (q\sin\phi + r\cos\phi)\tan\theta,\quad
\dot\theta = q\cos\phi - r\sin\phi,\quad
\dot\psi = \frac{q\sin\phi + r\cos\phi}{\cos\theta}
$$

JSBSim integrates the **quaternion** (4 parameters, no singularity, one
constraint ‖q‖ = 1) and reports Euler angles derived from it.

### Air data

$$
\alpha = \operatorname{atan2}(w_r, u_r), \qquad \beta = \arcsin(v_r / V), \qquad V = \lVert \mathbf v_r \rVert
$$

with **r = relative to the air mass** (body velocity minus wind). With no
wind, as here, it equals the ground-relative velocity. Flight-path angle
γ = asin(−v_D / |V_NED|); in wings-level flight with no sideslip, γ = θ − α.

### Equations of motion (flat, non-rotating Earth form, body axes)

$$
\begin{aligned}
m(\dot u + qw - rv) &= X - mg\sin\theta \\
m(\dot v + ru - pw) &= Y + mg\cos\theta\sin\phi \\
m(\dot w + pv - qu) &= Z + mg\cos\theta\cos\phi \\
I\dot{\boldsymbol\omega} + \boldsymbol\omega \times I\boldsymbol\omega &= \mathbf M
\end{aligned}
$$

The moment equations are coupled through I<sub>xz</sub> (roll–yaw). JSBSim
solves the full version: rotating oblate Earth (WGS-84, J2 gravity), Coriolis
and centripetal terms, quaternion attitude. See RM *Formulation* §2. For an
aircraft over a few minutes the flat-Earth form above is within about 1%, and
that is what the textbooks and your linear models (Module 06) use.

### Units

JSBSim's internals are **English**: ft, slug, lbf, s, rad, °R. The XML accepts
units per value (`<wingarea unit="M2">`, `unit="KG"`, `"KG*M2"`, `"IN"`, `"DEG"`,
`"KTS"`, `"N"`, `"LBS"`...) and converts on load. Properties carry their unit in
the name. **1 slug = 14.59 kg; 1 lbf = 4.448 N; 1 kt = 1.688 ft/s.**
`gnclab.units` has the constants.

---

## Walkthrough

1. **Frame math:** read [`solutions/frames.py`](solutions/frames.py), then run
   [`solutions/kinematics_check.py`](solutions/kinematics_check.py). It flies a
   rolling pull-up and checks your math against JSBSim:

   ```text
   1. alpha err 1.2e-13 deg, beta err 9.5e-14 deg
   2. NED velocity err 1.3e-08 ft/s
   3. Euler-rate err vs JSBSim 1.6e-15 deg/s; vs numerical d(phi)/dt 0.573 deg/s
   4. gamma err 3.2e-15 deg
   5. udot err 0.049 ft/s^2 (1.2% of max |udot|)
   ```

   Two things to notice. (3) A numerical derivative of logged data is much
   worse than the analytic kinematics; remember that when you differentiate
   flight-test logs (Module 14). (5) Newton holds to about 1%. The rest is the
   rotating Earth plus a one-step offset (Propagate runs at the *start* of
   `run()`, so the logged velocities are one step ahead of the logged
   accelerations). Also note that `forces/fbx-total-lbs` **excludes gravity**;
   the weight component is a separate property.

2. **Rotating Earth:** run [`solutions/ball_drop.py`](solutions/ball_drop.py).
   A drag-free ball dropped from 10,000 ft at the equator:
   - falls with g<sub>eff</sub> = gravitation − centrifugal ≈ 32.06 ft/s²,
     not 32.17;
   - drifts **east** at ω g t² cos(lat): 0.94 ft/s after 20 s. JSBSim matches the
     Coriolis estimate to three decimals.

   This is why JSBSim is trusted for long-range and high-speed vehicles. A
   flat-Earth sim would miss both effects, and over a 30-minute flight they add
   up to navigation-sized errors.

3. **Stiff contact and `dt`:** the second half of `ball_drop.py` drops the ball
   onto its ground spring/damper at 120, 240 and 480 Hz. At 120 Hz it "rebounds"
   faster than it hit. That energy comes from the integrator: the damper
   eigenvalue −c/m ≈ −320 s⁻¹ times dt sits outside the Adams–Bashforth 2
   stability region. Only 480 Hz gives the physical (overdamped, no-bounce)
   answer. When someone's sim "bounces" on landing, check `dt` first.

---

## Exercises

1. **[`exercises/frames_ex.py`](exercises/frames_ex.py):** implement
   `dcm_body_to_ned`, `euler_rates`, `alpha_beta` and `euler_to_quat`. The
   file tests each against JSBSim and prints PASS/FAIL.
2. Gimbal lock: with your `euler_rates`, evaluate ψ̇ for θ = 89.9° and
   p = q = r = 0.1 rad/s. What happens at 90°? Why doesn't JSBSim care?
3. Change `ball_drop.py` to latitude 45° and 80°. Predict the east drift and
   the effective g before you run it.
4. In the structural frame, the C172's *empty* CG is at x = 41.0 in
   (`<location name="CG">` in `c172x.xml`) and the aero reference point
   (`AERORP`) is at x = 43.2 in. With pilot, passengers, fuel and baggage the
   loaded CG moves to `inertia/cg-x-in` ≈ 45.5 in. Is each CG ahead of or
   behind the ARP? Which way does lift acting at the ARP pitch the airplane in
   each case? (You'll need this in Module 05.)

## Self-check

- Which property is in the body frame: `velocities/v-fps` or `velocities/v-east-fps`?
- Write C<sub>b/n</sub> for a pure roll φ. Which way does +φ rotate the right wing?
- Why does JSBSim integrate quaternions rather than Euler angles?
- In wings-level flight θ = 5°, α = 3°. What is γ? What about at φ = 60°?
- Why is `forces/fbx-total-lbs` not equal to m·u̇?

## Done when

- [ ] `frames_ex.py` prints four PASS lines
- [ ] You can write the force equations and Euler kinematics from memory
- [ ] You can explain the eastward drift and the 120 Hz bounce
