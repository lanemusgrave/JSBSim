# Module 10: Lateral-directional autopilot

**Time:** about 5 h · **Reading:** B&M §6.3 (lateral autopilot: roll, course, sideslip), §5.3–5.4 lateral transfer functions; RM Case Study 3 *Piston aircraft with autopilot* (wing leveler §4.1, heading hold §4.2) and the bundled `aircraft/c172x/c172ap.xml`

## Objectives

1. Design roll hold, course hold and a yaw damper from your JSBSim linear model.
2. Handle **heading wrap-around** correctly, and know why washout belongs in
   a yaw damper.
3. Add the lateral loops to the XML autopilot and fly coordinated, altitude-held
   course changes with all non-idealities on.
4. Read someone else's autopilot (the C172's) and compare design choices.

---

## Refresher

$$
\frac{\phi}{\delta_a}\approx\frac{a_{\phi2}}{s(s+a_{\phi1})},\; a_{\phi1}=-L_p,\; a_{\phi2}=L_{\delta a}
\qquad
\frac{\chi}{\phi}\approx\frac{g}{V_g\,s}\;\;(\dot\chi = \tfrac{g}{V_g}\tan\phi)
$$

$$
k_{p\phi}=\frac{\delta_{a,max}}{e_{\phi,max}},\;
\omega_\phi=\sqrt{|a_{\phi2}|k_{p\phi}},\;
k_{d\phi}=\frac{2\zeta_\phi\omega_\phi-a_{\phi1}}{a_{\phi2}}
\qquad
k_{p\chi}=\frac{2\zeta_\chi\omega_\chi V_g}{g},\;k_{i\chi}=\frac{\omega_\chi^2V_g}{g},\;\omega_\chi=\omega_\phi/W_\chi
$$

**Yaw damper:** δ<sub>r</sub> = k<sub>r</sub> · [s/(s + 1/τ)] r. The **washout** passes the Dutch-roll
oscillation (about 4.7 rad/s here) and blocks the steady yaw rate of a turn, which
the damper must not fight.

**Wrap-around:** e<sub>χ</sub> = atan2(sin(χ<sub>c</sub> − χ), cos(χ<sub>c</sub> − χ)) ∈ (−π, π]. Commanding
350° from 0° must give −10°, not +350°.

---

## Walkthrough

### 1. Design: [`solutions/design_lateral.py`](solutions/design_lateral.py)

```text
roll: a1 = 21.65 (= -Lp), a2 = 54.64; kp = 1.432, kd = 0.000 -> w = 8.85 rad/s
yaw damper: kr = 0.230 -> Dutch roll zeta 0.24 -> 0.50
loop at aileron (roll)       PM  62 deg  wc 4.7 rad/s  delay margin 229 ms
loop at phi_c (course)       GM  29 dB  PM  70 deg  wc 0.97 rad/s  delay margin 1260 ms
loop at rudder (yaw damper)  GM  30 dB  PM 142 deg
```

- k<sub>dφ</sub> comes out **negative**: the airframe already has more roll damping than
  ζ = 0.8 needs (L<sub>p</sub> = −21.6/s). Following B&M, set it to 0 and keep the
  structure for other airframes.
- The course loop was picked from a small sweep, the same way as the altitude loop:

  | W<sub>χ</sub>, ζ<sub>χ</sub> | PM | linear OS |
  |---|---|---|
  | 15, 1.0 | 61° | 20 % |
  | 15, 2.5 | 64° | 14 % |
  | **25, 1.5** | **70°** | **10 %** ← chosen |

### 2. Implementation

The "LATERAL-DIRECTIONAL" section of
[`gnc_autopilot.xml`](../../aircraft/gnc_trainer/fcs/gnc_autopilot.xml):
course PI (wrap via `atan2`, integrator frozen while the bank command saturates)
→ φ<sub>c</sub> clipped to ±30° → roll P → aileron, plus the washout yaw damper.
Engage with `gnclab.autopilot.engage_lateral(fdm)`.

### 3. Verification: [`solutions/verify_lateral.py`](solutions/verify_lateral.py)

Six requirements, flown ideal and realistic, all passing:

```text
[realistic]
   PASS  R-A1 roll rise 0.30 s <= 1.0 s;  overshoot 5.7 %
   PASS  R-A2 course overshoot 1.09 deg <= 5 deg; within 2 deg after 20 s
   PASS  R-A3 max |beta| 1.55 deg <= 3
   PASS  R-A4 altitude excursion 7.2 ft <= 30
   PASS  R-A5 wrap: course went -11.2..0.0 deg (left turn to -10)
   PASS  R-A6 beta after doublet 0.107 deg (damper off: 0.687)
```

> 🐛 **A test-harness bug worth knowing:** the first realistic run "failed"
> R-A2 with a 270° overshoot. The autopilot was fine. The *logged* course is in
> [0°, 360°), and with sensor noise near north it read 359.9°. Always unwrap
> angles before computing metrics. A wrong metric fails a good controller as
> easily as it passes a bad one.

Look at `ground_tracks.png`: the 0 → 350° test turns left, as it should.

---

## Exercises ([`exercises/lateral_ex.py`](exercises/lateral_ex.py))

- **A.** Read the C172 autopilot (`aircraft/c172x/c172ap.xml` in the JSBSim data
  folder) and compare it with ours: structure, wrap-around, anti-windup, feedback.
- **B.** Add **sideslip feedback** to the rudder in your copy of the autopilot.
  How much does peak |β| drop in the 90° turn, and why don't small UAS usually do this?
- **C.** Raise the bank limit to 45°. Faster turns, but what does it cost?

> Gotcha from building the solution: `<property>` declarations must sit at the
> `<system>` level. Inside a `<channel>`, JSBSim reports *"Unknown FCS component: property"*.

## Self-check

- Why is the roll loop's DC gain 1 without an integrator, while the pitch loop's isn't?
- What happens to a yaw damper without washout during a 30° bank turn?
- Course vs heading: which does the course loop use, and why does it matter in wind?
- Your heading hold sometimes turns the long way round. What's the first thing you check?

## Done when

- [ ] All six lateral requirements pass, ideal and realistic
- [ ] You can explain the washout and the wrap-around handling to a colleague
- [ ] You've compared the C172 autopilot with yours
