# Module 10: answers

**A. The C172 autopilot (c172ap.xml) vs ours.** (Read the file; this summary is from v1.3.1.)
- *Heading hold:* heading error in **degrees** → wrap → clip ±30 → ×0.01745
  → `lag_filter` → that *is* the roll command. Then a **PID** (kp 6, ki 0.13, kd 6)
  on (roll command − φ) drives the aileron. So the heading loop is proportional
  only, with the integral action in the roll loop. Ours: PI on course → φ<sub>c</sub>, P on φ.
- *Wrap-around:* a `switch` adds ±360° to the error when it's outside ±180°.
  That works but is special-case logic. Ours: atan2(sin e, cos e).
- *Altitude hold:* altitude error clipped to ±100 ft → lag → **scheduled gain
  on altitude** (`scheduled_gain` component, table vs h) → **climb-rate
  command** → ḣ error → PID → elevator. That is a different architecture: it commands climb
  rate rather than pitch, and it is gain scheduled. Compare with Module 11.
- *Anti-windup:* the altitude PID's trigger comes from a `deadband` on elevator
  position (non-zero when the elevator is near its stops), so it freezes the
  integrator at saturation. The wing leveler's PID is triggered by the AP on/off switch.
- *Feedback:* the wing leveler reads a `<sensor>` on φ with noise/lag; the
  heading and altitude loops use truth properties (`attitude/heading-true-rad`,
  `position/h-agl-ft`, `velocities/h-dot-fps`).

**B. Sideslip feedback.** Peak |β| in the 90° turn: 1.40° (k<sub>β</sub> = 0), 1.00° (1.0),
0.78° (2.0). The turn time barely changes. Sideslip feedback adds directional
stiffness (it acts like extra C<sub>nβ</sub>), which raises the Dutch-roll frequency.
Combined with the washed-out r feedback, the turn is better coordinated. The
cost is that β is hard to measure on a small UAS (no vane; estimated from
lateral acceleration), so most small-UAS autopilots use the yaw damper alone or
a lateral-acceleration loop.

**C. Bank limit 45°.** The 90° change takes 6.6 s instead of 9.6 s (turn rate
∝ tan φ: tan 45°/tan 30° = 1.73). The altitude excursion grows from 7 to 17 ft:
the load factor goes from 1.15 to 1.41 g, so the altitude/pitch loop must find
about 25% more lift quickly, and the transient error grows. Bank limits are
usually set by structural or stall margin (n = 1/cos φ raises the stall speed by
√n) and by how much altitude deviation the mission tolerates.

**Self-check.**
- *Why washout on the yaw damper?* In a steady turn r ≠ 0. Without washout the
  damper would apply opposite rudder and fight the turn.
- *Why no k<sub>dφ</sub>?* The airframe's own roll damping (L<sub>p</sub> = −21.6/s) is already more
  than the design asks for; the formula gives a negative k<sub>d</sub>.
- *Why does the course loop use GPS course rather than heading?* Course is the
  direction you actually travel. With wind, heading ≠ course, and guidance
  (Module 12) wants the ground track.
