# Module 04: answers

**Exercise 1, Q1 (flaps).** With flaps the glider's lift curve shifts up by
ΔC<sub>L</sub> = 0.5. Pitch equilibrium (C<sub>m</sub> = 0) fixes the angle of attack;
the small nose-down ΔC<sub>m</sub> lowers it from 2.15° to 0.72°. But the total
C<sub>L</sub> at that α is now *higher* (about 0.86 vs 0.49), and weight is unchanged, so
V = √(2W/ρSC<sub>L</sub>) drops from 14.8 to 11.2 m/s. Pilots know this: flaps
let you fly slower at a lower deck angle.

*Reviewer's catch:* the glide angle is unchanged (−3.27°). That's partly
coincidence, and partly a **modeling error**: the polar only squares
`L_alpha`, so the flap's extra lift creates no induced drag. A reviewer should
flag that the induced-drag term should use the *total* C<sub>L</sub>.

**Exercise 1, Q2 (spiral).** With C<sub>lβ</sub> = −0.06:
C<sub>lβ</sub>C<sub>nr</sub> = 0.0048 < C<sub>nβ</sub>C<sub>lr</sub> = 0.0072, so the spiral is
unstable. Any small bank produces a slip into the low wing. The weathercock
stiffness (C<sub>nβ</sub>) yaws the nose into the turn, the yaw rate creates a rolling
moment (C<sub>lr</sub>, the outer wing moves faster) that out-pushes the dihedral
restoring moment (C<sub>lβ</sub>), and the bank keeps growing. It's slow (time to
double is tens of seconds), which is why pilots can fly spirally unstable
airplanes easily and why it's the classic job of a wing leveler.

**Exercise 3 (CG shift).** Moving the CG 5 cm forward with the ARP fixed:
C<sub>m,CG</sub> = C<sub>m,ARP</sub> + C<sub>L</sub>(x<sub>ARP</sub> − x<sub>CG</sub>)/c̄ =
C<sub>m,ARP</sub> + C<sub>L</sub>(0.05/0.27). With C<sub>L</sub> ≈ 0.49 that's ΔC<sub>m</sub> ≈ −0.091
(nose down). The elevator must supply +0.091: Δδ<sub>e</sub> = −0.091/1.2 ≈
−0.076 rad ≈ −4.3° (trailing edge up), or −0.22 in normalized command. The
equilibrium α also shifts a little because C<sub>Lδe</sub> ≠ 0. Static margin goes
up by 0.05/0.27 ≈ 0.19 c̄.

**Self-check.**
- DRAG/SIDE/LIFT: wind. ROLL/PITCH/YAW: body (about the AERORP).
- Without b/(2V): b/(2V) = 3/(2·15) = 0.1, so the damping is **10× too large**.
- Moments are taken about the CG in the equations of motion. Any force not
  acting at the CG (lift and drag at the ARP, thrust on its line, gear) has an
  arm r × F, and the arm changes when the CG moves.
- `BOGEY`: a wheel (rolling, brakes, steering, retractable). `STRUCTURE`:
  a hard point that only touches the ground in a crash or on skids.
- Contribution table before and after the change, at the same trim point,
  and diff them term by term.
