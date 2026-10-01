# Module 05: Trim and performance

**Time:** about 4 h · **Reading:** Nelson ch. 2 (static stability) and ch. 3 (performance) or S&L §2.6 and §3.6 *Steady-state flight / trim*; RM §3.4 *Initialization* and the `simulation/do_simple_trim` notes in RM ch. 4

## Objectives

1. State what "trim" means mathematically and what JSBSim's trim modes solve for.
2. Trim in level flight, climbs, turns and pull-ups, and turn trim sweeps into
   performance charts: drag bucket, power curves, fitted drag polar.
3. **Write your own trim solver** for a vehicle JSBSim can't trim (a glider).
4. Find a neutral point and static margin the way a flight-test engineer does,
   with a CG sweep.

---

## Refresher

**Trim** is an equilibrium of ẋ = f(x, u): pick some states and controls,
solve for the rest so that the accelerations vanish:

$$
\dot u = \dot v = \dot w = 0,\qquad \dot p = \dot q = \dot r = 0 \quad(\text{plus } \dot\phi=\dot\theta=0 \text{ for straight flight})
$$

Steady *turns* and *pull-ups* are equilibria too, in the sense that the body
accelerations are constant: n = 1/cos φ and ψ̇ = g tan φ / V for a coordinated
level turn; q = g(n − 1)/V for a pull-up at the bottom of a loop.

Every linear model (Module 06) and every gain-scheduled controller (Module 11) is
built **around a trim point**, which is why trim is the first thing a GNC engineer
does with a new model.

**Static longitudinal stability.** For a stable airplane C<sub>mα</sub> < 0 about the CG.
The *neutral point* x<sub>NP</sub> is the CG location where C<sub>mα</sub> = 0, and
static margin = (x<sub>NP</sub> − x<sub>CG</sub>)/c̄ (x positive aft). Trim elevator per unit C<sub>L</sub>,
dδ<sub>e</sub>/dC<sub>L</sub>, is proportional to the static margin. Pilots feel this as "more nose-down trim as
you speed up". Flight test measures it at several CGs and extrapolates to
zero to find x<sub>NP</sub>.

**Performance.** Level flight: L = W and T = D. Power required P = DV. Minimum
drag (best L/D) gives max range/glide for jets and gliders; minimum power gives
max endurance for props. A parabolic polar C<sub>D</sub> = C<sub>D0</sub> + KC<sub>L</sub>² gives
(L/D)<sub>max</sub> = 1/(2√(C<sub>D0</sub>K)) at C<sub>L</sub>* = √(C<sub>D0</sub>/K).

## JSBSim's trim

```python
fdm.do_trim(mode)            # or gnclab.trim.trim(fdm, "full") which raises TrimError nicely
```

| mode | name | target set by | solves with |
|---|---|---|---|
| 0 | longitudinal | `ic/vc-kts`, `ic/gamma-deg` | α, throttle, pitch trim |
| 1 | full | + wings level | + φ, aileron, rudder |
| 2 | ground | on the gear | altitude, θ, φ |
| 3 | pullup | `ic/targetNlf` | α (for n), throttle, pitch trim |
| 5 | turn | `ic/phi-deg` (n = 1/cos φ) | α, throttle, pitch trim, β, aileron, rudder |

It pairs each acceleration with one control (u̇↔throttle, ẇ↔α, q̇↔pitch trim...)
and iterates axis by axis. Two consequences:

- **The FCS must route `fcs/pitch-trim-cmd-norm` (and roll/yaw trim) to the
  surfaces.** Look at the glider's `<summer>`s in Module 04.
- **No throttle, no u̇ trim.** A glider fails with *"udot doesn't appear to be
  trimmable"*. So does any airplane asked to fly faster than its thrust allows,
  and that is how you find V<sub>max</sub> by bisection.

The trim also prints some diagnostics even in quiet mode (e.g. *"Sorry, udot
doesn't appear to be trimmable"*). That's expected when a sweep goes past the
envelope.

---

## Walkthrough

### 1. C172 performance charts: [`solutions/c172_performance.py`](solutions/c172_performance.py)

A level sweep, climbs and turns. Highlights from the output:

```text
fitted polar: CD = 0.0401 + 0.0315 CL^2  ->  (L/D)max = 14.1 at CL = 1.13
minimum drag at 60 KCAS, minimum power at 50 KCAS
 bank_deg  nz_g  1/cos(phi)  turn_rate_dps  g*tan(phi)/V [dps]
       60 1.994       2.000         19.834              19.816
```

- ⚠️ JSBSim stores wind-axis aero forces as **magnitudes**:
  `forces/fwx-aero-lbs` = drag (positive), `forces/fwz-aero-lbs` = lift
  (positive). They are not a right-handed x/y/z.
- The pitch-trim plot rises with speed. That is positive speed stability (the
  airplane needs more nose-down trim to go faster).

### 2. Write your own trim: [`solutions/glider_trim.py`](solutions/glider_trim.py)

Unknowns [α, γ, pitch trim], residuals [u̇, ẇ, q̇], solved with
`scipy.optimize.least_squares`. Each evaluation writes ICs and calls
`run_ic()`, which runs every model **once with integration suspended**, so
the residuals come from the real JSBSim model. Then the script proves the
answer: started exactly at the solution, the glider holds V to 0.04 ft/s
over 30 s.

The same idea, generalized, is `gnclab.trim.trim_custom` (read it in
[`src/gnclab/trim.py`](../../src/gnclab/trim.py)). You'll use it for the UAS model.

### 3. Neutral point: [`solutions/neutral_point.py`](solutions/neutral_point.py)

A 1 kg ballast slides along the glider (`inertia/pointmass-location-X-inches`).
At each CG the script trims at four speeds and fits dδ<sub>e</sub>/dC<sub>L</sub>:

```text
CG at x = 0.400 m: d(de)/dCL = -8.07 deg
CG at x = 0.450 m: d(de)/dCL = +1.26 deg
stick-fixed neutral point: x_NP = 0.4433 m  (theory ~0.443 m)
```

The last CG (0.45 m) is *behind* the neutral point: still trimmable, but statically
unstable. A real program sets an aft CG limit with margin (typically 5–10 %
c̄ for an unaugmented UAS), and that number goes on your mass-properties
requirement.

---

## Exercises

1. **[`exercises/t38_trim_ex.py`](exercises/t38_trim_ex.py)**, a jet you
   know: the T-38's level envelope at 15,000 ft, max level speed by
   bisection, and a 2 g pull-up compared with q = g(n−1)/V. Where is the
   minimum-drag speed, and what does it mean to fly below it?
2. Extend `glider_trim.py` with a fourth unknown, φ, and a fifth residual,
   so you can trim a **steady turn**. (Hint: in a turn the body rates aren't
   zero. Set `ic/p,q,r-rad_sec` from ψ̇ = g tan φ / V. That's harder than it
   looks; compare with JSBSim's mode 5 on the C172 first.)
3. In `c172_performance.py`, add a 2 g pull-up and a 60° turn at the same
   speed and compare trim α. Why do they match?
4. Use the C172 climb table to estimate the best rate of climb at full
   throttle (sweep KCAS at `gamma-deg` values until the trim fails).

## Self-check

- Why does every linear model need a trim point first?
- JSBSim can't trim your new glider. Name two ways to fix that.
- The C172's pitch trim goes from −0.22 at 50 KCAS to +0.25 at 115 KCAS. Is
  that stable or unstable speed stability, and why?
- What happens to the stick-free/stick-fixed neutral point when you add a
  bigger horizontal tail? To the static margin when you burn fuel from an aft tank?

## Done when

- [ ] You can explain each trim mode's pairing of states and controls
- [ ] Your T-38 envelope matches the solution's to within a knot or two
- [ ] You can write a 15-line custom trim with `least_squares`
