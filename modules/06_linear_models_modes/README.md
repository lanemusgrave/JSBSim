# Module 06: Linear models and dynamic modes

**Time:** about 5 h · **Reading:** Nelson ch. 4 (longitudinal motion) and ch. 5 (lateral motion), or S&L §3.7–3.8 *Linearization / modes*; Etkin & Reid ch. 6–7 for the approximations; RM *Programmer's Manual* §3.3 *JSBSim and Python*

## Objectives

1. Linearize a trimmed JSBSim model two ways: with JSBSim's `FGLinearization`
   and with **your own finite-difference linearizer**.
2. Read the A and B matrices: which entry is M<sub>α</sub>, L<sub>p</sub>, N<sub>β</sub>...
3. Extract and name the five classic modes: short period, phugoid, Dutch roll,
   roll subsidence and spiral. Check them against the textbook approximations.
4. Know when the linear model stops being true, including two traps that have
   nothing to do with aerodynamics: actuator hysteresis and JSBSim's one-frame
   input delay.

---

## Refresher

Linearize ẋ = f(x, u) about a trim point (x₀, u₀):

$$
\delta\dot{\mathbf x} = A\,\delta\mathbf x + B\,\delta\mathbf u,\qquad
A = \left.\frac{\partial f}{\partial \mathbf x}\right|_0,\quad
B = \left.\frac{\partial f}{\partial \mathbf u}\right|_0
$$

For a symmetric airplane in wings-level flight, the longitudinal states
[V, α, θ, q] and the lateral-directional states [β, φ, p, r] decouple.

| Mode | States | Typical (light aircraft) | What you feel |
|---|---|---|---|
| **Short period** | α, q | ω ≈ 3–10 rad/s, ζ ≈ 0.4–0.8 | the quick "bob" after a stick pulse |
| **Phugoid** | V, θ (α ≈ const) | period 20–60 s, ζ ≈ 0.05–0.2 | the slow speed/altitude exchange (Module 01) |
| **Roll subsidence** | p | τ = −1/L<sub>p</sub> ≈ 0.1–1 s | how quickly the roll rate stops after you center the stick |
| **Dutch roll** | β, r, (p) | ω ≈ 1–3 rad/s, ζ ≈ 0.05–0.3 | the tail wag/yaw-roll coupling (Module 03 rudder doublet) |
| **Spiral** | φ, ψ | very slow real root, ±0.01–0.05 /s | the bank slowly building or decaying hands-off (Module 04) |

**Approximations** (useful for sanity checks and for designing controllers by hand):

$$
\text{short period: } \det\begin{bmatrix} Z_\alpha/V & 1 \\ M_\alpha & M_q \end{bmatrix} \;\Rightarrow\;
\omega_{sp}^2 \approx \frac{Z_\alpha M_q}{V} - M_\alpha
\qquad
\text{phugoid (Lanchester): } \omega_{ph} \approx \frac{\sqrt2\, g}{V},\; \zeta_{ph} \approx \frac{1}{\sqrt2\,(L/D)}
$$

$$
\text{roll: } \lambda \approx L_p \qquad
\text{Dutch roll: } \omega_{dr}^2 \approx N_\beta + \frac{Y_\beta N_r}{V} \qquad
\text{spiral stable if } L_\beta N_r > N_\beta L_r
$$

## Two ways to linearize

| | `jsbsim.FGLinearization(fdm)` (`gnclab.linear.linearize`) | `gnclab.linear.linearize_fd(fdm, inputs)` |
|---|---|---|
| States | Vt, α, θ, q, **Rpm0**, β, φ, p, ψ, r, lat, lon, alt | Vt, α, θ, q, β, φ, p, r |
| Inputs | throttle, aileron, elevator, rudder **commands** | any properties you name |
| Needs | **at least one engine** (it segfaults on the glider!) | a *static* FCS between the input and the surface |
| How | JSBSim perturbs its own state vector | central differences with `run_ic()`; about 40 lines you can read |

Read `linearize_fd` in [`src/gnclab/linear.py`](../../src/gnclab/linear.py).
In a fork you'll often need exactly this: a linearizer around a custom state or
input (a new control effector, a different reference frame) that the built-in
one doesn't know about.

---

## Walkthrough

### 1. C172 modes: [`solutions/c172_modes.py`](solutions/c172_modes.py)

```text
quantity                   exact    approx
short period wn            6.472     6.469
short period zeta          0.676     0.676
phugoid wn                 0.195     0.254
phugoid zeta               0.155     0.078
Dutch roll wn              2.251     2.087
Dutch roll zeta            0.157     0.196
roll time const [s]        0.204     0.208
spiral: lambda = -0.0168 1/s (stable, time to half 41 s)
```

- Short period and roll: the 2×2 / 1×1 approximations are almost exact.
- **The phugoid approximation misses by 30% in ω and 2× in ζ.** Lanchester
  assumes constant thrust and no speed effects on the propeller. A prop
  airplane's thrust *falls* as speed rises, which adds damping. That's a
  reminder that "textbook approximation" ≠ "your airplane".
- The Dutch-roll period (2.8 s) matches what you measured from the rudder
  doublet in Module 03. Same mode, different method.

### 2. Linear vs. nonlinear: [`solutions/linear_vs_nonlinear.py`](solutions/linear_vs_nonlinear.py)

It linearizes the glider with `linearize_fd` (C172 short period: 6.470 vs
FGLinearization's 6.472 rad/s, so the two methods agree), then flies the same
doublet through both models:

```text
elevator doublet +/-0.05:  Alpha err 2.2 %, Theta 1.6 %, Q 3.7 %
elevator doublet +/-0.4:   Alpha err 6.3 %, Theta 7.5 %, Q 5.8 %, Vt 21.6 %
```

Large inputs shift the phugoid frequency (the speed excursion is no longer
small). **Two traps** you'll hit in real work:

1. **One-frame delay.** JSBSim's `run()` integrates *first* and evaluates FCS
   and aero *after*, so an input written before `run()` first moves the state
   one frame later. Compare with a *discrete* linear model (zero-order hold at
   the same dt) with its input delayed by one sample. Without that, the
   "linear-model error" in q was 14–22% even for tiny steps. The same
   one-frame delay shows up in every SIL loop (Module 16).
2. **Hard nonlinearities in the FCS.** The C172's elevator actuator has a
   `hysteresis_width` of 0.05 rad. A ±0.02 doublet **never moves the
   surface**, and after a ±0.15 doublet the surface stays 0.025 rad away from
   its trim position. The airplane then dives away (see
   `c172_hysteresis.png`). The linear model knows nothing about this. Module 08
   builds these effects on purpose.

---

## Exercises

1. **[`exercises/t38_modes_ex.py`](exercises/t38_modes_ex.py):** T-38 modes at
   four flight conditions. How does short-period frequency scale with q̄? Does
   Lanchester's phugoid period hold? Which condition would make you want a yaw
   damper?
2. Using the glider's A matrix, compute the spiral criterion
   L<sub>β</sub>N<sub>r</sub> − N<sub>β</sub>L<sub>r</sub> from the matrix entries (careful: A uses
   dimensional derivatives). Then compute it again for C<sub>lβ</sub> = −0.06 (Module 04)
   and confirm the sign change predicts the instability you saw.
3. Plot the glider's Bode diagram for θ/δ<sub>e</sub> and for φ/δ<sub>a</sub>. At what frequency
   does each plant's phase pass −180°? Keep the plots; you'll close loops
   around these plants in Modules 09–10.

## Self-check

- Why can't you linearize an untrimmed airplane?
- Which element of A is M<sub>α</sub> in `gnclab`'s longitudinal subsystem, and what sign
  must it have for static stability?
- The roll mode of a jet has τ = 1.2 s at low q̄. What would a pilot say about
  its handling?
- Your linear model and JSBSim disagree right after a step input, then agree.
  What's the first thing you check?

## Done when

- [ ] You can name each C172 mode from its eigenvalue and eigenvector
- [ ] You can explain the one-frame delay and the hysteresis trap
- [ ] The T-38 table is done and you have an opinion about a yaw damper
