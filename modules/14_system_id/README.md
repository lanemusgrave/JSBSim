# Module 14: System identification

**Time:** about 7 h · **Reading:** Klein & Morelli, *Aircraft System Identification* (2nd ed.): ch. 5 (regression / equation error), ch. 6 (output error), ch. 7 (frequency domain), ch. 9 (input design). Free alternative: Morelli & Klein's NASA TMs on equation-error and frequency-response methods (search NTRS for "Morelli system identification").

## Objectives

1. Design and fly a **simulated flight test** with realistic instrumentation noise.
2. Identify aerodynamic derivatives by **equation error** (OLS), **output error**,
   and **frequency response**, and compare them with the truth the model was built from.
3. **Validate** on held-out data, and know why fit quality on the training
   maneuver proves little.
4. Recognize identifiability problems: poor excitation, small regressors,
   biased error bars.

This is the "system identification and aerodynamic model development from flight
data" part of the job. Here the truth is known (the Module 07 parameters), so
you can see exactly how well each method works.

---

## Refresher

**Equation error.** Turn the equations of motion into a measured coefficient
and regress it on the states and controls:

$$
C_m = \frac{I_{yy}\,\dot q}{\bar q S \bar c} = C_{m_0} + C_{m_\alpha}\alpha + C_{m_q}\frac{\bar c}{2V}q + C_{m_{\delta e}}\delta_e
\quad\Rightarrow\quad \hat\theta = (X^TX)^{-1}X^Ty
$$

It needs q̇, a *differentiated* noisy signal, and noise in the regressors biases
the answer (errors-in-variables).

**Output error.** Simulate the model; choose parameters to minimize the output
mismatch Σ(y<sub>meas</sub> − y<sub>model</sub>(θ))². No differentiation, and noise only on outputs.
This is a nonlinear least-squares problem (`scipy.optimize.least_squares`).

**Frequency response.** H(f) = S<sub>uy</sub>/S<sub>uu</sub> from a sweep. **Coherence**
γ² = |S<sub>uy</sub>|²/(S<sub>uu</sub>S<sub>yy</sub>) tells you where H is trustworthy (γ² > 0.6).

**Input design.** The input must make the regressors *independent*: excite the
mode of interest with broad frequency content (3-2-1-1, sweeps), from trim, at
amplitudes that stay linear but beat the noise.

## The code

- [`src/gnclab/sysid.py`](../../src/gnclab/sysid.py): `smooth_derivative`
  (Savitzky–Golay), `ols` (estimates + standard errors + truth comparison),
  `frf` (H1 estimator + coherence), `theil` (validation fit metric).
- [`solutions/flight_test.py`](solutions/flight_test.py) flies five maneuvers and
  writes 100 Hz CSV logs with noise on α, β, rates, accelerometers, airspeed and
  surface positions (`outputs/14_system_id/`). Module 16 reuses these logs.

---

## Walkthrough

### 1. Equation error: [`solutions/equation_error.py`](solutions/equation_error.py)

Fit on the 3-2-1-1s, predict the held-out doublets:

```text
Cm:  Cma -2.645 (truth -2.74, 3.5 %)  Cmq -31.1 ± 0.8 (truth -38.2, 19 %)  Cmde -0.920 (7 %)
CL:  CL0 0.224 (2 %)  CLa 5.35 (5 %)  CLq -1.5 ± 1.5 (truth 7.95)   CLde -0.03 (truth 0.13)   <- not identifiable
Cl:  Clb -0.126 (3 %)  Clp -0.495 (3 %)  Clr 0.249 (0.6 %)  Clda 0.167 (2 %)
Cn:  Cnb 0.071 (3 %)  Cnr -0.087 (8 %)  Cndr -0.069 (0.5 %)
validation Theil U: Cm 0.19, CL 0.006, CY 0.02, Cl 0.09, Cn 0.04
```

Three lessons, all of which you'll meet in real flight-test data:

- **Error bars lie.** Cmq is 7 units off with a "standard error" of 0.8. OLS
  assumes white residuals; real (and here, filtered) residuals are coloured.
- **Some parameters aren't in the data.** CLq and CLδe contribute about 1% of CL,
  below the noise. Don't report them from this maneuver; fix them a priori.
- **Lateral-directional came out great** because the two-input 3-2-1-1s excite
  β, p and r independently.

### 2. Output error: [`solutions/output_error.py`](solutions/output_error.py)

```text
         equation error  output error   truth
Cmq            -31.1          -42.4    -38.21    (19 % -> 11 %)
Cmde           -0.920         -0.992   -0.990    (7 % -> 0.2 %)
validation Theil U (q):  0.074 -> 0.047
```

Better where equation error was biased, and better on held-out data. The Cm0
and CL0 shifts show **model-structure error**: the short-period model ignores
CLq, the speed change and the thrust. A parameter can be "wrong" in a way that
makes the *model* more right. Judge models on predictions, not on individual numbers.

### 3. Frequency response: [`solutions/frequency_response.py`](solutions/frequency_response.py)

```text
34 frequency points with coherence > 0.6 between 0.15 and 4 Hz
identified vs model: magnitude error mean +0.36 dB, phase error mean -1.5 deg
```

Look at `frequency_response.png`: a near-perfect match up to about 3.3 Hz. Coherence
then collapses, because the sweep's taper ends and the actuator attenuates
high-frequency input. The same plot from a *closed-loop* sweep gives you
measured bandwidth and stability margins, which is a flight-readiness criterion.

---

## Exercises ([`exercises/sysid_ex.py`](exercises/sysid_ex.py))

- **A. Input design:** step vs doublet vs 3-2-1-1. Solution:
  [`solutions/sysid_ex.py`](solutions/sysid_ex.py). A step leaves α and δe 94%
  correlated, and every Cm derivative is 25–32% wrong.
- **B.** Double the instrumentation noise. Which method degrades more, and why?
- **C.** Output error for the Dutch roll (β, p, r, φ).

## Self-check

- Why does equation error need q̇, and what does differentiating noise do?
- What makes two regressors "collinear", and how do you design an input to avoid it?
- Fit R² = 0.93 on the 3-2-1-1. Why is that not evidence the model is right?
- When would you *fix* a derivative rather than estimate it?

## Done when

- [ ] You can explain every number in the equation-error table, including the bad ones
- [ ] Your output-error fit beats equation error on the held-out doublet
- [ ] You can design a sysID test card (inputs, amplitudes, trim points, validation maneuvers)
