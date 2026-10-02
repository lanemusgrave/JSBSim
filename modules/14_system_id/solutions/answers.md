# Module 14: answers

**A. Excitation design** (Cm derivatives by equation error, same peak amplitude):

| input | corr(α, δe) | Cmα err | Cmq err | Cmδe err |
|---|---|---|---|---|
| step (5 s) | −0.94 | 31 % | 26 % | 32 % |
| doublet | −0.57 | 13 % | 15 % | 8 % |
| 3-2-1-1 | −0.60 | 3 % | 23 % | 6 % |

With a step, the airplane settles to a new trim where α is almost a
linear function of δe (the trim relation!). The two regressors become nearly
collinear (r = −0.94), and least squares can't tell "Cmα" from "Cmδe": any
combination along that line fits. A good input excites the dynamics so
the states move **independently** of the input: broad frequency content
around the mode of interest. That is the point of 3-2-1-1s and sweeps. Cmq stays
hard in all three because the q regressor is small (c/2V ≈ 0.004) and comes
from a differentiated signal. (The condition numbers are huge partly because the
regressors aren't scaled; normalize columns before reading cond() literally.)

**B. Doubling the noise.** Equation error degrades more and becomes *biased*:
noise in the regressors (α, q) biases OLS toward zero (errors-in-variables),
and differentiating q amplifies its noise. Output error keeps the noise in the
outputs, where least squares assumes it is, so it is noisier but unbiased.
That's why output error (or filter error, with process noise) is the
flight-test standard for final models, with equation error used for start
values and for quick model-structure decisions.

**C. Lateral OE.** States [β, p, r, φ], parameters {CYβ, Clβ, Clp, Clr, Clδa,
Cnβ, Cnp, Cnr, Cnδr} plus biases; same `least_squares` pattern as
`output_error.py`. Expect Clp and Cnr to move toward truth, as Cmq did.

**Self-check.**
- *Why can't you trust OLS standard errors here?* They assume white residuals.
  Flight-data residuals are coloured (model-structure error, filtered signals),
  so the real uncertainty is several times larger. Cmq was 9 "standard errors" off.
- *Why did CLq and CLδe come out as nonsense?* Their contribution to CL is about
  1% of the signal, below the measurement noise. They are practically
  non-identifiable from this maneuver, and you'd fix them at wind-tunnel/CFD
  values (or a priori) rather than estimate them.
- *What does coherence < 0.6 mean?* The output isn't linearly explained by the
  input at that frequency (noise, nonlinearity, other inputs). Don't trust H(f) there.
