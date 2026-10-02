# Module 18: answers (self-check)

**10 m/s of wind instead of 5.** Cross-track (MR-2/3) and bank (MR-6) move: ground speed on
the downwind leg rises to 35 m/s, so the turn needs φ = atan(V<sub>g</sub>²/(gR)) ≈ 46° on a
120 m fillet, beyond the 30° bank limit (Module 12). Expect MR-2 failures at the
downwind corners, and possibly MR-1 against the headwind. Re-run the Monte Carlo with the new
forecast (about 2 minutes here). Better, run the wind-stratified Monte Carlo *before* the flight
(Module 15, exercise A) and brief a wind limit instead of a single forecast.

**MR-6 on the updated model.** Yes, brief it, as "retired by system ID". Show the
design-model failure rate, the drivers (roll damping, aileron power), the identified
values, and the margin that remains (worst 34.1° against 35°). That's thin. A
reviewer will ask what happens with the remaining Clp/Clda uncertainty plus
stronger turbulence, so run that sweep (Module 15 margin to failure) before
saying "closed".

**Cmq identified 0.61, actual 0.80.** It barely mattered here: the pitch loop has
large margins, and altitude errors are dominated by turbulence and payload
(MR-4 drivers). Without the answer key you'd know by: (1) Module 14's validation
(EE's Cmq is biased low on known-truth data), so inflate its σ, as the solution
does with 20%; (2) **output error** on the same maneuvers, which gave Cmq within 11% in
Module 14; (3) the sensitivity of the requirement metrics to Cmq in the Monte Carlo.
If no metric cares, its error is a documentation item, not a flight risk.

**"How do you know the simulation is right?"**
- the model is generated from a published parameter set and verified against the textbook
  linear model (Module 07: modes and eigenvalues);
- the Python control laws reproduce the XML ones (Module 11), and the SIL reproduces
  in-process bit-for-bit (Module 16);
- the linear delay margin predicted the nonlinear cliff (Modules 11 and 16);
- system ID on flight data, with validation on held-out maneuvers (Module 14), and
  the first-flight metrics sit inside the updated model's predicted distributions (this module);
- unit and regression tests run on every commit (CI).

And state the limits: what the sim doesn't model (memo section 8).
