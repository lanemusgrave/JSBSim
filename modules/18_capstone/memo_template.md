# Flight-readiness memo: <vehicle>, <mission>

**Date:** · **Author:** · **Reviewers:**

## 1. Recommendation

GO / GO with limits / NO-GO, in one sentence, then the 3 reasons that matter most.

## 2. Configuration under review

Vehicle (as built, mass/CG), software versions (autopilot, guidance, estimator, *JSBSim fork
commit*), mission, weather limits assumed.

## 3. Model status

What the simulation is based on, and what has been validated against flight or ground data:
aero (sysID table: derivative, design, identified, σ and why), mass properties (measured?),
propulsion, actuators (bench-tested?), sensors (bench-characterized?).

## 4. Requirements verification

| req | requirement | method (analysis / sim / MC / SIL / test) | result (pass rate, 95 % CI, worst) | margin |
|---|---|---|---|---|

## 5. Robustness

Monte Carlo set-up (dispersions, N, seeds), top drivers, margins to failure for the
drivers (in σ), and stability margins (gain, phase, *delay*) on the updated model.

## 6. Comparison with flight data (if any)

Where each flown metric falls in the predicted distribution, and what you changed because of it.

## 7. Operating limits

Wind, turbulence, weight & CG, bank / loiter radius, minimum airspeed, link and GPS requirements.

## 8. Not covered by this analysis

Failure modes, unmodeled dynamics, untested software paths. Say it before a reviewer does.

## 9. Open items

| item | owner | needed before flight? | closure evidence |
|---|---|---|---|
