# Module 01: answers to the written exercises

**Exercise 2: no trim.** Without `do_trim`, the controls are at their default
values (elevator 0, pitch trim 0, throttle 0) and the attitude is whatever the
ICs imply (θ = α = 0). Nothing balances. The C172 pitches up to almost 30° in
3 s, rolls past 60° from the unbalanced propeller torque, stalls, and ends in a
steep spiral dive (about 150 KCAS and θ ≈ −30° by 30 s). A trim is the
equilibrium the linear analysis and the controllers are built around (Module 05).

**Exercise 3: flaps.** `fcs/flap-cmd-norm` (command, 0–1) drives
`fcs/flap-pos-deg` (position) through a rate-limited actuator, so the
flaps take a few seconds to reach 10°. Flaps add lift (ΔCL) at the same
angle of attack, so the airplane balloons: it climbs about 200 ft
and loses about 25 kt, then settles into a phugoid around a new trim at a lower
speed. The wings again drift into a bank, for the same reasons as in the
elevator-pulse case. Pilots know this: lowering flaps in a trimmed airplane
needs forward pressure and a re-trim.

**Self-check.**
- `run_ic()` copies `ic/...` into the state, resets the integrators and sets
  time to 0; `run()` integrates one step. Writing `ic/` mid-flight does nothing
  until the next `run_ic()`.
- The elevator position lags the command because the FCS contains a
  rate-limited actuator and possibly a lag filter (look for `<actuator>` in the
  c172x file). Module 08 is all about that.
- 60 vs 120 Hz giving different answers: check for stiff dynamics (gear,
  fast actuators/filters relative to dt), discontinuous inputs sampled at
  different times, and FCS components whose discretization depends on dt.
- Order: position (lat/long), then altitude, then airspeed. Airspeed
  conversions (KCAS ↔ KTAS) depend on altitude and atmosphere, and setting
  position after airspeed re-derives the CAS from the preserved true airspeed.
