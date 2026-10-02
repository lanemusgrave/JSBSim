# Module 07: answers

## Bug hunt

| # | Bug | Symptom in the comparison | Automatic check that catches it |
|---|---|---|---|
| 1 | `negated_crossproduct_inertia="false"` dropped. JSBSim's default then reads I<sub>xz</sub> with the opposite sign | Trim identical; **Dutch roll and spiral** change (roll–yaw coupling goes through I<sub>xz</sub>) | Compare the inertia tensor JSBSim reports (`inertia/ixz-slugs_ft2`) with the mass-properties source; a lateral mode comparison against an independent model |
| 2 | `aero/bi2vel` missing from C<sub>lp</sub> | **Roll mode λ goes from −21 to −378 /s.** Roll damping is too large by 2V/b ≈ 17× | Unit-consistency lint: every rate derivative must contain `bi2vel`/`ci2vel`; mode comparison |
| 3 | Chord typed with `unit="FT"` (0.19 ft instead of 0.19 m) | **Trim unchanged** (all pitching moments scale by c̄, so the C<sub>m</sub> = 0 balance is the same) but **short period ω halves**: M<sub>α</sub>, M<sub>δe</sub> scale with c̄ and M<sub>q</sub> with c̄² | Check `metrics/cbarw-ft` against the drawing; short-period frequency against wind-tunnel/flight data |

The lesson: **trim agreement proves very little.** All three bugs trim exactly
like the correct model. Dynamic checks (modes, frequency responses, doublet
matches against flight data) are what catch model errors. That's why system ID
(Module 14) and independent reference models exist.

## Self-check
- Why is generating XML from a script better than typing it? One source of
  truth (the parameter table), reviewable diffs, unit conversions done once,
  and an independent model can import the same parameters.
- Why does the trim α match to 0.006° but the aileron trim differs by 0.03°?
  The aileron trim balances the propeller torque. Small differences in how
  each model evaluates the advance ratio (JSBSim uses u<sub>aero</sub>, the reference
  uses u) and in gravity/density move it slightly. The modes agree to 0.35%.
- The spiral is unstable (time to double ≈ 8 s). Fine for an autopiloted UAS,
  but the roll loop must work before anything else, and a lost-link
  "wings-level" mode is a safety requirement.
