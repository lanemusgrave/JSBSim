# Module 03: answers

**Exercise 1.** The c172x heading hold commands about **30° of bank** (it
saturates its roll command at about 30°), so the 90° turn takes about 25 s
(t = 15 → 40 s), roughly a standard-rate turn at 94 KCAS. In the climb to 4500 ft
the airspeed bleeds from about 94 to about 82 KCAS: the altitude hold only moves the
elevator, the throttle stays where the trim left it, and the climb is paid for
in kinetic energy. An autothrottle or a TECS-style energy controller fixes
that (Modules 09 and 12).

**Exercise 2.** Four events at t = 80, 81.2, 82.0 and 82.4 s, adding −0.05,
+0.10, −0.10, +0.10 and then −0.05 at 82.8 s to `fcs/elevator-cmd-norm` with
`type="FG_DELTA"` (3u = 1.2 s up, 2u = 0.8 s down, 1u up, 1u down, then back
to neutral).

**Self-check.**
- Persistent: fires each time the condition *becomes* true. Continuous:
  runs every step *while* it is true.
- `<set name="fcs/throttle-cmd-norm" value="1.0" action="FG_RAMP" tc="3"/>`
  on an event triggered while the throttle is 0.6.
- Test points must start from the same, known condition so the results are
  comparable and traceable. `do_simple_trim` *teleports* the state to an
  equilibrium (an instantaneous jump), while a pilot flies there with transients.
- The property didn't exist when the `<output>` was parsed (typo, or a
  script-local property). JSBSim prints a warning and logs nothing for it.
