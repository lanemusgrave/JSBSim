# Module 17: answers

**A. The component.** The solution is the patch. Points reviewers look for:
- `ResetPastStates()` is overridden *and* calls the base class version (it resets the
  `<delay>` buffer that `FGFCSComponent` provides to every component).
- Bad input fails at load time with the file and line (`XMLLogException(element)`),
  not with a crash in flight.
- The output is computed by summing the window each frame rather than with a running
  sum. For N ≈ 10 it's just as cheap, and it can't accumulate round-off over a
  10-hour soak test. (A running sum with periodic re-summation is the optimization
  if N is large.)
- The test pins the *semantics*: window contents, N = 1 identity, `<clipto>`, and
  reset behavior.

**B. Sync.** `git log v1.3.1..upstream/master -- src/models/FGFCS.cpp
src/models/flight_control/FGFCSComponent.cpp src/models/flight_control/CMakeLists.txt`
lists, among others, a per-model execution-enable property, the thread-local
logger and `FGPropertyNode` removal. Some of these are already in v1.3.1 via
the release branch (cherry-picks), which is exactly why the plain rebase got
confused. Our patch touches only the if/else chain and the CMake lists, so the
`--onto` rebase applied without conflicts and compiled. Re-run the rig and
ctest anyway: "it compiled" is not "it works".

**C. Property-driven window (sketch, not in the patch).** Parse `<samples>` with
`FGParameterValue` (as `FGDeadBand` does for `<width>`), read it in `Run()`,
and when N changes either (a) refill the new buffer with the current mean (no
jump in the output: the safe choice in flight), or (b) keep the most recent
min(N_old, N_new) samples (closer to the "true" boxcar, but it can step).
Whichever you pick, the test should change N mid-run and assert the output
does exactly that. Also clamp N to ≥ 1 at run time, since a property can be
written to anything.

## Self-check

- **Run order:** Propagate first, then Systems/FCS, Aerodynamics, …, Accelerations.
  The FCS sees the state at the start of the frame. A command it computes changes forces
  this frame, but the *motion* only next frame, when Propagate integrates
  those accelerations. That's one frame of input-to-motion delay, inherent to the scheme.
- **Four registration points:** the class files, `flight_control/CMakeLists.txt`,
  the `FGFCS::Load` if/else chain, the `FGFCSComponent` type list. Plus the test in
  `tests/CMakeLists.txt`.
- **Priming:** zero-filling makes the output ramp up from 0 after every reset: a fake
  transient that an autopilot would react to. JSBSim's own lag filters and
  integrators also start from their stored state, so check what each starts at after
  `run_ic()` (and trim it). For a filter, initialize to the input.
- **Analysis tool and SIL disagree after a sync:** (1) are both using the same JSBSim
  build (`jsbsim.__file__`, the executable's `--version`/commit)? (2) do the logs show
  "Unknown FCS component" or "No property by the name"? (3) diff the run order and
  defaults that changed upstream (`git log` on the files you depend on), then
  re-run the SIL validation (bit-for-bit vs in-process) and the Monte Carlo regression.
- **Upstream or fork:** upstream anything general (bug fixes, generic components like
  this one) so you stop carrying it. Keep in the fork what is proprietary (vehicle
  data, company-specific interfaces) or not yet mature. Either way, the fork's
  delta should be small enough to rebase every release.
