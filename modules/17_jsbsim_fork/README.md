# Module 17: Working in a JSBSim fork (C++)

**Time:** about 8 h · **Reading:** RM Programmer's Manual (class hierarchy, "Extending JSBSim"); [`docs/source_build.md`](../../docs/source_build.md) (build, code tour, git workflow); skim `src/models/FGFCS.cpp`, `flight_control/FGFCSComponent.h` and `FGDeadBand.cpp`; Pro Git ch. 3.6 (rebasing), free online

## Objectives

1. Build JSBSim from source, both the C++ executable and the Python module, and run
   upstream's test suite.
2. Know your way around the source: the executive, the run order, where FCS
   components live and how one is born, parsed, run and reset.
3. **Add a feature in C++**: a new `<moving_average>` FCS component, with an
   upstream-style test, as a clean, reviewable patch.
4. Keep a fork healthy: feature branches, **syncing with upstream** (rebase
   `--onto`), and making sure the team's Python tools use the fork.

This is the part of the job that's specific to "we develop in a fork of JSBSim".

---

## Refresher: the life of an FCS component

```text
load_model()  FGFCS::Load        for each <channel>: for each element:
                                   "deadband" -> new FGDeadBand(fcs, element)      <- the if/else chain
              constructor         FGFCSComponent(...) parses name, <input>, <clipto>, <output>
                                  your code parses its own elements, then bind() creates the property
run_ic()      ResetPastStates()   clear filter memory / integrators
run()         Run()               every frame (or every N frames: <channel execrate="N">), in file order:
                                  Input = InputNodes[0]->getDoubleValue(); ... Output = ...;
                                  Clip(); SetOutput();
```

Per frame, `FGFDMExec::Run()` executes: **Propagate** → Input → Inertial →
Atmosphere → Winds → **Systems/FCS** → MassBalance → Auxiliary → Propulsion →
**Aerodynamics** → Ground/External reactions → Aircraft → **Accelerations** →
Output. Propagate goes *first*, using the previous frame's accelerations.
That's the one-frame delay you measured in Module 06, explained by the source.

## The feature: `<moving_average>`

```xml
<moving_average name="fcs/q-avg">
  <input> velocities/q-rad_sec </input>
  <samples> 12 </samples>
</moving_average>
```

A boxcar over N samples has a pure delay of (N−1)/2 samples and **spectral nulls at
multiples of 1/(N·dt)**. 12 samples at 120 Hz removes 10 Hz exactly: a
vibration filter for a known rotor or prop frequency. Real avionics do this
(and the lag it adds is the price; Module 11).

The test rig ([`rig/`](rig/aircraft/m17_rig/m17_rig.xml)) drives it with a step plus a 10 Hz sine.

## Walkthrough

### 1. Build ([`docs/source_build.md`](../../docs/source_build.md))

```bash
bash scripts/build_jsbsim_source.sh     # clone v1.3.1 into external/jsbsim, build the executable  (~1 min)
bash scripts/build_jsbsim_python.sh     # Python module in external/pyfork-venv + upstream ctest     (~4 min)
```

On Windows, run these inside WSL2. Upstream's suite: **58/58 pass** (57 upstream + ours),
but only when run serially (see the gotchas).

### 2. The patch: [`solutions/0001-Add-moving_average-FCS-component.patch`](solutions/0001-Add-moving_average-FCS-component.patch)

```text
 src/models/FGFCS.cpp                          |   3 +    <- parse "moving_average"
 src/models/flight_control/CMakeLists.txt      |   6 +-   <- compile the new files
 src/models/flight_control/FGFCSComponent.cpp  |   2 +    <- its Type name
 src/models/flight_control/FGMovingAverage.cpp | 108 +++  <- the component
 src/models/flight_control/FGMovingAverage.h   |  66 +++
 tests/CMakeLists.txt                          |   3 +-   <- register the test with ctest
 tests/TestMovingAverage.py                    |  57 +++  <- the test
 tests/moving_average.xml                      |  18 +++
```

Apply it to your checkout with `cd external/jsbsim && git am ../../modules/17_jsbsim_fork/solutions/0001-*.patch`,
or better, write it yourself from the skeleton (exercise A).

### 3. Verify: [`solutions/check_moving_average.py`](solutions/check_moving_average.py)

```text
A. fork C++ executable
  external/jsbsim/build/src/JSBSim: max |avg - numpy boxcar| = 4.44e-16;  10 Hz residual after the step: 1.25e-14
B. stock pip wheel (jsbsim 1.3.1)
  Unknown FCS component: moving_average
  load_model returned True: the model LOADS.
  ...but 'No property named rig/avg': the unknown component was dropped with only a log message.
C. fork Python module
  jsbsim 1.3.1 from external/jsbsim/build-py/tests/jsbsim/__init__.py
  fork Python module: max |avg - numpy boxcar| = 4.44e-16;  10 Hz residual after the step: 1.28e-14
```

**Part B is the important lesson.** The stock JSBSim doesn't refuse a model with an
unknown component. It logs one line, drops the component, and carries on. Anything
reading that property then fails (or worse, a `<property>` declared elsewhere
reads a constant 0). If an analyst's laptop has the pip wheel and the team's
models use fork-only features, their Monte Carlo is quietly wrong. Pin the fork's wheel in the team's
requirements, and make model loading fail loudly in CI.

> 🐛 **Gotchas found while building this module**
> - **`invalid use of incomplete type FGFCS`:** the component's .cpp must
>   `#include "models/FGFCS.h"`. The header only forward-declares it.
> - **Type "UNKNOWN":** `FGFCSComponent`'s constructor has its *own* list of
>   element names. Register the new one there too, or the debug output says UNKNOWN.
> - **A model needs `<aerodynamics>`** even if it's a test rig ("A proper axis type
>   has NOT been selected"): give it one dummy term.
> - **ctest:** the tests need pandas and scipy, plus the `fpectl` target (build everything),
>   and they **must run serially**. `ctest -j4` gave 3–6 false failures from shared files.
> - **Output paths:** files go relative to `--root`. v1.3.1's `--outputlogfile` did
>   not redirect an `<output>` defined in the aircraft file (on current master it does).
> - **Reset semantics:** `reset_to_initial_conditions()` returns declared
>   `<property>` values to their defaults *and* calls `ResetPastStates()`. Our
>   first test assumed otherwise and failed. Tests teach you the API.

### 4. Sync with upstream

The fork branch was cut at tag v1.3.1. Upstream `master` is a year ahead.
`git rebase upstream/master` tried to replay upstream's own release-branch history
(v1.3.1 is not on master) and conflicted on dozens of files. This replayed only our commit, cleanly:

```bash
git fetch upstream master
git rebase --onto upstream/master v1.3.1 gnc/moving-average
```

The rebased patch built against master (2026-10-01) and gave identical rig results.

---

## Exercises

- **A. Write the component** from the skeletons in [`exercises/`](exercises/): `FGMovingAverage.h/.cpp`
  with TODOs, plus the three registration points listed at the top of the .cpp. Build it and
  run `check_moving_average.py` until A passes. Then write `tests/TestMovingAverage.py`
  in upstream's style (copy `TestDeadBand.py`) and get `ctest -R TestMovingAverage` green.
  Solution: the patch.
- **B. Sync:** rebase your branch onto upstream master, rebuild, re-run the rig and ctest.
  Record what changed upstream in the files you touched (`git log v1.3.1..upstream/master -- src/models/FGFCS.cpp`).
- **C. Extension:** make `<samples>` accept a *property*, so the window can change in flight.
  Decide what happens to the buffer when N changes, and write the test that pins that decision.

## Self-check

- Where in `FGFDMExec::Run()` do the FCS and Propagate run, and what does that imply for
  input-to-motion delay?
- What are the four places a new FCS component must be registered?
- Why prime the buffer at reset instead of zero-filling it? Name a component in
  JSBSim that has a start-up transient and how you'd avoid it.
- Your analysis tool and your SIL rig disagree after a fork sync. List the
  first three things you check.
- When should a fork change go upstream, and when should it stay in the fork?

## Done when

- [ ] You've built the executable and the Python module from source, and run ctest green
- [ ] Your own `<moving_average>` passes the rig check and an upstream-style test
- [ ] You can rebase a fork branch onto a new upstream and explain `--onto`
- [ ] You can explain part B to a colleague, and the CI check that prevents it
