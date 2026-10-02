# Module 00: Setup and first run

**Time:** about 2 h · **Reading:** RM *Quickstart* (pp. 1–3) and RM §1 *Overview* (pp. 6–9)

## Objectives

By the end of this module you can:

1. Install JSBSim and check the install with one command.
2. Explain in two sentences what JSBSim is and where it sits in a GNC toolchain.
3. Run a JSBSim *script* two ways: from the command line and from Python.
4. Find the bundled aircraft, engines, systems and scripts on disk.
5. Use the repo's workflow: lesson → exercise → solution → test → commit.

---

## 1. Install

Follow [docs/setup.md](../../docs/setup.md). You are done when this prints
seven `[ OK ]` lines:

```powershell
python scripts\verify_install.py
```

## 2. What JSBSim is

JSBSim is an open-source **flight dynamics model (FDM)** written in C++. It is
a library that, given a vehicle description, integrates the 6-DOF rigid-body
equations of motion over a rotating, oblate Earth. The vehicle description
covers mass properties, aerodynamics, propulsion, ground reactions and the
flight control system, and every part of it is written in XML.

It has no graphics. A visual sim (FlightGear, Unreal) or a test harness
(Python, MATLAB, a SIL rig) drives it and reads its outputs. That is why GNC
teams use it: the plant model lives in data files that engineers can review,
and the same model runs in batch Monte Carlo, in a SIL loop and in a pilot
station.

```text
          ┌──────────────────── FGFDMExec.run(): one time step dt ────────────────────┐
 inputs ─►│ Propagate (integrate EOM) → Input → Inertial → Atmosphere → Winds →        │─► outputs
 (props,  │ Systems/FCS → MassBalance → Auxiliary → Propulsion → Aerodynamics →        │   (props,
 sockets) │ GroundReactions → ExternalReactions → Aircraft (sum forces) →             │    CSV, UDP)
          │ Accelerations → Output                                                     │
          │            all models read and write one shared PROPERTY TREE              │
          └────────────────────────────────────────────────────────────────────────────┘
```

Three ideas carry you through the rest of this curriculum:

| Idea | What it means | First seen |
|---|---|---|
| **Property tree** | Every quantity is a named property: `velocities/vc-kts`, `fcs/elevator-cmd-norm`, `aero/alpha-deg`... You read and write the simulation only through properties. | Module 01 |
| **XML configuration** | Aircraft, engines, systems (FCS/autopilot) and test scripts are all XML. A fork of JSBSim at work will mostly change *these* files, plus some C++. | Modules 03–04 |
| **Executive loop** | `run()` advances every model one time step `dt` (default 1/120 s) in a fixed order. | Module 01 |

Your bundled data lives in the installed package. Find it with:

```powershell
python -c "import jsbsim; print(jsbsim.get_default_root_dir())"
```

Look inside. `aircraft/` holds about 60 models, including `c172x`, `f16`, `T38`,
`T37`, `A4`, `737` and `ball`. The other folders are `engine/`, `systems/` and
`scripts/`. Open `aircraft/c172x/c172x.xml` in VS Code and scroll through it;
Module 04 takes it apart.

## 3. Run a script from the command line

A *script* (`<runscript>`) names an aircraft and initial conditions, then
lists *events*: "at t ≥ 3 s release the brakes", "when airspeed ≥ 51 kt
engage altitude hold". The bundled `scripts/c1723.xml` flies a C172 takeoff
and climb using the C172's built-in autopilot.

```powershell
$root = python -c "import jsbsim; print(jsbsim.get_default_root_dir())"
jsbsim --root=$root --script=scripts/c1723.xml
```

(bash: `root=$(python -c "import jsbsim; print(jsbsim.get_default_root_dir())")` and `--root=$root`.)

It runs 200 s of simulated flight in about a second and prints each event as it
fires. That is the "notify" output defined in the script. Open the script file in
the data folder and match each printed block to its `<event>`.

> `--root` tells the program where `aircraft/`, `engine/` and `scripts/` live.
> Relative script paths are resolved against it. In Python, `gnclab` handles
> this for you.

## 4. Run it from Python

```powershell
python modules\00_setup\solutions\first_flight.py --show
```

Read the script. It is short, and it is the pattern for everything that follows:

```python
fdm = load_script("scripts/c1723.xml")     # FGFDMExec + load_script + run_ic
df = run(fdm, 200.0, props, record_every=12)  # loop fdm.run(), record properties
```

`gnclab.load_script` and `gnclab.run` are about 20 lines each, in
[`src/gnclab/sim.py`](../../src/gnclab/sim.py). Open that file now; Module 01
has you write the same loop by hand.

Look at the plot (`outputs/00_setup/first_flight.png`). Things a pilot will
notice:

- On the runway the heading trace jumps between 0° and 360°. That is angle
  wrap-around. Every heading/course controller you write must handle it
  (Module 10).
- After the autopilot engages, airspeed oscillates with a period of about 15 s
  while the climb continues. Keep that in mind; Exercise 4 asks why.

## 5. Repo workflow

```text
modules/NN_topic/README.md    ← read
modules/NN_topic/exercises/   ← edit these (TODOs); run with --show
modules/NN_topic/solutions/   ← compare after you've tried
pytest -m "not slow"          ← quick regression check
git add -A; git commit -m "M00 exercises"   ← commit your work often
```

Git refresher, the part you'll use daily at work:

```powershell
git status                 # what changed?
git diff                   # how?
git switch -c m00-work     # do work on a branch
git add -A; git commit -m "Finish M00 exercises"
git switch main; git merge m00-work
git log --oneline --graph  # history
```

---

## Exercises

Edit [`exercises/first_flight_ex.py`](exercises/first_flight_ex.py):

1. Record pitch attitude and vertical speed as well.
2. Find the lift-off time (AGL > 10 ft).
3. Compute the average climb rate from 60 to 120 s in ft/min. How does it
   compare with what you'd expect from a C172 at full power?
4. The altitude hold uses the elevator and the throttle is fixed at full.
   What is controlling airspeed? Explain the oscillation in a comment.

Bonus: open `scripts/T38.xml` in the data folder (a T-38 taxi and engine-start
test). Run it from the command line and match each printed event to the XML.
Then change one event's trigger time in a *copy* of the script saved under
`modules/00_setup/exercises/`, and run your copy by passing its full path to
`--script`.

## Self-check

- What is the difference between JSBSim and FlightGear?
- What does `--root` do, and why does the Python API not need it?
- Name three folders in the JSBSim data directory and what they hold.
- What is the default integration time step, and where did you see it?

## Done when

- [ ] `verify_install.py` is all `[ OK ]`
- [ ] You ran c1723 from the CLI **and** from Python, and opened the plot
- [ ] Exercise answers are committed
