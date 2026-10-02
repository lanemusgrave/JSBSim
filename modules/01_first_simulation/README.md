# Module 01: First simulation in Python

**Time:** about 3 h · **Reading:** RM §2.1 *Simulation* and §2.4 *Properties*; skim RM Appendix A *Native Properties*; API docs for [`FGFDMExec`](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGFDMExec.html)

## Objectives

1. Write the JSBSim loop by hand: create → load → ICs → `run_ic` → step → record.
2. Find, read and write properties, and know the difference between `ic/`,
   command and position properties.
3. Know what `dt` does and how to pick it.
4. Use `gnclab.make_fdm / initialize / run` and know what they do inside.

---

## Refresher: what "simulating" means here

JSBSim integrates the nonlinear 6-DOF rigid-body equations

$$
\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}, t), \qquad
\mathbf{x} = [\text{position}, \text{attitude quaternion}, \mathbf{v}_{body}, \boldsymbol{\omega}_{body}, \ldots]
$$

using fixed-step numerical integration. The defaults are rectangular Euler
for the rotational rate and attitude, Adams–Bashforth 2 for translational
velocity and Adams–Bashforth 3 for position. They are selectable through the
`simulation/integrator/...` properties (RM §2.1). Each `fdm.run()` call:

1. integrates the state forward one `dt` using the accelerations from the last step (`Propagate`),
2. runs every model in a fixed order (atmosphere → FCS → propulsion →
   aerodynamics → ground reactions → mass balance → accelerations ...).
   Each model reads properties and writes properties.

Sub-models such as an FCS `<channel>` can run at a lower rate than the
executive (Module 08). Real-time is irrelevant: JSBSim runs as fast as your CPU
allows, typically 300–2000× real time for one aircraft.

## Walkthrough

### 1. The loop, by hand

Read and run [`solutions/hand_loop.py`](solutions/hand_loop.py) (no helpers):

```python
fdm = jsbsim.FGFDMExec(jsbsim.get_default_root_dir())
fdm.load_model("c172x")
fdm["ic/lat-geod-deg"] = 33.67; fdm["ic/h-sl-ft"] = 4000; fdm["ic/vc-kts"] = 100
fdm.run_ic()
fdm["propulsion/set-running"] = -1
fdm.do_trim(1)
while fdm.get_sim_time() <= 120:
    ...record fdm["..."]...
    fdm.run()
```

The trimmed C172 flies straight and level until a half-second pull at t = 5 s,
and then you see two things you've felt in the airplane:

- **Phugoid:** about a 27 s period with light damping. Altitude and airspeed trade
  back and forth while pitch oscillates about ±10°.
- **A slowly building bank:** nothing in the airframe holds the wings level.
  Speed and power changes alter propeller torque and slipstream, the
  lightly damped spiral mode lets the bank wander, and the airplane ends up in a
  descending turn. Module 06 puts numbers on both modes.

> ⚠️ **Gotcha: IC order matters.** Set *position* (lat/long/altitude) before
> *airspeed*. Writing `ic/lat-geod-deg` after `ic/vc-kts` keeps the *true*
> airspeed and changes the calibrated one; in this example you'd start at
> 106 KCAS instead of 100. Swap the blocks in `hand_loop.py` to see it.

### 2. The property tree

Run [`solutions/property_explorer.py`](solutions/property_explorer.py). Key points:

| Family | Example | Meaning |
|---|---|---|
| `ic/` | `ic/vc-kts`, `ic/alpha-deg` | Initial conditions. **Only read at `run_ic()`.** |
| `fcs/*-cmd-norm` | `fcs/elevator-cmd-norm` | Pilot/autopilot *command*, normalized −1…+1 (throttle 0…1) |
| `fcs/*-pos-*` | `fcs/elevator-pos-rad` | Surface *position* after the FCS (gains, limits, actuator) |
| state / derived | `velocities/vc-kts`, `aero/alpha-deg`, `attitude/phi-rad` | Outputs of the models |
| `propulsion/` | `propulsion/engine/thrust-lbs` | Engines; `engine[1]` is the second one |
| `simulation/` | `simulation/sim-time-sec`, `simulation/dt` | Executive state |

- `fdm.query_property_catalog("alpha")` is grep for the property tree. You will
  use it constantly. `(R)` means read-only and `(RW)` means read-write.
- Units are always in the name: `-ft`, `-fps`, `-kts`, `-rad`, `-deg`,
  `-psf`, `-lbs`, `-slugs_ft3`, `-R` (Rankine), `-norm` (dimensionless).
- **Sign conventions** (standard aero, Module 02): positive elevator =
  trailing edge **down** = nose-down moment. So *pulling* is a **negative**
  elevator command. Each aircraft file defines how commands map to surface
  deflections, so always check the model's `<flight_control>` section and
  confirm the sign with a quick pulse test (Exercise 1 does exactly that for the
  aileron).

### 3. Time step

Run [`solutions/timestep_study.py`](solutions/timestep_study.py). The same
elevator pulse is flown at 20–1200 Hz. Typical output:

```text
    rate  dt [ms]  max |q err| [deg/s]  alt err @end [ft]  x real time
      20    50.00               6.27              2.32          1844
     120     8.33               2.13              0.60           328
     480     2.08               0.46              0.25            68
```

Part of the error at low rates is simply when the elevator step edge lands
(the input is sampled at `dt`), but the dynamics also change. Rules of thumb:

- Keep `dt` at least 10–20× faster than the fastest mode you care about. Gear
  contact and stiff actuators are usually the fastest. The JSBSim default of
  120 Hz is chosen for ground handling.
- Match your flight software's frame rate when you run a controller in the
  loop (Modules 11 and 16). The *controller* rate is what matters there.
- When results change with `dt`, you have a numerical problem, not a physics
  insight.

### 4. The helpers

From here on most scripts use the three helpers in
[`src/gnclab/sim.py`](../../src/gnclab/sim.py). Open the file: it contains
exactly the calls you just wrote.

```python
from gnclab import make_fdm, initialize, run
fdm = make_fdm("c172x")                                  # FGFDMExec + load_model
initialize(fdm, {"h-sl-ft": 4000, "vc-kts": 100})        # ic/... + run_ic + engines on
df = run(fdm, 30.0, {"kcas": "velocities/vc-kts"}, callback=my_pilot)  # -> pandas DataFrame
```

`callback(fdm, t)` runs before every step. It is your stick, your test-signal
generator, and later your Python flight controller.

---

## Exercises

1. **[`exercises/t38_roll.py`](exercises/t38_roll.py):** write the raw loop for the
   T-38 at 10,000 ft / 300 KCAS, trim it, and apply a 1 s aileron pulse. Log φ, p,
   β, r and total thrust. Why doesn't it roll back to wings level?
2. In `hand_loop.py`, comment out `fdm.do_trim(1)`. Describe in two sentences
   what happens and why.
3. Using `query_property_catalog`, find the C172's flap command and position
   properties. Lower the flaps 0.33 at t = 5 s in the hand loop (no elevator
   pulse) and describe the response.

## Self-check

- What does `run_ic()` do that `run()` doesn't? What happens if you write
  `ic/h-sl-ft` mid-flight?
- Why might `fcs/elevator-pos-rad` lag `fcs/elevator-cmd-norm`?
- A colleague's sim gives different results at 60 Hz and 120 Hz. What do you
  check first?
- In which order should you set lat/long, altitude and airspeed ICs, and why?

## Done when

- [ ] You can write the JSBSim loop from memory
- [ ] `t38_roll.py` runs and you can explain the final bank angle
