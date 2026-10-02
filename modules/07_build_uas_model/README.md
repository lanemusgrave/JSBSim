# Module 07: Build your own UAS model (`gnc_trainer`)

**Time:** about 6 h · **Reading:** B&M ch. 3 (kinematics and dynamics) and ch. 4 (forces and moments) plus the propulsion addendum on the [mavsim_public](https://github.com/randybeard/mavsim_public) site; RM §3.1–3.3 (aircraft, engines, thrusters)

## Objectives

1. Turn a published set of aerodynamic, mass and propulsion data into a JSBSim
   aircraft **with a generator script**, the same workflow real aero databases use.
2. Know the JSBSim gotchas that bite when you do: product-of-inertia sign,
   units on every number, rate non-dimensionalization, and engine/trim interactions.
3. **Verify** the model against an independent implementation (trim + every
   mode), and know why trim agreement alone proves almost nothing.
4. End up with `aircraft/gnc_trainer`: the vehicle you'll design autopilots,
   guidance, estimators and Monte Carlo campaigns for in Modules 08–18.

---

## The vehicle

An Aerosonde-class UAS from Beard & McLain: 11 kg, 2.9 m span, 0.55 m² wing,
20 in propeller on a DC motor, cruise about 25 m/s.

```text
modules/07_build_uas_model/solutions/build_gnc_trainer.py   ← parameters + generator (the source of truth)
          │ writes
          ▼
aircraft/gnc_trainer/gnc_trainer.xml        airframe, mass, aero tables, motor-prop system, FCS
aircraft/gnc_trainer/Engines/*.xml          zero-power placeholder engine + "direct" thruster
aircraft/gnc_trainer/fcs/gnc_autopilot.xml  autopilot (empty now; Modules 09-12 fill it)
```

**Never edit the generated XML by hand.** Change the generator and re-run it.
Read the generator top to bottom. The highlights:

| Piece | How it's modeled | Why |
|---|---|---|
| C<sub>L</sub>(α), C<sub>D</sub>(α) | 1-D tables generated from B&M's blended linear + flat-plate stall model | Tables are how wind-tunnel/CFD data arrive. The generator could just as well read a CSV. |
| Rate derivatives | `qbar·S·c̄ · (c̄/2V)·q · C_mq` via `aero/ci2vel`, `aero/bi2vel` | Non-dimensional rates. Forgetting them is Bug 2 in the exercise. |
| Inertia | `negated_crossproduct_inertia="false"` + textbook J<sub>xz</sub> | JSBSim's *default* expects the opposite sign of I<sub>xz</sub>, a classic roll/yaw coupling bug. |
| Propulsion | B&M's DC-motor + propeller model, solved **algebraically** in a `<system>` and applied through `<external_reactions>` (thrust force and reaction torque) | See below. |
| FCS | normalized commands + trim → `aerosurface_scale` → optional **actuator** (lag 40 rad/s, 250°/s rate limit) selected by `fcs/actuators-on` | Ideal surfaces for linearization, realistic ones for flight (Module 08). |
| Uncertainty hooks | `uncertainty/<coef>-scale` multipliers (default 1) on 11 aero/thrust terms, a 0 kg `PAYLOAD` point mass, actuator lag and rate as properties | Added for the Monte Carlo in Module 15: build the dispersion knobs into the model from day one. |

### A war story: why not JSBSim's `<electric_engine>` + `<propeller>`?

The first version used them. It produced **NaN** at the first time step and
would not trim. Reading the C++ (`FGPropeller::Calculate`,
`FGPropulsion::GetSteadyState`) showed two problems:

1. At 0 RPM the propeller divides excess power by 1.0 instead of ω, so full
   power on a small rotor jumps it to about 29,000 RPM in one 8 ms step. An
   electric engine has no `InitRunning()` to start it spinning.
2. JSBSim's trim spins propellers up by **time-marching with a fixed 0.5 s
   step**, which is only stable if the rotor's spin-up time constant is above about
   0.25 s. A small electric motor's is about 0.05 s.

Making the rotor inertia 25× too large "fixes" both, but gives a 10–20 s
throttle response and gyroscopic moments larger than the aerodynamic ones.
The solution is the book's motor/prop model as algebraic equations, plus a
zero-power placeholder engine so the trim routine still has a throttle. The
lesson for working in a fork: **when the sim misbehaves, read the source.**
Module 17 shows you how to navigate it.

---

## Walkthrough

1. Generate the model (already done; re-run after any change):
   `python modules/07_build_uas_model/solutions/build_gnc_trainer.py`
2. Read [`solutions/reference_model.py`](solutions/reference_model.py): about 100 lines
   of numpy implementing the same equations (B&M ch. 3–4) **independently** of
   JSBSim, importing the same parameter table.
3. Run [`solutions/verify_gnc_trainer.py`](solutions/verify_gnc_trainer.py):

   ```text
   Trim at 25.0 m/s          JSBSim  reference
   alpha [deg]               3.0796     3.0859
   elevator [deg]           -7.7420    -7.7596
   throttle                  0.7812     0.7807

   Modes at 25 m/s      JSBSim eig        reference eig     wn err %
   roll subsidence      -21.45            -21.45             0.01
   short period         -4.685+9.660j     -4.685+9.659j      0.01
   Dutch roll           -1.123+4.561j     -1.104+4.560j      0.12
   phugoid              -0.140+0.480j     -0.140+0.480j     -0.12
   spiral               +0.090            +0.091            -0.35
   level-flight trim exists from 18 to 32 m/s at 100 m
   CLmax (alpha table) 2.42 -> 1 g stall speed 11.5 m/s
   static margin = -Cma/CLa = 0.49 cbar (49 %)
   ```

   Two independent implementations agree to 0.35% on every mode. Things a
   reviewer would flag, and you should note for later:
   - **The spiral is unstable** (λ = +0.09/s, time to double ≈ 8 s). Normal
     for this airframe; the roll loop is the first autopilot loop you'll close.
   - **C<sub>Lmax</sub> = 2.4 is unrealistic.** The book's stall blend (α₀ = 27°) keeps C<sub>L</sub>
     linear far too long. In practice the elevator runs out first (trim fails
     below 18 m/s at −25°). Don't use this model for stall/spin work.
   - **49 % static margin is very large** (the book's C<sub>mα</sub> is very
     negative). It makes a stiff, heavily trimmed airplane, which is great for
     learning controls and unlike most real UAS.

---

## Exercises

1. **[`exercises/bug_hunt.py`](exercises/bug_hunt.py):** a colleague's copy of
   the model with three injected bugs (the kind that really happen). Use the
   trim and mode comparison to form hypotheses, then diff the XML to confirm.
   Notice that **all three trim exactly like the correct model.**
2. Change one parameter in the generator, `Cma` from −2.74 to −1.0 (a more
   typical static margin of about 18%). Regenerate, re-run the verification, and
   note how the short period, the trim elevator and the trim speed range move.
   **Change it back** afterwards: later modules assume the book values.
3. Replace the C<sub>D</sub>(α) table with a polar that uses the *full* C<sub>L</sub> (including
   C<sub>Lq</sub>, C<sub>Lδe</sub>) the way you did for the glider in Module 04. Update the
   reference model to match and re-verify. (Restore afterwards.)

## Self-check

- Name four ways a JSBSim aero model can be wrong and still trim perfectly.
- Why do we keep the placeholder engine at all?
- What does `negated_crossproduct_inertia` do, and what's its default?
- The verification uses `linearize_fd`. Why couldn't we use JSBSim's
  `FGLinearization` here? (Hint: what state does it assume the engine has?)

## Done when

- [ ] `verify_gnc_trainer.py` shows < 1% mode differences
- [ ] You found all three bugs and wrote the automatic check for each
- [ ] You can explain the propeller war story to a colleague
