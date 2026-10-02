# Curriculum: JSBSim for fixed-wing GNC

Nineteen modules in six phases, each building on the one before. Every module
leaves you with working code you will reuse later: the C172 lessons teach the
tool, then you build your own small UAS model (`gnc_trainer`, Module 07), and
from there on you design, verify and identify GNC for *that* vehicle.

**Time:** about 100 hours. At 10–12 h/week that is 8–9 weeks; at 5–6 h/week it
is about 4 months. Phases 0–2 alone (about 25 h) get you productive in a JSBSim
codebase.

**How each module works**

1. Read `README.md`: objectives, a *refresher* on the theory (key
   equations only, with pointers to the textbooks), the reading assignment, and a
   walkthrough.
2. Do the exercises in `exercises/`. They are runnable starter scripts with
   `TODO`s.
3. Compare with `solutions/`. Run any script with `--show` to see the plots.
4. Answer the *self-check* questions without looking anything up.
5. Tick the box below and commit.

**Reading abbreviations used in the modules**

| Abbrev. | Source |
|---|---|
| **RM** | *JSBSim Reference Manual* (Berndt et al.). PDF: <https://jsbsim.sourceforge.net/JSBSimReferenceManual.pdf>. Newer web edition: <https://jsbsim-team.github.io/jsbsim-reference-manual/> |
| **API** | JSBSim C++/Python API docs: <https://jsbsim-team.github.io/jsbsim/> |
| **B&M** | Beard & McLain, *Small Unmanned Aircraft: Theory and Practice* (Princeton, 2012). Free supplement site: <https://github.com/randybeard/mavsim_public> |
| **S&L** | Stevens, Lewis & Johnson, *Aircraft Control and Simulation*, 3rd ed. (Wiley, 2016) |
| **Nelson** | Nelson, *Flight Stability and Automatic Control*, 2nd ed. (McGraw-Hill, 1998) |
| **M&K** | Klein & Morelli, *Aircraft System Identification: Theory and Practice*, 2nd ed. (2016) |
| **Ogata / FPE** | Any undergraduate controls text (Ogata; Franklin, Powell & Emami-Naeini) |

You do not need to own all of these. RM and B&M cover about 70% of the
curriculum. S&L is the book to have on your desk at work.

---

## Phase 0 — Setup (day 1)

| ✓ | # | Module | Reading | You will… | h |
|---|---|---|---|---|---|
| ☐ | 00 | [Setup and first run](modules/00_setup/README.md) | RM *Quickstart* | install JSBSim, run a scripted C172 takeoff from the command line and from Python, learn the repo workflow | 2 |

## Phase 1 — JSBSim fundamentals (week 1)

| ✓ | # | Module | Reading | You will… | h |
|---|---|---|---|---|---|
| ☐ | 01 | [First simulation in Python](modules/01_first_simulation/README.md) | RM 2.1, 2.4; App. A *Native Properties* | drive `FGFDMExec` by hand: load, set ICs, step, read/write properties, record and plot; see how `dt` matters | 3 |
| ☐ | 02 | [Frames, units and the equations of motion](modules/02_frames_units_eom/README.md) | RM 2.2, 2.3; RM §3 *Formulation* | check α, β, γ and the Euler kinematics against the simulation; drop a ball and compare with the analytic answer | 4 |
| ☐ | 03 | [Scripts, events and output](modules/03_scripts_events_output/README.md) | RM ch. 4 *Scripting* | read and run the C172 autopilot scripts; write your own test-card script; log CSV and analyze it with pandas | 3 |
| ☐ | 04 | [Anatomy of an aircraft file](modules/04_aircraft_anatomy/README.md) | RM 2.5, 2.6, 3.1–3.4; Case Studies 1–2 | read the C172's aero build-up; build a ball, then a ball with a parachute, then a glider from scratch | 5 |

## Phase 2 — Flight dynamics refresher (week 2)

| ✓ | # | Module | Reading | You will… | h |
|---|---|---|---|---|---|
| ☐ | 05 | [Trim and performance](modules/05_trim_performance/README.md) | Nelson ch. 2–3; S&L 3.6 | trim in level, climbing and turning flight; build drag-polar and power-required curves; find the neutral point with a CG sweep | 4 |
| ☐ | 06 | [Linear models and dynamic modes](modules/06_linear_models_modes/README.md) | Nelson ch. 4–5; S&L 3.7–3.8 | linearize, pull out short period, phugoid, Dutch roll, roll and spiral; compare with the textbook approximations and with nonlinear doublets | 5 |
| ☐ | 07 | [Build your own UAS model](modules/07_build_uas_model/README.md) | RM 3.1–3.3; B&M ch. 3–4 | turn a published set of aero derivatives into a JSBSim aircraft (`gnc_trainer`) with electric propulsion; check it against an independent linear model | 6 |

## Phase 3 — Control laws (weeks 3–4)

| ✓ | # | Module | Reading | You will… | h |
|---|---|---|---|---|---|
| ☐ | 08 | [FCS building blocks and non-idealities](modules/08_fcs_components/README.md) | RM 2.7 | use actuators (rate limit, lag, deadband, hysteresis), sensors (noise, bias, drift, quantization, delay), filters, PID and switches; see what each one does to a pitch damper | 4 |
| ☐ | 09 | [Longitudinal autopilot](modules/09_longitudinal_autopilot/README.md) | B&M ch. 5–6; FPE ch. 5–6 | design pitch → altitude → airspeed loops by successive loop closure from the linear model (root locus, Bode, margins); implement them in JSBSim XML; verify in nonlinear simulation | 6 |
| ☐ | 10 | [Lateral-directional autopilot](modules/10_lateral_autopilot/README.md) | B&M ch. 6; RM Case Study 3 | build a yaw damper with washout, roll hold and course hold with coordinated turns; review the C172 wing leveler and heading hold | 5 |
| ☐ | 11 | [Robustness: limits, delay, noise, scheduling](modules/11_robustness/README.md) | S&L ch. 4.5–4.7 | add integrator anti-windup; compare Padé with pure delay; quantify delay margin; trade noise filtering against phase lag; gain-schedule on dynamic pressure; run the controller in Python at a fixed rate (SIL-style) | 5 |

## Phase 4 — Guidance and navigation (week 5)

| ✓ | # | Module | Reading | You will… | h |
|---|---|---|---|---|---|
| ☐ | 12 | [Guidance](modules/12_guidance/README.md) | B&M ch. 9–11; RM Case Study 4 | implement straight-line and orbit path following (vector field), a waypoint manager with fillets, and a simplified TECS | 6 |
| ☐ | 13 | [Navigation and state estimation](modules/13_navigation_estimation/README.md) | B&M ch. 7–8 | model IMU/GPS/baro/pitot sensors, build a complementary filter and an EKF, and fly the autopilot on *estimated* states | 6 |

## Phase 5 — Modeling, system ID, verification, flight test (weeks 6–7)

| ✓ | # | Module | Reading | You will… | h |
|---|---|---|---|---|---|
| ☐ | 14 | [System identification](modules/14_system_id/README.md) | M&K ch. 5–6, 9–10 | design doublet, 3-2-1-1 and sweep maneuvers; fly a noisy simulated flight test; estimate derivatives by equation error (OLS) and output error; build frequency responses with coherence; validate on held-out data | 7 |
| ☐ | 15 | [V&V: Monte Carlo and requirements testing](modules/15_monte_carlo_vv/README.md) | NASA-STD-7009A (overview) | write requirements; disperse mass, CG, aero, thrust, wind/turbulence, actuators and sensors; run batches in parallel (Windows-safe); compute pass/fail statistics; add pytest regression tests; write a flight-readiness summary | 6 |
| ☐ | 16 | [SIL harness and flight-test log analysis](modules/16_sil_log_analysis/README.md) | RM *Programmer's Manual* (I/O) | run JSBSim as a plant at a fixed rate with an external controller process over UDP plus added latency; write test cards; build a log-analysis toolkit (align, segment, metrics); run a tuning iteration | 6 |

## Phase 6 — Working in the fork (week 8)

| ✓ | # | Module | Reading | You will… | h |
|---|---|---|---|---|---|
| ☐ | 17 | [Working in a JSBSim fork (C++)](modules/17_jsbsim_fork/README.md) | RM *Programmer's Manual*; `src/` tour | build from source (WSL2 or VS 2022); tour `FGFDMExec`, `FGPropagate`, `FGFCS`, `FGFCSComponent`; add a new FCS component in C++; run it from a script; keep a fork in sync with upstream | 8 |

## Capstone (weeks 8–9)

| ✓ | # | Module | Reading | You will… | h |
|---|---|---|---|---|---|
| ☐ | 18 | [Capstone: end-to-end GNC for `gnc_trainer`](modules/18_capstone/README.md) | everything | fly a waypoint mission with the full stack (estimator → guidance → autopilot → actuators) with every non-ideality on; update the model from system ID; run a Monte Carlo against requirements; write a flight-readiness memo | 10 |

---

## Map: Reference Manual → modules

| Reference Manual section | Module(s) |
|---|---|
| Quickstart | 00 |
| User's Manual §2.1 Simulation, §2.4 Properties | 01 |
| §2.2 Frames of reference, §2.3 Units | 02 |
| §2.5 Math (functions, tables) | 04, 07 |
| §2.6 Forces and moments | 04, 07 |
| §2.7 Flight control and systems modeling | 08–11 |
| §3.1 Aircraft, §3.2 Engines, §3.3 Thrusters | 04, 07 |
| §3.4 Initialization | 01, 05 |
| §4 Scripting (events) | 03 |
| Programmer's Manual: class hierarchy, Python, extending | 01, 17 |
| Formulation Manual: equations of motion | 02, 06 |
| Case Study: Simple ball / ball with parachute | 02, 04 |
| Case Study: Wing leveler / heading hold (C172) | 03, 10 |
| Case Study: Waypoint navigation | 12 |
| Case Study: Rocket with GNC (J246) | 03 (optional exercise) |
| Appendix: Native properties | 01 and the [property cheat sheet](reference/properties.md) |

## Map: job skills → modules

| Skill | Where you practise it |
|---|---|
| Control laws that respect actuator limits, time delays and sensor noise | 08, 09, 10, **11**, 13 |
| Aero model development from data | 04, **07**, 14 |
| System identification | 06, **14** |
| Verification and validation: SIL, Monte Carlo, flight-readiness criteria | 11, **15**, **16**, 18 |
| Flight-test planning, log analysis, tuning | 03, **16**, 18 |
| Working in a JSBSim fork, code review | **17** |
