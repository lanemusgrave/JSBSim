# Module 08: FCS building blocks and non-idealities

**Time:** about 4 h · **Reading:** RM §2.7 *Flight Control and Systems Modeling* (pp. 25–32) and the component reference in the API docs ([FGActuator](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGActuator.html), [FGSensor](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGSensor.html), [FGFilter](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGFilter.html), [FGPID](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGPID.html), [FGSwitch](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGSwitch.html))

## Objectives

1. Use every JSBSim FCS component: actuator, sensor, filters, PID, gain,
   summer, switch, function, kinematic.
2. Model the **non-idealities** your control laws must survive: lag, rate
   limit, deadband, hysteresis, travel limit, transport delay, noise, bias,
   drift, quantization.
3. Close your first loop (a pitch damper) on `gnc_trainer` and *see* what
   actuators, sensors and latency do to it, linear and nonlinear.
4. Fly your own autopilot XML on the shared airframe (`systems_dir`).

---

## Refresher: why these effects matter

Every element between "control law output" and "surface moves", and between
"airplane moves" and "control law input", adds **phase lag** (and sometimes
gain change or nonlinearity):

| Effect | Phase at ω | Typical small UAS |
|---|---|---|
| first-order lag, bandwidth a | −atan(ω/a) | servo 20–60 rad/s |
| pure delay τ | −ωτ (rad), no gain change | 10–50 ms compute + bus + sensor |
| rate limit R, amplitude A | none until Aω > R, then large lag and gain loss | 200–600 °/s servos |
| hysteresis/backlash | lag ∝ width/amplitude; limit cycles | linkages |

Phase margin is what the loop has to spend. **Delay margin = PM / ω<sub>c</sub>**: a
loop crossing over at 10 rad/s with 45° PM goes unstable with 78 ms of extra
delay. That's the number that tells you whether your flight computer is fast
enough (Module 11 goes deeper).

## Component cheat sheet (JSBSim XML)

```xml
<actuator name="..."> <input/> <lag> 30 </lag> <rate_limit> 2 </rate_limit>
          <deadband_width/> <hysteresis_width/> <bias/> <clipto><min/><max/></clipto> <delay type="time"> 0.05 </delay> </actuator>
<sensor name="..."> <input/> <lag/> <noise variation="ABSOLUTE|PERCENT" distribution="GAUSSIAN|UNIFORM"> σ </noise>
        <bias/> <drift_rate/> <quantization><bits/><min/><max/></quantization> <delay/> </sensor>
<lag_filter> C1/(s+C1)   <lead_lag_filter> (C1 s+C2)/(C3 s+C4)   <washout_filter> s/(s+C1)
<second_order_filter> (C1 s²+C2 s+C3)/(C4 s²+C5 s+C6)   <integrator> C1/s
<pid> <kp/> <ki type="rect|trap|ab2|ab3"/> <kd/> <trigger> prop </trigger>   (trigger > 0 freezes the integrator: anti-windup; < 0 resets it to 0)
<pure_gain> <summer> (a "-" before an input negates it) <switch> <fcs_function> <kinematic>
```

Any parameter can usually be a **property** instead of a number (`<gain> ap/kq </gain>`),
so you can tune and gain-schedule at run time.

> ⚠️ **Two JSBSim gotchas found while building this module**
> 1. **`<delay>` is parsed by every component but only applied by `<actuator>`,
>    `<sensor>` and `<switch>`** (v1.3.1). On a `pure_gain`, filter or summer it is
>    silently ignored. Use a noise-free `<sensor>` as a delay element.
> 2. **All `<system>`s run before `<flight_control>`, regardless of file order.**
>    That's why `gnc_trainer` sums its throttle inside the motor system, and why
>    the autopilot is a `<system>` whose `ap/...-cmd-norm` outputs feed the
>    `<flight_control>` summers in the same frame.

## How `gnc_trainer`'s avionics are wired (built in this module)

```text
 truth ──► Systems/gnc_sensors.xml ──► fb/...        (sensors/enabled = 0: truth, 1: sensor models)
                                         │
 fcs/gnc_autopilot.xml (swappable) ◄─────┘ ──► ap/elevator|aileron|rudder|throttle-cmd-norm
                                                      │
 pilot fcs/*-cmd-norm + trim ──► <flight_control> ◄───┘ ──► actuator (fcs/actuators-on = 1) ──► surfaces
```

- [`aircraft/gnc_trainer/Systems/gnc_sensors.xml`](../../aircraft/gnc_trainer/Systems/gnc_sensors.xml):
  gyros, AHRS, pitot, baro and GPS course with representative MEMS numbers.
  Read it.
- `make_fdm("gnc_trainer", systems_dir=...)` swaps in **your** `gnc_autopilot.xml`
  without touching the airframe. That's how every autopilot module works from
  here on, and how teams keep one plant model and many control-law versions.
- Everything defaults to **off/ideal**, so trim and `linearize_fd` see the bare
  airframe. Switch effects on *after* trimming.

---

## Walkthrough

### 1. The component rig: [`solutions/fcs_rig.py`](solutions/fcs_rig.py)

[`aircraft/m08_fcs_rig`](../../aircraft/m08_fcs_rig/m08_fcs_rig.xml) drives one
input through every component type. Look at the four figures in
`outputs/08_fcs_components/`, especially the actuators:

- a **rate limit** turns a 0.5 Hz sine into a lagging, shrunken triangle
  (that's rate-limited PIO);
- **hysteresis** leaves the output stuck 0.1 away from zero after the step
  returns (the C172 elevator problem from Module 06);
- a **deadband** of width 0.2 subtracts 0.1 from everything.

### 2. A pitch damper: [`solutions/pitch_damper.py`](solutions/pitch_damper.py) + [`solutions/fcs/gnc_autopilot.xml`](solutions/fcs/gnc_autopilot.xml)

δe = K<sub>q</sub>·q. The linear design picks K<sub>q</sub> for ζ<sub>sp</sub> = 0.7, then predicts:

```text
open-loop short period zeta = 0.44
design gain for zeta_sp = 0.7 (ideal): Kq = 0.50 (12.5 deg elevator per rad/s)
   zeta_sp + actuator (40 rad/s)        = 0.72
   zeta_sp + actuator + 0.1 s delay     = 0.54
   first unstable gain + actuator + 0.1 s delay : 0.97
```

and the nonlinear flights confirm it (`pitch_damper.png`):

```text
no damper              peak |q| 23.2 deg/s, residual q after 1.5 s  3.14 deg/s
damper, ideal          peak |q| 13.5 deg/s, residual                0.34 deg/s
+ actuators + sensors  peak |q| 14.3 deg/s, residual                0.34 deg/s
+ ... + 0.1 s delay    peak |q| 21.4 deg/s, residual                3.67 deg/s
4x gain + act + delay  sustained 41 deg/s limit cycle, elevator hitting its limits
```

Realistic actuators and sensors cost almost nothing here. **100 ms of delay
erases the damper's benefit**, and a gain that is harmless without delay becomes
a sustained oscillation, bounded only by saturation. Remember that picture when
someone asks "does the autopilot really need to run at 100 Hz?"

---

## Exercises ([`exercises/fcs_ex.py`](exercises/fcs_ex.py))

- **A.** Identify an unknown actuator's rate limit, lag, delay and travel from
  step responses alone. Why do you need two step sizes?
- **B.** Find the delay-induced stability limit of the damper in the nonlinear
  sim and compare with the linear prediction (0.97).
- **C.** Add a 20 rad/s gyro noise filter to your own copy of the autopilot.
  What does it buy and what does it cost?

## Self-check

- A servo has 40 rad/s bandwidth. How much phase does it cost at a 10 rad/s
  crossover? What about a 30 ms delay at the same frequency?
- Why does a rate limit cause oscillations that the linear analysis doesn't predict?
- Why do we trim with actuators and sensors **off**, then switch them on?
- What's the difference between a washout filter and a lag filter, and where
  would you use each in a yaw damper?

## Done when

- [ ] You can write any component from memory and predict its step response
- [ ] Your answers to A–C are within about 10% of the solutions
- [ ] You can explain delay margin to a colleague in one sentence
