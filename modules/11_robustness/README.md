# Module 11: Robustness: limits, delay, noise, scheduling

**Time:** about 5 h · **Reading:** S&L §4.5–4.7 (robustness, sampled-data effects); Åström & Murray, *Feedback Systems* ch. 10–12 (free online) on PID, anti-windup and robustness; B&M §6.5 (implementation, anti-windup)

## Objectives

1. Re-implement the autopilot in Python and **validate** it against the XML
   reference before trusting it, as you would for any SIL or flight-software port.
2. Quantify what flight software does to a loop: **frame rate**, **latency**,
   **noise filtering**. Turn it into a latency budget.
3. See integrator **windup** and fix it two ways (clamping, back-calculation).
4. **Gain-schedule** on dynamic pressure and show it holds the closed-loop
   dynamics constant across speed.

---

## Refresher

| Effect | Phase cost at crossover ω<sub>c</sub> | Rule of thumb |
|---|---|---|
| sample-and-hold at T | ≈ ω<sub>c</sub>T/2 | frame rate ≥ 10–20× ω<sub>c</sub>/(2π) |
| pure latency τ | ω<sub>c</sub>τ | delay margin = PM/ω<sub>c</sub> must exceed the total latency with margin |
| first-order filter at ω<sub>f</sub> | atan(ω<sub>c</sub>/ω<sub>f</sub>) | ω<sub>f</sub> ≥ 3–5 ω<sub>c</sub> unless you designed it in |

**Windup:** when the actuator (or a command limit) saturates, an integrator
keeps integrating the error. After the error reverses, the stored value has to
unwind first, and the result is a big overshoot. Fixes: *clamping* (freeze the
integrator while saturated; JSBSim's PID `<trigger>`) or *back-calculation*
(feed the saturation error back into the integrator).

**Gain scheduling:** control power scales with q̄, so scale gains by q̄₀/q̄ (or
look them up in a table vs. q̄, Mach or altitude). This works if the schedule
variable changes slowly compared with the loop dynamics.

---

## Walkthrough

### 1. A Python autopilot, validated first

[`src/gnclab/controllers.py`](../../src/gnclab/controllers.py) `LongitudinalAP` is
the Module 09 control law in Python, with knobs for rate, latency, anti-windup,
gyro filter and gain schedule. It drives the **pilot** inputs. (The XML
autopilot's switches write `ap/...` every frame, so a second controller can't
share those properties.) [`solutions/python_vs_xml.py`](solutions/python_vs_xml.py):

```text
alt_ft        max |XML - Python| = 0.0625  (0.06 % of its range)
theta_deg     max |XML - Python| = 0.1     (0.68 % of its range)
elevator_rad  max |XML - Python| = 0.0292  (7.94 % of its range)   <- one-frame timing at the step edge
```

The two are equivalent. The only difference is a one-frame timing offset at the
step, because the Python controller computes from the state at the start of a
frame while XML components run inside it.

### 2. Frame rate and latency: [`solutions/rate_and_delay.py`](solutions/rate_and_delay.py)

```text
 rate  latency  effective delay  residual q osc  elevator rms
  120      0           4 ms          3.8 deg/s      1.0 deg
   25      0          20 ms         10.3            1.7
   10      0          50 ms         13.2            2.2
   50     80          90 ms         33.1            4.6     <- at the linear delay margin (~90 ms)
   50    100         110 ms         91.9           14.0     <- limit cycle
```

The linear delay margin predicted the cliff. The residual oscillation also
grows smoothly before it, so "stable" is not the same as "good".

### 3. Windup: [`solutions/antiwindup.py`](solutions/antiwindup.py)

A 400 ft climb saturates θ<sub>c</sub> for about 20 s:

```text
anti-windup ON : overshoot    7.0 ft, peak integrator    17 ft*s
anti-windup OFF: overshoot  237.0 ft, peak integrator  4667 ft*s
```

### 4. Noise vs. phase: [`solutions/noise_filter.py`](solutions/noise_filter.py)

```text
q filter none: linear PM 84.2 deg, delay margin 89 ms | RMS elevator rate (noise) 5.7 deg/s
q filter 5 Hz: linear PM 58.3 deg, delay margin 62 ms | RMS elevator rate (noise) 2.4 deg/s
q filter 2 Hz: linear PM 46.1 deg, delay margin 55 ms | RMS elevator rate (noise) 1.7 deg/s
```

The step response hardly changes because this loop has a lot of margin. The cost
shows up in the **margins**, which is where you'd look before flight.

### 5. Gain scheduling: [`solutions/gain_schedule.py`](solutions/gain_schedule.py)

```text
fixed    : closed-loop SP wn spans 12.9-31.0 rad/s, delay margin 55-140 ms
scheduled: closed-loop SP wn spans 19.3-22.8 rad/s, delay margin 85-102 ms
```

Without scheduling, the high-speed end loses a third of its delay margin.

---

## Exercises ([`exercises/robustness_ex.py`](exercises/robustness_ex.py))

- **A.** Implement **back-calculation** anti-windup as a subclass; compare with
  clamping and with none.
- **B.** Find the **latency budget** at a 25 Hz frame and write it as a requirement.
- **C.** Put the q̄ schedule into **your copy of the XML autopilot** with an
  `<fcs_function>`, and compare with the C172's `scheduled_gain` component.

## Self-check

- Your flight computer runs the pitch loop at 50 Hz with 30 ms of bus latency.
  Using this module's numbers, is that OK? What would you ask for?
- Why validate a re-implementation against the reference before using it?
- Name two places integrators hide in an autopilot besides the obvious PI loops.
- When does gain scheduling break down?

## Done when

- [ ] You can predict the delay cliff from the linear margins and confirm it in the sim
- [ ] Back-calculation works and you can explain why it beats clamping here
- [ ] You can state a latency budget for this airframe and defend it
