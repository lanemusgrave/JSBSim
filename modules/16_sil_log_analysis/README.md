# Module 16: SIL harness and flight-test log analysis

**Time:** about 6 h · **Reading:** RM Programmer's Manual, the sections on the socket interfaces (`<input>`/`<output type="SOCKET">`) for how JSBSim itself talks to other programs; any flight-test text's chapter on data reduction (e.g. Kimberlin, *Flight Testing of Fixed-Wing Aircraft*, ch. 3); the Python `socket` and `multiprocessing` docs (the "spawn" start method)

## Objectives

1. Run the control law as **separate flight software** talking to the JSBSim
   plant over UDP, the way a SIL rig does. **Validate the rig** before using it.
2. Measure what **transport delay, frame timing and jitter** do to the loop,
   and connect it to the delay margin from Module 11.
3. Reduce flight-test logs: **align clocks, find test points and events, score
   against requirements, find maneuvers**.
4. Run a **tuning iteration**: log → diagnosis → hypothesis → re-fly → compare →
   regression check.

---

## Refresher

```text
   plant process (JSBSim, 120 Hz)                     flight computer process (40 Hz)
 ┌───────────────────────────────┐   SENS k (UDP)   ┌─────────────────────────────┐
 │ gnc_trainer + actuators +     │ ───────────────► │ LongitudinalAP.step()       │
 │ sensors + XML lateral loops   │ ◄─────────────── │ (Module 11 control law)     │
 └───────────────────────────────┘   CMD_ k (UDP)   └─────────────────────────────┘
```

- **Lockstep** (CI mode): the plant waits for CMD k before stepping. It is
  deterministic and faster than real time. A configurable `delay_frames` emulates transport delay.
- **Real time:** the plant paces itself to the wall clock and never waits. It uses
  whatever command has arrived by the frame boundary. This design has **one frame of latency
  built in**, because the command computed from frame k's measurements is applied at frame k+1.
- **Latency is quantized by the frame.** A controller that finishes in 20 ms of a
  25 ms frame costs exactly the same as one that finishes in 1 ms. One that sometimes
  misses the deadline costs a whole extra frame each time it misses.

JSBSim also has built-in socket I/O (`<output type="SOCKET">`, `<input>`), used
for FlightGear and for some HIL rigs. Writing the loop in Python, as here,
gives you control of timing, which is the point of this module. A C++ team would
write the same thing in their flight-software framework.

## The code

- [`src/gnclab/sil.py`](../../src/gnclab/sil.py): packet formats, `controller_main`
  (the flight-computer process), `run_sil(SILConfig)` (the plant), and
  `in_process_reference` (the same law with no sockets, for validation).
- [`src/gnclab/logs.py`](../../src/gnclab/logs.py): `resample`, `estimate_offset`
  (cross-correlation clock alignment), `steps`, `intervals`, `activity_windows`.

> 🐛 **Gotchas found while building this module**
> - **A slow controller builds a backlog.** UDP queues packets. A controller that
>   takes 30 ms per 25 ms frame and processes packets in order works on
>   measurements that are older every frame: +7 frames per second, without bound.
>   **Drain the socket and use only the newest packet** (`drain=True`).
> - **"Boot" clocks.** Our flight computer's log first had a near-zero clock offset,
>   because the process starts milliseconds before the test. Real avionics are
>   powered on minutes earlier, so `fc_clock_offset_s` emulates that.
> - **Windows:** `time.sleep()` resolution was about 15.6 ms before Python 3.11; newer
>   versions use high-resolution timers. The plant sleeps until 0.5 ms before
>   each deadline and spins the rest. Binding to `127.0.0.1` normally avoids a
>   firewall prompt.

---

## Walkthrough

### 1. SIL: [`solutions/sil_run.py`](solutions/sil_run.py)

```text
1. lockstep SIL vs in-process controller (same law, same 40 Hz frame):
   alt_ft        max |difference| = 0          <- bit-for-bit: the rig adds nothing
2. transport delay (lockstep, 40 Hz frame = 25 ms):
    delay   added  q RMS in hold  elevator RMS  alt overshoot
        0     0 ms        2.07 °/s       1.44 °          7.0 ft
        2    50 ms        4.06 °/s       1.90 °          7.1 ft
        3    75 ms       18.92 °/s       6.17 °          7.2 ft     <- limit cycle
        5   125 ms       60.81 °/s      14.80 °          7.3 ft
3. real time: command age always 1 frame; identical to lockstep with delay_frames = 1
   controller takes 30 ms per 25 ms frame, drain=False: age 8, 15, 22, 29, ... frames
   controller takes 30 ms per 25 ms frame, drain=True : age 2, 2, 2, 3, ... frames
```

- **Validate the rig first.** A SIL that doesn't reproduce the in-process
  result bit-for-bit (in lockstep) has a bug: packing, ordering, units, or an
  off-by-one frame. Every later result depends on this one.
- **The delay cliff** is between 50 and 75 ms of added delay. Add the built-in
  frame of computation plus half a frame of zero-order hold, and the total is
  about 90 ms: Module 11's linear delay margin, now confirmed on a different
  rig. **Altitude overshoot doesn't notice**: 7.0 → 7.3 ft while the
  airplane oscillates at 60°/s. Pick metrics that can see the failure you're
  looking for (`sil_delay.png`).

### 2. Logs: [`solutions/log_analysis.py`](solutions/log_analysis.py)

```text
  flight-computer log: 2800 rows, interval 25.0 ms mean, 20.5..29.5 ms
  clock offset: plant time = flight-computer time -123.458 s (correlation peak 1.00; emulated 123.456 s)
   t [s] test point              overshoot   settle  final err  verdict
     2.0 alt +100                     8.6%     6.0s     +0.05  PASS
    30.0 vt +10                       7.6%     3.6s     +0.32  PASS
    50.0 alt -100                     4.7%     5.8s     -1.42  PASS
Part 2: 5 maneuvers joined into one 100 s log; finding them blind:
     1.6 -    4.2 s  de -> longitudinal | actually: long_3211
    21.6 -   23.2 s  de -> longitudinal | actually: long_doublet
    45.0 -   56.6 s  de -> longitudinal | actually: long_sweep
    61.6 -   69.3 s  dr -> lateral      | actually: lat_3211
    81.6 -   87.0 s  dr -> lateral      | actually: lat_doublet
```

- **Clock alignment** by cross-correlating pitch rate recovers the 123.456 s
  offset to within 2 ms (the extra 2 ms is the real boot-to-start time).
  Correlate a *busy* signal: rates and surface commands, not altitude.
- **Test points from the command channel** (`steps`). Never hand-type test-point times
  from the card; the card and the flight never agree exactly.
- **Part 2** uses the Module 14 logs. On a real program you get hours of data and a
  kneeboard card, and finding the maneuvers is the first job. Activity
  detection (rolling std above a robust noise floor) does it without labels.
  The 2 s of excitation in lat_3211's aileron-then-rudder input merge into one window, which is correct.

### 3. A tuning iteration: [`solutions/tuning_iteration.py`](solutions/tuning_iteration.py)

A new servo bus adds 50 ms. The card is flown in W20 = 10 ft/s:

```text
iteration                   kp    kd  overshoot  settle     q RMS  R-L1  R-Q
0 baseline                1.00  1.00       9.0%    6.0s   4.09°/s  PASS  FAIL
1 scale pitch loop        0.65  0.65      11.2%    9.5s   2.36°/s  FAIL  PASS
2 cut kd more than kp     0.80  0.50       9.2%    6.1s   2.62°/s  PASS  PASS
Re-check the candidate:
   delay   0 ms, W20  0.0: overshoot  7.1 %, q RMS  1.73   PASS PASS
   delay  75 ms, W20 10.0: overshoot  9.3 %, q RMS  2.90   PASS PASS   (baseline: 18.9 deg/s)
```

- **Diagnose from the log:** the q spectrum (`tuning_iteration.png`) shows a peak at
  2.5–3.3 Hz, near the pitch-loop crossover, and it grows with delay. That's
  lost phase margin, not turbulence.
- **Iteration 1** is the obvious fix, and it trades one failure for another. A slower
  pitch loop lets the altitude response overshoot.
- **Iteration 2:** a pure delay costs phase ωτ, which grows with frequency, and the q (derivative)
  path carries the high-frequency gain. Cutting k<sub>d</sub> more than k<sub>p</sub> recovers delay
  margin while keeping the low-frequency pitch stiffness that controls overshoot.
- **Regression check:** the new gains must still pass where the old gains were
  verified (no delay), and they survive 75 ms, where the baseline limit-cycled.
  Next step on a real program: re-run the Module 15 Monte Carlo with the new gains.

---

## Exercises ([`exercises/sil_ex.py`](exercises/sil_ex.py))

- **A. Average vs worst-case latency:** constant 20 ms vs jittery 0–40 ms vs constant 30 ms.
  Solution: [`solutions/sil_ex.py`](solutions/sil_ex.py).
  ```text
  constant 20 ms   1: 100.0%                       2.67°/s
  jitter 0-40 ms   1: 48.7%  2: 45.7%  3:  5.6%    4.09°/s    <- same mean, behaves like +1 frame
  constant 30 ms   1:  0.2%  2: 66.1%  3: 33.7%    5.41°/s
  ```
- **B.** Airspeed-error intervals from the plant log (`intervals`).
- **C.** (Open-ended) Move the lateral autopilot into the flight computer.

## Self-check

- How do you prove a SIL rig adds nothing? What would a one-frame mismatch tell you?
- Why does the real-time design have one frame of latency built in? How could you
  remove it, and what would it cost?
- Two flight computers both average 20 ms. Why can one be much worse for the loop?
- You have two recorders with unknown clock offset. Which signal do you correlate,
  and how do you know the alignment is right?
- Why did "scale both pitch gains" fail where "cut kd more" worked?

## Done when

- [ ] Your SIL reproduces the in-process run bit-for-bit, and you can explain why it must
- [ ] You can predict the delay cliff from Module 11's margin and confirm it on the rig
- [ ] You can turn a raw log into a test-point table without typing a single time by hand
- [ ] You can defend iteration 2 to a reviewer, including the regression check
