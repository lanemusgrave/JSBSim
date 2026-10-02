# Glossary

| Term | Meaning |
|---|---|
| **AHRS** | Attitude and heading reference system: gyros + accelerometers (+ magnetometer) → attitude |
| **Anti-windup** | Stopping an integrator from charging while the actuator or command is saturated (11) |
| **Bumpless engage** | Engaging an autopilot with its reference = the current state, so nothing jumps (09) |
| **Coherence (γ²)** | How linearly an output is explained by an input at each frequency; trust an FRF where γ² > 0.6 (14) |
| **Course (χ) vs heading (ψ)** | Direction of travel over the ground vs where the nose points; they differ by the crab angle (12) |
| **Delay margin** | Extra pure delay a loop tolerates before instability: PM/ω<sub>c</sub> (11, 16) |
| **Dispersion** | An uncertain parameter given a distribution for a Monte Carlo (15) |
| **Equation error / output error** | SysID by regressing measured coefficients / by matching simulated outputs (14) |
| **execrate** | `<channel execrate="N">`: run a channel every N frames (a slower software rate) |
| **FCS channel / component** | JSBSim's `<channel>` and its blocks (gain, filter, pid, actuator, sensor, …) (08) |
| **FDM** | Flight dynamics model: the equations of motion plus the vehicle models; `FGFDMExec` in JSBSim |
| **Fillet** | An arc joining two straight legs, so the path is flyable at finite bank (12) |
| **Flight readiness** | The evidence-based decision that a configuration may fly a mission (18) |
| **HIL / SIL** | Hardware- / software-in-the-loop: the real flight computer / software against a simulated plant (16) |
| **Lockstep** | Plant and controller advance frame by frame, waiting for each other: deterministic SIL (16) |
| **Margin to failure** | How far (often in σ) a parameter can move before a requirement fails (15) |
| **Neutral point** | The CG position where static pitch stability (C<sub>mα</sub>) is zero (05) |
| **Property tree** | JSBSim's global name → value store; how everything is read, written, logged and connected (01) |
| **q̄ (qbar)** | Dynamic pressure, ½ρV²; aero forces scale with it, so gains are scheduled on it (11) |
| **Rule of three** | Zero failures in n trials → P(fail) < 3/n at 95% confidence (15) |
| **Specific force** | Non-gravitational force per unit mass: what an accelerometer measures (13) |
| **Successive loop closure** | Designing inner loops first, then slower outer loops around them (09) |
| **Test card** | A flight-test plan: conditions, inputs, timings, abort criteria and what is measured (03, 16) |
| **3-2-1-1** | A multi-step input with broad frequency content for system ID (14) |
| **Transport delay** | Pure time delay between a measurement and the actuator response (11, 16) |
| **Trim** | The steady state where forces and moments balance (05) |
| **Turbulence W20** | MIL-F-8785C wind speed at 20 ft that sets low-altitude turbulence intensity (15) |
| **Vector field guidance** | Commanding course from the path error so every point flows onto the path (12) |
| **Washout** | A high-pass filter: it passes changes and blocks steady values (yaw damper, 10) |
