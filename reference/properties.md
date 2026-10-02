# JSBSim property cheat sheet

The names used most in this curriculum, each **checked to exist** on `c172x` and `gnc_trainer`
with JSBSim 1.3.1. Units are in the name (`-ft`, `-rad`, `-fps`, `-psf`, `_sec`). Find the rest with
`fdm.query_property_catalog("alpha")` or `fdm.print_property_catalog()`, or RM appendix
*Native properties*.

> ⚠ **Check signs and frames of every property you didn't create** (Module 12):
> `position/distance-from-start-lat-mt` / `-lon-mt` look like north/east but are
> **unsigned** distances. Use `gnclab.guidance.ne_from_latlon`.

## Simulation and initial conditions

| property | meaning |
|---|---|
| `simulation/sim-time-sec`, `simulation/dt` | time, step |
| `simulation/randomseed` | seed for sensor noise (set per Monte Carlo case) |
| `simulation/do_simple_trim` | write a trim mode to run JSBSim's trim (scripts) |
| `ic/h-sl-ft`, `ic/lat-geod-deg`, `ic/long-gc-deg` | IC position. **Set before airspeed** |
| `ic/vt-fps`, `ic/vc-kts`, `ic/psi-true-deg`, `ic/gamma-deg`, `ic/alpha-deg`, `ic/theta-deg`, `ic/phi-deg` | IC velocity and attitude |
| `propulsion/set-running` | `-1` starts all engines at steady state |

## State

| property | meaning |
|---|---|
| `position/h-sl-ft`, `position/h-agl-ft` | altitude above MSL / ground |
| `position/lat-geod-rad`, `position/long-gc-rad` (and `-deg`) | geodetic latitude, longitude |
| `attitude/phi-rad`, `theta-rad`, `psi-rad` (and `-deg`) | Euler angles (psi = true heading) |
| `velocities/u-fps`, `v-fps`, `w-fps` | body-axis velocity relative to the ground (inertial) |
| `velocities/u-aero-fps` | body-axis x velocity relative to the **air** |
| `velocities/p-rad_sec`, `q-rad_sec`, `r-rad_sec` | body rates (what a gyro measures, without the earth rate) |
| `velocities/p-aero-rad_sec`, … | body rates relative to the air (turbulence rates removed); used in aero damping terms |
| `velocities/vt-fps` | true airspeed |
| `velocities/vc-kts`, `velocities/ve-kts` | calibrated / equivalent airspeed |
| `velocities/vg-fps`, `flight-path/psi-gt-rad` | ground speed, ground track (**course** χ, in [0, 2π)) |
| `velocities/v-north-fps`, `v-east-fps`, `v-down-fps`, `velocities/h-dot-fps` | NED velocity, climb rate |
| `flight-path/gamma-rad` | flight-path angle |
| `velocities/mach` | Mach |

## Aerodynamics and forces

| property | meaning |
|---|---|
| `aero/alpha-rad`, `aero/beta-rad` (and `-deg`), `aero/alphadot-rad_sec` | angle of attack, sideslip |
| `aero/qbar-psf` | dynamic pressure |
| `aero/ci2vel` = c̄/(2V), `aero/bi2vel` = b/(2V) | rate non-dimensionalizers for C<sub>mq</sub>, C<sub>lp</sub>, … |
| `forces/fwx-aero-lbs` (wind), `forces/fsx-aero-lbs` (stability), `forces/fbx-aero-lbs` (body) | aero force components in three frames |
| `forces/fbx-total-lbs`, `fby-…`, `fbz-…` | total body force **excluding gravity** → specific force (accelerometer) × mass |
| `moments/m-aero-lbsft`, `moments/m-total-lbsft` | pitching moment |
| `forces/load-factor`, `accelerations/n-pilot-z-norm` | load factor; at the pilot station |
| `accelerations/udot-ft_sec2`, `qdot-rad_sec2`, `pdot-rad_sec2` | body accelerations |
| `aero/force/<name>`, `aero/moment/<name>` | each function in your aero build-up (Module 04) |

## Mass, atmosphere, wind

| property | meaning |
|---|---|
| `inertia/mass-slugs`, `inertia/weight-lbs`, `inertia/cg-x-in`, `inertia/ixx-slugs_ft2`, … | mass properties |
| `inertia/pointmass-weight-lbs[i]`, `inertia/pointmass-location-X-inches[i]` | movable point masses (payload, ballast) |
| `atmosphere/rho-slugs_ft3`, `T-R`, `P-psf`, `a-fps` | density, temperature, pressure, speed of sound |
| `atmosphere/wind-north-fps`, `wind-east-fps`, `wind-down-fps` | steady wind: the direction it blows **toward** |
| `atmosphere/total-wind-north-fps`, … | wind + gusts + turbulence (read only) |
| `atmosphere/turb-type` | 0 off, 3 MIL-F-8785C spectra, 4 the same spectra Tustin-discretized (1, 2: older models) |
| `atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps` | W20: intensity below 1000 ft (σ<sub>w</sub> = 0.1·W20) |
| `atmosphere/turbulence/milspec/severity` | probability-of-exceedance index. **0 disables turbulence** even at low altitude |
| `atmosphere/randomseed` | turbulence seed |

## Controls and propulsion

| property | meaning |
|---|---|
| `fcs/elevator-cmd-norm`, `aileron-cmd-norm`, `rudder-cmd-norm` | pilot commands, −1…1 |
| `fcs/throttle-cmd-norm` (= `[0]`), `fcs/throttle-pos-norm` | throttle command / position, 0…1 |
| `fcs/pitch-trim-cmd-norm`, `roll-…`, `yaw-…` | trim inputs (what JSBSim's trim adjusts) |
| `fcs/elevator-pos-rad`, `fcs/elevator-pos-norm` | surface position |
| `propulsion/engine/thrust-lbs` | thrust of engine 0 |

## gnc_trainer-specific (this repo)

| property | meaning |
|---|---|
| `fcs/actuators-on`, `sensors/enabled` | realistic actuators / sensor models on (`gnclab.autopilot.realism`) |
| `fb/<signal>` | what the autopilot sees: truth, or sensor output when `sensors/enabled = 1` |
| `ap/*-hold-on`, `ap/alt-cmd-ft`, `ap/vt-cmd-fps`, `ap/chi-cmd-rad`, `ap/phi-cmd-ext-rad` | autopilot engage and commands |
| `ap/elevator-cmd-norm`, … | autopilot output, summed with the pilot command |
| `uncertainty/<coef>-scale` | Monte Carlo aero/thrust multipliers (1 = nominal) |
| `fcs/actuator-bw-rad_sec`, `fcs/actuator-rate-rad_sec`, `sensors/gyro-bias-q-rad_sec`, … | actuator and sensor dispersions |
| `propulsion/thrust-N`, `propulsion/omega-rad_sec` | the algebraic motor/propeller model |
