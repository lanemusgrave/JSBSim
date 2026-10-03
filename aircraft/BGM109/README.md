# BGM-109 Tomahawk — JSBSim flight model

Cruise configuration of a BGM-109C/Block III class Tomahawk, with an optional
12-second solid booster for the launch transient. Built from open sources and
sized so the missile trims near the published cruise point. It is not a
contractor aero deck or a flight-test model.

## What was matched to published data

| Item | Model | Source |
| --- | --- | --- |
| Length, no booster | 18.25 ft | GlobalSecurity / FAS |
| Length with booster | 20.5 ft | GlobalSecurity |
| Diameter | 20.4 in | GlobalSecurity |
| Wingspan, deployed | 8.75 ft | GlobalSecurity |
| Missile weight, fueled, no booster | 2,850 lb | 2,900 lb published, less a bit of unusable |
| Booster | 550 lb, jettisoned at 12 s | ~550 lb published case |
| Fuel | 900 lb | Block III class, ~800–1,026 lb |
| Sustainer | F107-WR-402, 700 lbf SL static, BPR 1, TSFC 0.683 lb/lbf/hr | Williams / Wikipedia |
| Booster thrust | 6,000 lbf for 12 s | Atlantic Research figure quoted in open sources |
| Cruise | Mach 0.72 class, low altitude | ~550 mph published |

Wing area (13 ft²), inertias, and the stability derivatives are estimates.
They were checked in JSBSim 1.3.1: from `reset_cruise.xml` the missile holds
400 ft, wings level, about Mach 0.71 and 620 lbf, alpha about 2.9 deg.

Fuel flow at that point is about 430 lb/hr, so 900 lb is roughly 2.1 hours
and about 1,000 nm. That is the Block III conventional ballpark, not a
specific mission profile.

## Layout

```
BGM109/
  BGM109.xml          flight model
  reset_cruise.xml    wings out, Mach 0.72, 400 ft, alpha 3.2 deg
  reset_launch.xml    40 deg climb, 90 ft/s, booster armed
  Engines/F107-WR-402.xml
  Engines/direct.xml
  run_cruise.py
```

Structural frame is the JSBSim aircraft frame: origin at the nose, X aft,
Y right, Z up. Body-axis forces stay X forward, Z down.

## Run it

Python, from this directory, with the `jsbsim` package installed:

```
python3 run_cruise.py
python3 run_cruise.py launch
```

Standalone JSBSim: put this folder in `aircraft/` and the two engine files
where the engine path can see them (or copy `direct.xml` from the JSBSim
data package). Then:

```
JSBSim --aircraft=BGM109 --initfile=reset_cruise.xml
```

Set these before the first frame if you are not using `run_cruise.py`:

- `guidance/altitude-setpoint-ft` (default 400)
- `guidance/mach-setpoint` (default 0.72)
- `guidance/heading-setpoint-deg` (default 90)
- `fcs/launch-mode` = 1 for a booster shot, 0 for cruise
- `inertia/pointmass-weight-lbs` = 550 before a launch IC, so the booster mass is on the rail

The cruise autopilot is altitude hold, heading hold, and a Mach hold with a
0.86 throttle bias. It is a stability-augmentation loop, not TERCOM/DSMAC.

## Launch

`fcs/launch-mode = 1` lights a 6,000 lbf body-axis force for 12 seconds, holds
the wings and inlet stowed, and drops the 550 lb booster mass at separation.
A theta-hold keeps the nose near 38 deg during the burn. The airframe is
trimmed for the cruise configuration, so the boost-to-cruise handover is the
rough part of this model. For trajectory work, start from `reset_cruise.xml`.

## Knobs worth knowing

- Cruise altitude and speed: the three `guidance/` properties above.
- Wing deployment: `fcs/wings-deployed` scales wing lift. The launch sequence
  writes it; set it to 1 for cruise.
- Reference area is 13 ft². Derivatives are per rad, about the aero reference
  point 108 in aft of the nose.
