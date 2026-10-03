# AGM-158B JASSM-ER — JSBSim flight model

Air-released cruise missile. The default initial condition is a bomber
release at 25,000 ft and Mach 0.75, wings folding out over the first two
seconds, then a descent onto a 500 ft / Mach 0.80 cruise.

This is an engineering approximation from open sources, not a contractor
aero deck.

## What was matched

| Item | Model | Source |
| --- | --- | --- |
| Length | 14.08 ft | Wikipedia, AGM-158A/B |
| Width / height | 25 in / 18 in | Wikipedia, JASSM-ER |
| Wingspan, deployed | 8.86 ft | Wikipedia |
| Launch weight | 2,600 lb | Wikipedia estimate for the ER |
| Warhead | 1,000 lb class, inside empty weight | WDU-42/B |
| Engine | Williams F107-WR-105, 1,400 lbf SL static | Wikipedia / Air & Space Forces |
| Cruise | Mach 0.80, low level | subsonic, published top end near Mach 0.85-0.90 |
| Range class | about 600 nm on 560 lb fuel | ER is greater than 500 nm |

The baseline AGM-158A uses a Teledyne J402 turbojet and carries less fuel
for about 230 mi. The external body is the same. This file is the ER.

Wing area (12 ft²), the 560 lb fuel load, inertias and derivatives are
estimates. Fuel was sized so a low-level Mach 0.80 leg lands in the
published ER range class. Checked in JSBSim 1.3.1: a release from 25,000 ft
captures 500 ft and Mach 0.80, wings level, in about six minutes.

## Layout

```
AGM158/
  AGM158.xml           flight model
  reset_release.xml    25,000 ft, Mach 0.75, gamma -2 deg
  reset_cruise.xml     already on the 500 ft cruise
  Engines/F107-WR-105.xml
  Engines/direct.xml
  run_release.py
```

Structural frame is the JSBSim aircraft frame: origin at the nose, X aft,
Y right, Z up.

## Run it

```
python3 run_release.py            # air release, then descent to cruise
python3 run_release.py cruise     # start already at 500 ft
```

Set these before the first frame if you are driving it yourself:

- `guidance/altitude-setpoint-ft` (default 500)
- `guidance/mach-setpoint` (default 0.80)
- `guidance/heading-setpoint-deg` (default 90)
- `fcs/release-mode` = 1 to keep the wings stowed for the first 2 seconds

The autopilot is altitude, heading and Mach hold. It is not the GPS/INS
plus imaging-IR guidance. On a release it commands a 6 deg descent, but the
first half-minute overshoots that and gets steeper while the speed builds.
It then captures. For a clean cruise leg, start from `reset_cruise.xml`.
