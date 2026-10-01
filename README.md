# JSBSim GNC Lab

Flight dynamics and controls simulation practice with
[JSBSim](https://github.com/JSBSim-Team/jsbsim): a hands-on curriculum, built in
steps, for getting back up to speed on fixed-wing guidance, navigation and
control. It starts at "install JSBSim" and ends at designing, identifying and
verifying a full GNC stack for a small UAS. Along the way it covers the skills
a fixed-wing UAS GNC role needs:

- control laws that respect actuator limits, delays and sensor noise
- aero modeling and system identification
- Monte Carlo and SIL verification
- flight-test log analysis
- working in a fork of JSBSim's C++

**Start here:** [docs/setup.md](docs/setup.md) → [CURRICULUM.md](CURRICULUM.md) → [modules/00_setup](modules/00_setup/README.md)

## Quick start (Windows 10, PowerShell)

```powershell
git clone https://github.com/lanemusgrave/JSBSim.git
cd JSBSim
powershell -ExecutionPolicy Bypass -File scripts\setup.ps1   # venv + JSBSim + checks
.\.venv\Scripts\Activate.ps1
python modules\00_setup\solutions\first_flight.py --show
```

Linux / macOS / WSL2: `bash scripts/setup.sh && source .venv/bin/activate`.

## What's in the repo

```text
CURRICULUM.md        roadmap: 19 modules in 6 phases, readings, progress checklist
docs/                setup guide (Windows-first) and source-build guide
modules/NN_topic/    README (lesson) + exercises/ (starter code) + solutions/ (reference)
aircraft/            models you build: gnc_trainer (small UAS), glider, ball variants
src/gnclab/          small helper package used by the lessons (sim, trim, linear, metrics...)
reference/           cheat sheets: properties, equations, glossary, reading list
scripts/             setup, install check, build JSBSim from source
tests/               pytest: helper unit tests + checks that every solution still runs
outputs/             plots and logs written by the lessons (gitignored)
external/            JSBSim C++ source checkout for Module 17 (gitignored)
```

## Versions

Everything is pinned to and verified against **JSBSim 1.3.1** (the PyPI
`jsbsim` wheel, which includes the C++ engine, the Python bindings, the `jsbsim`
command-line program and about 60 aircraft) with Python 3.10–3.13.

## Running the checks

```bash
pytest -m "not slow"   # fast: helpers + numerical sanity checks (~10 s)
pytest                 # everything, including every module solution (a few minutes)
```

## License and credits

The lesson material and code here are original. JSBSim is LGPL-2.1 (© the JSBSim
Development Team). The `gnc_trainer` aerodynamic data are the published
Aerosonde-class parameters from Beard & McLain, *Small Unmanned Aircraft:
Theory and Practice* (Princeton University Press, 2012), used for study.
