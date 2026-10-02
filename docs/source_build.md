# Building JSBSim from source (and working in a fork)

You need a source build when you change JSBSim itself: a new FCS component, an
engine model, a bug fix. On the job you'll build the **team's fork**; here we
practice on upstream v1.3.1 in `external/jsbsim` (gitignored, so it's never
committed to this repo).

| What | Script | Output | Time (4 cores) |
|---|---|---|---|
| C++ executable | `scripts/build_jsbsim_source.sh` (`.ps1` on Windows) | `external/jsbsim/build/src/JSBSim` | ~1 min |
| Python module + upstream tests | `scripts/build_jsbsim_python.sh` | `external/jsbsim/build-py/tests/jsbsim` | ~1 min build, ~2.5 min tests |

## Windows 10: use WSL2 (recommended)

The fork's tooling, CI and most JSBSim developers are on Linux, and the
scripts here are tested on Ubuntu. On Windows 10 (version 2004 or later):

```powershell
wsl --install -d Ubuntu          # admin PowerShell, then reboot and create a user
```

Inside Ubuntu:

```bash
sudo apt update && sudo apt install -y build-essential cmake git python3-venv python3-dev
git clone <this repo url> ~/JSBSim && cd ~/JSBSim     # keep it in the Linux filesystem:
bash scripts/build_jsbsim_source.sh                    # /mnt/c is ~10x slower to build
bash scripts/build_jsbsim_python.sh
```

VS Code with the **WSL** extension (`code .` from the Ubuntu shell) edits and
debugs the Linux checkout as if it were local.

## Windows 10: native (Visual Studio 2022)

Install Git for Windows, CMake ≥ 3.15 and Visual Studio 2022 (Community or
Build Tools) with the **Desktop development with C++** workload. Then:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build_jsbsim_source.ps1
# -> external\jsbsim\build\src\Release\JSBSim.exe
```

The generator is multi-config, so pass `--config Release` to every
`cmake --build`. The executable lands in `build\src\Release\`. The native
Python-module build also needs a matching CPython with development headers and
Cython. It works, but WSL2 is far less fiddly. `check_moving_average.py` (Module 17) looks for
`build\src\Release\JSBSim.exe` automatically.

## A tour of the source (v1.3.1)

```text
src/
  FGFDMExec.{h,cpp}        the executive: loads models, runs them in order every frame
  JSBSim.cpp               the command-line program (scripts, --root, --script, ...)
  models/
    FGPropagate.cpp        integrates the equations of motion (runs FIRST each frame)
    FGAccelerations.cpp    forces/moments -> accelerations
    FGAerodynamics.cpp     <aerodynamics> axes and functions
    FGFCS.cpp              ALL <system>/<autopilot>/<flight_control> channels; parses components
    flight_control/        one class per FCS component (FGActuator, FGPID, FGSensor, ...)
    atmosphere/FGWinds.cpp wind, gusts, MIL-F-8785C turbulence
  math/                    FGFunction, FGTable, FGColumnVector3, FGQuaternion, ...
  input_output/            XML parsing, property manager, output (CSV, socket), FGLog
  initialization/          FGInitialCondition, FGTrim, FGLinearization
python/jsbsim.pyx.in       the Cython bindings (what `import jsbsim` wraps)
tests/                     upstream regression tests (Python unittest), run by ctest
```

`FGFDMExec::Run()` executes the models in this order every frame:
**Propagate → Input → Inertial → Atmosphere → Winds → Systems (FCS) → MassBalance →
Auxiliary → Propulsion → Aerodynamics → GroundReactions → ExternalReactions →
BuoyantForces → Aircraft → Accelerations → Output**. Propagate integrates
first, using the accelerations from the *previous* frame. That's why a command
written before `run()` affects the motion one frame later (Module 06), and why
the FCS sees the state at the start of the frame.

## Working in a fork: the git workflow

```bash
git clone <team fork url> jsbsim && cd jsbsim
git remote add upstream https://github.com/JSBSim-Team/jsbsim.git
git checkout -b feature/moving-average      # one feature per branch, with its test
# ... edit, build, ctest ...
git commit -m "Add <moving_average> FCS component"
git push -u origin feature/moving-average   # -> merge request into the fork's main branch
```

**Syncing with upstream.** Upstream moves on (bug fixes you want). Periodically:

```bash
git fetch upstream
git checkout main && git merge upstream/master         # or the release tag you track
# feature branches: replay your commits on the new base
git rebase --onto upstream/master <old-base> feature/moving-average
```

- `--onto` replays only *your* commits (`<old-base>..feature`). We hit this in
  Module 17: the checkout was cut at tag v1.3.1, which is **not on upstream master**
  (releases are tagged on a release branch). A plain `git rebase upstream/master`
  tried to replay upstream's own history and conflicted on dozens of files.
  `git rebase --onto upstream/master v1.3.1` replayed the one commit cleanly.
- Keep fork changes **small, self-contained and tested**. Each one is a future merge
  conflict. Better still, propose it upstream (a PR to JSBSim-Team), so it
  stops being yours to carry.
- Never edit generated or vendored code (`src/simgear`, `src/GeographicLib`) in place
  without a note. Those conflicts are the worst ones.
- After every sync, run the **full** test suite: upstream's (`ctest`) and the
  team's own (Monte Carlo regression, SIL). Rebuild the Python wheel that the
  team's tools import. A Python tool on the stock wheel *silently drops* your
  new components (Module 17, part B).

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Could NOT find Cython` | Use `scripts/build_jsbsim_python.sh` (it makes its own venv), or pass `-DPython3_EXECUTABLE=` a Python that has Cython |
| `ModuleNotFoundError: fpectl` in ctest | Build all targets (`cmake --build build-py`), not just `_jsbsim` |
| `ModuleNotFoundError: pandas` in ctest | The tests need pandas and scipy in the venv cmake found |
| Random ctest failures with `-j` | Several tests write the same output files; run `ctest` serially |
| `Unknown FCS component: ...` | You're running a JSBSim without your patch: check `jsbsim.__file__` / which executable |
| Output CSV not where you expect | Output paths are relative to `--root` (v1.3.1's `--outputlogfile` didn't redirect an aircraft-defined `<output>`) |
| `invalid use of incomplete type FGFCS` | `#include "models/FGFCS.h"` in your component's .cpp |
