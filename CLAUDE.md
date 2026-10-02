# CLAUDE.md

A personal, hands-on JSBSim GNC curriculum (19 modules, `modules/00..18`) for a
fixed-wing UAS GNC engineer. The learner works on **Windows 10** (PowerShell; WSL2
for source builds), is an experienced pilot, and is rusty on the engineering.

## Ground rules

- **Never commit personal or employer details** (offer letters, names, compensation,
  company specifics). Describe the goal generically. `*.pdf` is gitignored on purpose.
- Every number quoted in a README must come from actually running the script.
  If a result surprises you, investigate before writing it up. Many lessons here
  ("gotchas found while building this module") came from exactly that.
- Pinned to **JSBSim 1.3.1** (PyPI wheel). `external/` (the C++ checkout, fork venv) is gitignored.

## Layout and conventions

- `modules/NN_topic/`: `README.md` (objectives, refresher, walkthrough with real output,
  gotchas, exercises, self-check, done-when), `exercises/` (runnable starters with TODOs),
  `solutions/` (reference + `answers.md`). Every solution script supports `--fast`
  (used by the tests) and `--show`, and parses args with `gnclab.cli.parse_args`.
- `src/gnclab/`: small, readable helpers (sim, trim, linear, metrics, signals, autopilot,
  controllers, guidance, estimation, sysid, montecarlo, sil, logs, mission). Add a helper
  only when a module needs it; keep the JSBSim calls visible.
- `aircraft/gnc_trainer/gnc_trainer.xml` is **generated** by
  `modules/07_build_uas_model/solutions/build_gnc_trainer.py`. Edit the generator, re-run it.
  `Systems/gnc_sensors.xml` and `fcs/gnc_autopilot.xml` are hand-written.
- Scripts that use `multiprocessing` (Monte Carlo, SIL) must keep all work inside `main()`
  behind `if __name__ == "__main__":`. Workers are *spawned* (Windows behaviour on every OS).
- Outputs go to `outputs/<module>/` (gitignored) via `gnclab.plotting.outdir/save`.
- XML comments must not contain `--`.

## JSBSim facts that bit us (see the module READMEs for detail)

- All `<system>`s run before `<flight_control>`; within the FCS, channels run in file order.
- Calling `run_ic()` twice re-opens output files; set IC position before airspeed.
- `<delay>` works only on actuator, sensor and switch components.
- `position/distance-from-start-*-mt` are unsigned; use `gnclab.guidance.ne_from_latlon`.
- `forces/fb*-total-lbs` excludes gravity (= specific force × mass).
- Turbulence needs `atmosphere/turbulence/milspec/severity` ≠ 0, even at low altitude.
- Stock JSBSim silently drops unknown FCS components (it logs one error line).
- Random seeds give different noise/turbulence on Windows vs Linux (implementation-defined
  `std::default_random_engine`/`normal_distribution`). Never test on a single lucky draw.

## Checks

```bash
.venv/bin/pytest -q                  # everything (~3-4 min): unit tests + every solution --fast
.venv/bin/pytest -q -m "not slow"    # helpers and numerical checks only
```

CI (`.github/workflows/ci.yml`) runs `scripts/setup.ps1` on windows-latest and `setup.sh`
on ubuntu-latest, then the full pytest.

## Git

Work on the designated feature branch; commit per module with a descriptive message; no PRs
unless asked.
