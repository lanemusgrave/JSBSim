"""Create, initialize and run JSBSim models from Python.

The JSBSim objects you will use most:

* ``jsbsim.FGFDMExec`` - the *executive*.  Owns every model (atmosphere,
  aerodynamics, propulsion, FCS, equations of motion...) and the property tree.
* ``fdm["some/property"]`` - read a property; ``fdm["some/property"] = x`` writes it.
* ``fdm.run_ic()`` - apply the ``ic/...`` initial-condition properties.
* ``fdm.run()`` - advance the simulation by one time step ``dt``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Iterable, Mapping

import jsbsim
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
AIRCRAFT_DIR = REPO_ROOT / "aircraft"
OUTPUT_DIR = REPO_ROOT / "outputs"

# A handy default set of properties to record (alias -> JSBSim property).
LONGITUDINAL = {
    "V_kts": "velocities/vtrue-kts",
    "alt_ft": "position/h-sl-ft",
    "alpha_deg": "aero/alpha-deg",
    "theta_deg": "attitude/theta-deg",
    "q_rps": "velocities/q-rad_sec",
    "gamma_deg": "flight-path/gamma-deg",
    "elev_rad": "fcs/elevator-pos-rad",
    "throttle": "fcs/throttle-cmd-norm[0]",
}
LATERAL = {
    "beta_deg": "aero/beta-deg",
    "phi_deg": "attitude/phi-deg",
    "psi_deg": "attitude/psi-deg",
    "p_rps": "velocities/p-rad_sec",
    "r_rps": "velocities/r-rad_sec",
    "ail_rad": "fcs/left-aileron-pos-rad",
    "rud_rad": "fcs/rudder-pos-rad",
}


# Harmless JSBSim messages we hide (each is explained in the module READMEs).
_NOISE = (
    "unable to open the file",       # aircraft <output> re-opened by a 2nd run_ic()
    "Output to this file is disabled",
)


class _FilteringLogger(jsbsim.DefaultLogger):
    """Buffers each JSBSim log message and drops the known-harmless ones."""

    def __init__(self):
        super().__init__(jsbsim.LogLevel.INFO)
        self._buf: list[str] = []

    def set_level(self, level):
        super().set_level(level)
        self.log_level = level

    def message(self, message: str) -> None:
        self._buf.append(message)

    def flush(self) -> None:
        text = "".join(self._buf)
        self._buf.clear()
        if not text or any(n in text for n in _NOISE):
            return
        if int(getattr(self, "log_level", jsbsim.LogLevel.INFO)) >= int(self.min_level):
            print(text, end="")


_logger_installed = False


def quiet_jsbsim() -> None:
    """Silence JSBSim's console chatter (banner, model reports) and hide the
    known-harmless messages listed in ``_NOISE``.  Warnings and errors still show."""
    global _logger_installed
    jsbsim.FGJSBBase().debug_lvl = 0
    if not _logger_installed:
        jsbsim.set_logger(_FilteringLogger())
        _logger_installed = True


def make_fdm(
    aircraft: str = "c172x",
    dt: float = 1.0 / 120.0,
    quiet: bool = True,
    output: bool = False,
    systems_dir: str | Path | None = None,
) -> jsbsim.FGFDMExec:
    """Create an FGFDMExec and load ``aircraft``.

    Aircraft are looked up first in this repo's ``aircraft/`` folder (models you
    build, e.g. ``gnc_trainer``) and otherwise among the ~60 models bundled with
    the jsbsim pip package (``c172x``, ``f16``, ``T38``, ``ball``...).

    ``output=False`` disables any ``<output>`` file the aircraft file requests,
    so lessons do not litter CSV files in your working directory.

    ``systems_dir``: folder searched for ``<system file="...">`` includes.
    Default for repo aircraft: ``aircraft/<name>/fcs/``.  Point it at your own
    folder to fly your own autopilot XML with the same airframe.
    """
    if quiet:
        quiet_jsbsim()
    fdm = jsbsim.FGFDMExec(jsbsim.get_default_root_dir())
    # Output files requested by an aircraft/script land in outputs/, never in
    # the (possibly read-only) jsbsim package folder.
    OUTPUT_DIR.mkdir(exist_ok=True)
    fdm.set_output_path(str(OUTPUT_DIR))
    if (AIRCRAFT_DIR / aircraft).is_dir():
        fdm.set_aircraft_path(str(AIRCRAFT_DIR))
        fdm.set_systems_path(str(Path(systems_dir).resolve() if systems_dir else AIRCRAFT_DIR / aircraft / "fcs"))
    elif systems_dir:
        fdm.set_systems_path(str(Path(systems_dir).resolve()))
    if not fdm.load_model(aircraft):
        raise RuntimeError(f"JSBSim could not load aircraft '{aircraft}'")
    if not output:
        fdm.disable_output()
    fdm.set_dt(dt)
    return fdm


def load_script(
    script: str | Path,
    quiet: bool = True,
    output: bool = False,
    dt: float = 0.0,
) -> jsbsim.FGFDMExec:
    """Create an FGFDMExec and load a JSBSim *script* (``<runscript>`` XML).

    ``script`` can be a path relative to the JSBSim data folder (e.g.
    ``"scripts/c1723.xml"`` for the bundled scripts) or a path to one of your
    own script files.  The script names the aircraft and initial conditions
    and contains timed/conditional *events*.  After loading, call ``fdm.run()``
    until it returns False (the script's end time).

    With ``output=True`` any ``<output>`` files land in ``outputs/``.
    """
    if quiet:
        quiet_jsbsim()
    fdm = jsbsim.FGFDMExec(jsbsim.get_default_root_dir())
    OUTPUT_DIR.mkdir(exist_ok=True)
    fdm.set_output_path(str(OUTPUT_DIR))
    name = _uses_repo_aircraft(script)
    if name:
        fdm.set_aircraft_path(str(AIRCRAFT_DIR))
        fdm.set_systems_path(str(AIRCRAFT_DIR / name / "fcs"))
    path = Path(script)
    if path.exists():
        path = path.resolve()
    if not fdm.load_script(str(path), dt):
        raise RuntimeError(f"JSBSim could not load script '{script}'")
    if not output:
        fdm.disable_output()
    if not fdm.run_ic():
        raise RuntimeError("run_ic() failed")
    return fdm


def _uses_repo_aircraft(script: str | Path) -> str:
    """Aircraft name if the script's ``<use aircraft="...">`` is in aircraft/, else ""."""
    import xml.etree.ElementTree as ET

    path = Path(script)
    if not path.exists():
        return ""
    use = ET.parse(path).getroot().find("use")
    name = use.get("aircraft", "") if use is not None else ""
    return name if name and (AIRCRAFT_DIR / name).is_dir() else ""


def initialize(
    fdm: jsbsim.FGFDMExec,
    ic: Mapping[str, float] | None = None,
    engines_running: bool = True,
    **props: float,
) -> jsbsim.FGFDMExec:
    """Set initial conditions and call ``run_ic()``.

    ``ic`` keys may be written with or without the ``ic/`` prefix, e.g.
    ``{"h-sl-ft": 5000, "vc-kts": 100}``.  Extra keyword ``props`` are written
    verbatim *after* the IC (use ``{"fcs/throttle-cmd-norm": 0.7}`` style dicts
    via ``**{...}`` for names with slashes).

    Keys are written in dict order, and order matters: put position
    (lat/long/altitude) before airspeed.  Writing ``ic/lat-geod-deg`` after
    ``ic/vc-kts`` keeps the true airspeed and changes the calibrated one.
    """
    for key, value in (ic or {}).items():
        name = key if key.startswith("ic/") else f"ic/{key}"
        fdm[name] = value
    if not fdm.run_ic():
        raise RuntimeError("run_ic() failed")
    if engines_running and fdm.get_propulsion().get_num_engines() > 0:
        # -1 = all engines.  Sets them running at a steady state (no second
        # run_ic() needed; calling run_ic() twice re-opens output files).
        fdm["propulsion/set-running"] = -1
    for name, value in props.items():
        fdm[name] = value
    return fdm


def _normalize(props: Mapping[str, str] | Iterable[str]) -> dict[str, str]:
    if isinstance(props, Mapping):
        return dict(props)
    return {p: p for p in props}


def run(
    fdm: jsbsim.FGFDMExec,
    duration: float,
    props: Mapping[str, str] | Iterable[str] = LONGITUDINAL,
    callback: Callable[[jsbsim.FGFDMExec, float], None] | None = None,
    record_every: int = 1,
) -> pd.DataFrame:
    """Run for ``duration`` seconds and return a time-indexed DataFrame.

    ``props`` is a list of property names or a dict ``{column: property}``.
    ``callback(fdm, t)`` is called *before* every step: this is where you put
    pilot inputs, test signals, or a Python-side controller.
    """
    cols = _normalize(props)
    t_end = fdm.get_sim_time() + duration
    dt = fdm.get_delta_t()
    rows, times = [], []
    k = 0
    while fdm.get_sim_time() < t_end - 0.5 * dt:
        t = fdm.get_sim_time()
        if callback is not None:
            callback(fdm, t)
        if k % record_every == 0:
            times.append(t)
            rows.append([fdm[p] for p in cols.values()])
        if not fdm.run():
            break
        k += 1
    times.append(fdm.get_sim_time())
    rows.append([fdm[p] for p in cols.values()])
    df = pd.DataFrame(rows, columns=list(cols.keys()), index=pd.Index(times, name="t"))
    return df


def step_until(fdm: jsbsim.FGFDMExec, condition: Callable[[jsbsim.FGFDMExec], bool],
               t_max: float = 600.0) -> float:
    """Step until ``condition(fdm)`` is true (or ``t_max``); return sim time."""
    t_end = fdm.get_sim_time() + t_max
    while not condition(fdm) and fdm.get_sim_time() < t_end:
        fdm.run()
    return fdm.get_sim_time()
