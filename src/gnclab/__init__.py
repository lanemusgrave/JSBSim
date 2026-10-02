"""gnclab: small helpers shared by the JSBSim GNC curriculum.

The helpers are intentionally thin wrappers around the JSBSim Python API so you
can always see (and should read!) what JSBSim calls are being made.  Each module
README tells you which helper it introduces.
"""

from gnclab.sim import (
    AIRCRAFT_DIR,
    OUTPUT_DIR,
    REPO_ROOT,
    make_fdm,
    load_script,
    initialize,
    run,
    step_until,
)

__all__ = [
    "AIRCRAFT_DIR",
    "OUTPUT_DIR",
    "REPO_ROOT",
    "make_fdm",
    "load_script",
    "initialize",
    "run",
    "step_until",
]
