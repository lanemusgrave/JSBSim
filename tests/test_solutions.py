"""Run every module solution script with --fast and check it exits cleanly.

These are slow-ish (a few minutes in total), so they are marked ``slow``:
    pytest -m "not slow"   # skip them
    pytest -m slow         # only them
"""

import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
# Helper modules imported by other scripts, not runnable lessons:
NOT_SCRIPTS = {"frames.py", "reference_model.py", "build_gnc_trainer.py"}

SCRIPTS = sorted(p for p in (REPO / "modules").glob("*/solutions/*.py") if p.name not in NOT_SCRIPTS)


@pytest.mark.slow
@pytest.mark.parametrize("script", SCRIPTS, ids=lambda p: f"{p.parents[1].name}/{p.name}")
def test_solution_runs(script, tmp_path):
    env = dict(os.environ, MPLBACKEND="Agg")
    proc = subprocess.run([sys.executable, str(script), "--fast"], cwd=REPO, env=env,
                          capture_output=True, text=True, timeout=600)
    assert proc.returncode == 0, f"{script.name} failed:\n{proc.stdout[-3000:]}\n{proc.stderr[-3000:]}"
    assert "Traceback" not in proc.stderr
