#!/usr/bin/env bash
# One-time setup on Linux, macOS or WSL2.  Run from anywhere:
#
#   bash scripts/setup.sh
#
# Creates .venv, installs JSBSim + the lesson helpers, and runs the install check.
set -euo pipefail
cd "$(dirname "$0")/.."

PY=${PYTHON:-}
if [ -z "$PY" ]; then
  for cand in python3.12 python3.13 python3.11 python3.10 python3; do
    if command -v "$cand" >/dev/null 2>&1; then PY=$cand; break; fi
  done
fi
if [ -z "$PY" ]; then
  echo "Python 3.10+ not found (Ubuntu/WSL: sudo apt install python3 python3-venv)" >&2
  exit 1
fi
echo "Using: $($PY --version) ($PY)"

[ -d .venv ] || "$PY" -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m pip install -e .
.venv/bin/python scripts/verify_install.py

echo
echo "Next time, activate the environment with:  source .venv/bin/activate"
