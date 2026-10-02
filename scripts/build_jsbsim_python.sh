#!/usr/bin/env bash
# Build the Python module of the JSBSim checkout in external/jsbsim (your fork,
# with your patches) into a separate virtualenv, and run upstream's test suite.
#
#   bash scripts/build_jsbsim_source.sh      # first: clone (+ build the executable)
#   bash scripts/build_jsbsim_python.sh      # then: Python module + ctest
#   SKIP_TESTS=1 bash scripts/build_jsbsim_python.sh
#
# Result: external/jsbsim/build-py/tests/jsbsim (import it with
#   PYTHONPATH=external/jsbsim/build-py/tests external/pyfork-venv/bin/python)
# The course's own .venv keeps the stock pip wheel, so you can always compare
# the fork against upstream.  Windows: run this inside WSL2 (docs/source_build.md).
set -euo pipefail
cd "$(dirname "$0")/.."

SRC=external/jsbsim
BUILD=$SRC/build-py
VENV=external/pyfork-venv
JOBS=${JOBS:-$( (command -v nproc >/dev/null && nproc) || sysctl -n hw.ncpu 2>/dev/null || echo 2)}

[ -d "$SRC/.git" ] || { echo "No checkout in $SRC: run scripts/build_jsbsim_source.sh first"; exit 1; }

if [ ! -x "$VENV/bin/python" ]; then
  echo "==> Creating $VENV"
  python3 -m venv "$VENV"
fi
echo "==> Installing build/test requirements (Cython, numpy, pandas, scipy)"
"$VENV/bin/pip" install -q --upgrade pip
"$VENV/bin/pip" install -q cython setuptools wheel numpy pandas scipy

echo "==> Configuring $BUILD"
cmake -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_MODULE=ON -DBUILD_DOCS=OFF \
      -DPython3_EXECUTABLE="$PWD/$VENV/bin/python"

echo "==> Building everything (the tests need the fpectl helper module too)"
cmake --build "$BUILD" -j "$JOBS"

if [ -z "${SKIP_TESTS:-}" ]; then
  echo "==> Running upstream tests SERIALLY (several share files: ctest -j gives false failures)"
  (cd "$BUILD" && ctest --output-on-failure)
fi
echo "==> Done.  Try:  PYTHONPATH=$BUILD/tests $VENV/bin/python -c 'import jsbsim; print(jsbsim.__file__)'"
