#!/usr/bin/env bash
# Clone and build JSBSim from source (Linux / WSL2 / macOS).
#
#   bash scripts/build_jsbsim_source.sh            # upstream v1.3.1
#   JSBSIM_REPO=<your fork url> JSBSIM_REF=<branch> bash scripts/build_jsbsim_source.sh
#
# Result: external/jsbsim/build/src/JSBSim (the C++ executable).
# Needs: git, cmake (>=3.15), a C++17 compiler.
#   Ubuntu/WSL: sudo apt install build-essential cmake git
set -euo pipefail
cd "$(dirname "$0")/.."

REPO=${JSBSIM_REPO:-https://github.com/JSBSim-Team/jsbsim.git}
REF=${JSBSIM_REF:-v1.3.1}
SRC=external/jsbsim
BUILD=$SRC/build
JOBS=${JOBS:-$( (command -v nproc >/dev/null && nproc) || sysctl -n hw.ncpu 2>/dev/null || echo 2)}

mkdir -p external
if [ ! -d "$SRC/.git" ]; then
  echo "==> Cloning $REPO ($REF) into $SRC"
  git clone --branch "$REF" "$REPO" "$SRC"
else
  echo "==> $SRC already exists; leaving the checkout as-is ($(git -C "$SRC" describe --tags --always))"
fi

echo "==> Configuring (Release)"
cmake -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_MODULE=OFF -DBUILD_DOCS=OFF

echo "==> Building with $JOBS jobs (first build takes a few minutes)"
cmake --build "$BUILD" --target JSBSim -j "$JOBS"

EXE="$BUILD/src/JSBSim"
echo "==> Smoke test: $EXE --version"
"$EXE" --version
echo "==> Smoke test: run the c1723 script (C172 takeoff + climb)"
(cd "$SRC" && "build/src/JSBSim" --script=scripts/c1723.xml --end=20 > /dev/null)
echo "==> Done. Executable: $EXE"
