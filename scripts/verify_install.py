"""Check that Python, JSBSim and the lesson helpers are installed correctly.

Run from the repo root (with the virtual environment activated):

    python scripts/verify_install.py
"""

from __future__ import annotations

import os
import platform
import shutil
import subprocess
import sys

OK, FAIL = "[ OK ]", "[FAIL]"
problems = 0


def check(label: str, fn):
    global problems
    try:
        detail = fn()
        print(f"{OK} {label}: {detail}")
    except Exception as exc:  # noqa: BLE001 - report anything
        problems += 1
        print(f"{FAIL} {label}: {exc}")


def python_version():
    v = sys.version_info
    if v < (3, 10):
        raise RuntimeError(f"Python {v.major}.{v.minor} is too old; need 3.10+")
    return f"{platform.python_implementation()} {v.major}.{v.minor}.{v.micro} ({sys.executable})"


def in_venv():
    if sys.prefix == sys.base_prefix:
        raise RuntimeError("not running inside a virtual environment (activate .venv first)")
    return sys.prefix


def jsbsim_version():
    import jsbsim

    return f"jsbsim {jsbsim.__version__}, data in {jsbsim.get_default_root_dir()}"


def bundled_aircraft():
    import jsbsim

    root = jsbsim.get_default_root_dir()
    names = sorted(os.listdir(os.path.join(root, "aircraft")))
    for must in ("c172x", "f16", "T38", "ball"):
        if must not in names:
            raise RuntimeError(f"missing bundled aircraft {must}")
    return f"{len(names)} aircraft folders (c172x, f16, T38, ball, ...)"


def libraries():
    import control
    import matplotlib
    import numpy
    import pandas
    import scipy

    return (f"numpy {numpy.__version__}, scipy {scipy.__version__}, pandas {pandas.__version__}, "
            f"matplotlib {matplotlib.__version__}, control {control.__version__}")


def gnclab_sim():
    from gnclab import initialize, make_fdm, run
    from gnclab.trim import trim

    fdm = make_fdm("c172x")
    initialize(fdm, {"h-sl-ft": 5000, "vc-kts": 100})
    s = trim(fdm, "full")
    df = run(fdm, 10.0)
    dh = df["alt_ft"].iloc[-1] - df["alt_ft"].iloc[0]
    if abs(dh) > 20:
        raise RuntimeError(f"trimmed C172 drifted {dh:.1f} ft in 10 s")
    return (f"c172x trimmed at {s['V_kts']:.0f} kt, alpha {s['alpha_deg']:.2f} deg, "
            f"throttle {s['throttle']:.2f}; 10 s run, altitude change {dh:+.2f} ft")


def jsbsim_cli():
    # pip puts console programs next to python: .venv/Scripts (Windows) or .venv/bin
    name = "jsbsim.exe" if os.name == "nt" else "jsbsim"
    candidate = os.path.join(os.path.dirname(sys.executable), name)
    exe = candidate if os.path.exists(candidate) else shutil.which("jsbsim")
    if exe is None:
        raise RuntimeError("'jsbsim' program not found next to python or on PATH")
    out = subprocess.run([exe, "--version"], capture_output=True, text=True, timeout=60)
    return f"{exe} -> {out.stdout.strip() or out.stderr.strip()}"


print("JSBSim GNC curriculum - installation check\n")
check("Python", python_version)
check("Virtual environment", in_venv)
check("JSBSim Python module", jsbsim_version)
check("Bundled aircraft", bundled_aircraft)
check("Scientific libraries", libraries)
check("gnclab + a 10 s C172 flight", gnclab_sim)
check("jsbsim command-line program", jsbsim_cli)
print()
if problems:
    print(f"{problems} problem(s) found - see docs/setup.md#troubleshooting")
    sys.exit(1)
print("All good. Start with modules/00_setup/README.md")
