"""Module 17 - verify the <moving_average> component added in the fork.

A. C++ executable: run the rig script with the fork's JSBSim binary and
   compare rig/avg against a numpy boxcar of rig/input.
B. Stock pip wheel: load the same rig and show what happens to an unknown
   component (spoiler: it is silently dropped, and the model still loads).
C. Fork Python module (if built with scripts/build_jsbsim_python.sh): same
   check as A, from Python.

Skips A or C with instructions when that build doesn't exist yet, so
the smoke tests pass on a fresh checkout.

    python modules/17_jsbsim_fork/solutions/check_moving_average.py [--fast]
"""

import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[3]
RIG = REPO / "modules" / "17_jsbsim_fork" / "rig"
EXT = REPO / "external"
N = 12


def find_exe():
    env = os.environ.get("JSBSIM_EXE")
    cands = [Path(env)] if env else []
    b = EXT / "jsbsim" / "build" / "src"
    cands += [b / "JSBSim", b / "Release" / "JSBSim.exe", b / "JSBSim.exe"]
    return next((c for c in cands if c.is_file()), None)


def boxcar(x, x0):
    """Reference: mean of the last N samples, buffer primed with x0 (the run_ic value)."""
    buf = np.r_[np.full(N - 1, x0), x]
    return np.convolve(buf, np.ones(N) / N, mode="valid")


def check(name, t, x, y, x0):
    err = np.abs(y - boxcar(x, x0)).max()
    vib = y[(t > 1.0)] - 1.0                          # after the step, only the 10 Hz part remains
    print(f"  {name}: max |avg - numpy boxcar| = {err:.2e};  10 Hz residual after the step: "
          f"{np.abs(vib).max():.2e} (input amplitude 0.2)")
    return err < 1e-9


def part_a(out):
    exe = find_exe()
    print("A. fork C++ executable")
    if exe is None:
        print("  SKIPPED: no fork build found. Apply the patch and build:\n"
              "    bash scripts/build_jsbsim_source.sh\n"
              "    cd external/jsbsim && git apply ../../modules/17_jsbsim_fork/solutions/0001-*.patch\n"
              "    cmake --build build --target JSBSim -j4")
        return True
    # Output files are written relative to --root.  (In v1.3.1, --outputlogfile
    # did not redirect this aircraft-defined <output>; on later versions it does.)
    written = RIG / "m17_rig.csv"
    written.unlink(missing_ok=True)
    proc = subprocess.run([str(exe), f"--root={RIG}", "--script=scripts/m17_rig.xml"], capture_output=True, text=True)
    if "type: UNKNOWN" in proc.stdout or not written.exists():
        print(f"  {exe} does not know <moving_average>: apply the patch and rebuild.")
        return True
    csv = out / "m17_rig_cpp.csv"
    written.replace(csv)
    df = pd.read_csv(csv)
    t, x, y = df["Time"].to_numpy(), df["/fdm/jsbsim/rig/input"].to_numpy(), df["/fdm/jsbsim/rig/avg"].to_numpy()
    # row 0 is written at run_ic (t = 0): that's the priming value
    return check(f"{exe.relative_to(REPO)}", t[1:], x[1:], y[1:], x[0])


def fly_python(jsbsim_module):
    fdm = jsbsim_module.FGFDMExec(str(RIG))
    fdm.set_debug_level(0)
    fdm.load_model("m17_rig")
    fdm.disable_output()                       # the rig's <output> would write into the rig folder
    fdm.set_dt(1 / 120)
    fdm.run_ic()
    x0 = fdm["rig/input"]
    t, x, y = [], [], []
    for _ in range(240):
        fdm.run()
        t.append(fdm.get_sim_time())
        x.append(fdm["rig/input"])
        y.append(fdm["rig/avg"])
    return np.array(t), np.array(x), np.array(y), x0


def part_b():
    import jsbsim
    print(f"\nB. stock pip wheel (jsbsim {jsbsim.__version__})")
    fdm = jsbsim.FGFDMExec(str(RIG))
    fdm.set_debug_level(0)
    ok = fdm.load_model("m17_rig")
    print(f"  load_model returned {ok}: the model LOADS.")
    try:
        fdm["rig/avg"]
        print("  ...and rig/avg exists?!  (unexpected)")
    except KeyError as e:
        print(f"  ...but {e}: the unknown component was dropped with only a log message.")
        print("  Lesson: the team's Python tools must use the fork's wheel, and model loading")
        print("  should fail loudly in CI (check for the property, or scan the log for 'Unknown FCS component').")


def part_c():
    py = EXT / "pyfork-venv" / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    pkg = EXT / "jsbsim" / "build-py" / "tests"
    print("\nC. fork Python module")
    if not (py.exists() and (pkg / "jsbsim").is_dir()):
        print("  SKIPPED: build it with  bash scripts/build_jsbsim_python.sh")
        return True
    code = ("import sys; sys.path.insert(0, %r); sys.path.insert(1, %r)\n"
            "import jsbsim, check_moving_average as c\n"
            "t, x, y, x0 = c.fly_python(jsbsim)\n"
            "print('  jsbsim', jsbsim.__version__, 'from', jsbsim.__file__)\n"
            "sys.exit(0 if c.check('fork Python module', t, x, y, x0) else 1)\n") % (str(pkg), str(Path(__file__).parent))
    env = dict(os.environ, PYTHONPATH=str(REPO / "src"))
    proc = subprocess.run([str(py), "-c", code], capture_output=True, text=True, env=env)
    print(proc.stdout.rstrip())
    if proc.returncode:
        print(proc.stderr[-2000:])
    return proc.returncode == 0


def main():
    from gnclab.cli import parse_args          # imported here: part C re-imports this file in the
    from gnclab.plotting import outdir         # fork's venv, which has no matplotlib

    parse_args(__doc__)
    out = outdir("17_jsbsim_fork")
    ok = part_a(out)
    part_b()
    ok = part_c() and ok
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
