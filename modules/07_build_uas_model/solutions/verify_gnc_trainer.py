"""Module 07 - verify the JSBSim gnc_trainer against an independent model.

  1. Trim both models at 25 m/s; compare alpha, elevator, throttle.
  2. Linearize both (gnclab.linear.linearize_fd on JSBSim, central differences
     on reference_model.py) and compare every mode.
  3. Sanity numbers a reviewer asks for: stall speed, max level speed, static
     margin, max climb rate.

    python modules/07_build_uas_model/solutions/verify_gnc_trainer.py [--show]
"""

import sys
from pathlib import Path

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
import reference_model as ref  # noqa: E402
from build_gnc_trainer import P  # noqa: E402

from gnclab import initialize, make_fdm  # noqa: E402
from gnclab.linear import linearize_fd, modes  # noqa: E402
from gnclab.trim import TrimError, trim  # noqa: E402

VA = 25.0
FT = 0.3048
INPUTS = ["fcs/elevator-cmd-norm", "fcs/aileron-cmd-norm", "fcs/rudder-cmd-norm"]


def jsb_trimmed(Va, alt_ft=330.0, mode="full", **ic):
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": alt_ft, "vt-fps": Va / FT, **ic})
    trim(fdm, mode)
    return fdm


# --- 1. trim ---------------------------------------------------------------
fdm = jsb_trimmed(VA)
rho = fdm["atmosphere/rho-slugs_ft3"] * 515.379
x0, u0 = ref.trim_level(VA, rho)
cmp = pd.DataFrame({
    "JSBSim": [fdm["aero/alpha-deg"], np.degrees(fdm["fcs/elevator-pos-rad"]), fdm["fcs/throttle-cmd-norm[0]"],
               np.degrees(fdm["fcs/aileron-pos-rad"]), np.degrees(fdm["fcs/rudder-pos-rad"])],
    "reference": [np.degrees(x0[1]), np.degrees(u0[0]), u0[3], np.degrees(u0[1]), np.degrees(u0[2])],
}, index=["alpha [deg]", "elevator [deg]", "throttle", "aileron [deg]", "rudder [deg]"])
print(f"Trim at {VA} m/s, rho = {rho:.4f} kg/m^3\n" + cmp.round(4).to_string())

# --- 2. modes -------------------------------------------------------------------
lin = linearize_fd(fdm, INPUTS)
A_ref, _ = ref.linearize(x0, u0, rho)
# JSBSim states are in ft/s for Vt: convert A to SI so the matrices are comparable
S = np.diag([FT, 1, 1, 1, 1, 1, 1, 1])
A_jsb = S @ lin.A @ np.linalg.inv(S)
m_jsb = modes(A_jsb, lin.x_names)
m_ref = modes(A_ref, lin.x_names)
names = ["roll subsidence", "short period", "Dutch roll", "phugoid", "spiral"]


def label(df):
    out = {}
    osc = df[df.eig_imag > 0].sort_values("wn", ascending=False)
    real = df[df.eig_imag == 0].sort_values("wn", ascending=False)
    out["short period"], out["Dutch roll"], out["phugoid"] = osc.iloc[0], osc.iloc[1], osc.iloc[2]
    out["roll subsidence"], out["spiral"] = real.iloc[0], real.iloc[-1]
    return out


lj, lr = label(m_jsb), label(m_ref)
rows = []
for n in names:
    rows.append({"mode": n, "JSBSim eig": complex(lj[n].eig_real, lj[n].eig_imag),
                 "reference eig": complex(lr[n].eig_real, lr[n].eig_imag),
                 "wn err %": 100 * (lj[n].wn - lr[n].wn) / lr[n].wn,
                 "zeta JSB": lj[n].zeta, "zeta ref": lr[n].zeta})
mt = pd.DataFrame(rows).set_index("mode")
with pd.option_context("display.float_format", "{:.4f}".format):
    print("\nModes at 25 m/s\n" + mt.to_string())
worst = mt["wn err %"].abs().max()
print(f"largest natural-frequency difference: {worst:.2f} %")

# --- 3. envelope sanity ------------------------------------------------------------
speeds = np.arange(15.0, 40.0, 1.0)
ok = []
for V in speeds:
    try:
        jsb_trimmed(V)
        ok.append(V)
    except TrimError:
        pass
print(f"\nlevel-flight trim exists from {min(ok):.0f} to {max(ok):.0f} m/s at 100 m")
CLmax = max(ref.cl_static(np.radians(np.linspace(0, 30, 301))))
Vs = np.sqrt(2 * P["mass"] * 9.80665 / (rho * P["S"] * CLmax))
print(f"CLmax (alpha table) {CLmax:.2f} -> 1 g stall speed {Vs:.1f} m/s")
sm = -P["Cma"] / P["CLa"]
print(f"static margin = -Cma/CLa = {sm:.2f} cbar ({100 * sm:.0f} %)")
fdm = jsb_trimmed(VA)
fdm["fcs/throttle-cmd-norm"] = 1.0
fdm.run_ic()
excess = fdm["propulsion/thrust-N"] - fdm["forces/fwx-aero-lbs"] * 4.44822
print(f"full-throttle excess thrust at {VA} m/s: {excess:.1f} N -> climb rate ~ "
      f"{excess * VA / (P['mass'] * 9.80665):.1f} m/s")
