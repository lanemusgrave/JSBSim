"""Module 05 - trim the C172 across its envelope and build performance charts.

  1. Level-flight sweep 50..115 KCAS at 4000 ft: alpha, trim, throttle, drag,
     power required -> the drag "bucket", the power curve, and a fitted polar.
  2. Steady climbs at 75 KCAS: throttle needed vs flight-path angle.
  3. Steady coordinated turns: load factor, turn rate and alpha vs bank.

    python modules/05_trim_performance/solutions/c172_performance.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from gnclab import initialize, make_fdm  # noqa: E402
from gnclab.plotting import save  # noqa: E402
from gnclab.trim import TrimError, trim  # noqa: E402
from gnclab.units import KT2FPS  # noqa: E402

ALT = 4000.0


def trimmed(ic, mode="longitudinal"):
    fdm = make_fdm("c172x")
    initialize(fdm, {"lat-geod-deg": 33.0, "h-sl-ft": ALT, **ic})
    try:
        trim(fdm, mode)
    except TrimError:
        return None
    return fdm


# --- 1. level-flight sweep ---------------------------------------------------
rows = []
speeds = np.arange(50, 116, 15 if args.fast else 5)
for kcas in speeds:
    fdm = trimmed({"vc-kts": float(kcas)})
    if fdm is None:
        print(f"{kcas:5.0f} KCAS: no trim (beyond the thrust available)")
        continue
    qS = fdm["aero/qbar-psf"] * fdm["metrics/Sw-sqft"]
    # JSBSim stores the wind-axis aero forces as DRAG, SIDE, LIFT magnitudes
    # (drag positive aft, lift positive up) - not as a right-handed x,y,z.
    drag = fdm["forces/fwx-aero-lbs"]
    lift = fdm["forces/fwz-aero-lbs"]
    V = fdm["velocities/vtrue-fps"]
    rows.append({"KCAS": kcas, "KTAS": V / KT2FPS, "alpha_deg": fdm["aero/alpha-deg"],
                 "pitch_trim": fdm["fcs/pitch-trim-cmd-norm"], "throttle": fdm["fcs/throttle-cmd-norm[0]"],
                 "drag_lbf": drag, "P_req_hp": drag * V / 550.0, "CL": lift / qS, "CD": drag / qS,
                 "shaft_hp": fdm["propulsion/engine/power-hp"]})
lvl = pd.DataFrame(rows)
print(lvl.round(3).to_string(index=False))

# Fit CD = CD0 + K CL^2 (least squares on the clean, unstalled points)
A = np.c_[np.ones(len(lvl)), lvl.CL ** 2]
(cd0, k), *_ = np.linalg.lstsq(A, lvl.CD, rcond=None)
ld_max = 1 / (2 * np.sqrt(cd0 * k))
print(f"\nfitted polar: CD = {cd0:.4f} + {k:.4f} CL^2  ->  (L/D)max = {ld_max:.1f} at CL = {np.sqrt(cd0 / k):.2f}")
i_md, i_mp = lvl.drag_lbf.idxmin(), lvl.P_req_hp.idxmin()
print(f"minimum drag at {lvl.KCAS[i_md]:.0f} KCAS (best glide/range-ish), "
      f"minimum power at {lvl.KCAS[i_mp]:.0f} KCAS (best endurance)")

# --- 2. climbs ---------------------------------------------------------------
climb = []
for g in np.arange(0, 7.1, 3 if args.fast else 1):
    fdm = trimmed({"vc-kts": 75.0, "gamma-deg": float(g)})
    climb.append({"gamma_deg": g, "throttle": fdm["fcs/throttle-cmd-norm[0]"] if fdm else np.nan,
                  "ROC_fpm": fdm["velocities/h-dot-fps"] * 60 if fdm else np.nan})
climb = pd.DataFrame(climb)
print("\nclimbs at 75 KCAS\n" + climb.round(3).to_string(index=False))

# --- 3. turns ----------------------------------------------------------------
turns = []
for phi in (15, 30, 45, 60):
    fdm = trimmed({"vc-kts": 90.0, "phi-deg": float(phi)}, mode="turn")
    if fdm:
        turns.append({"bank_deg": phi, "nz_g": fdm["accelerations/Nz"], "1/cos(phi)": 1 / np.cos(np.radians(phi)),
                      "turn_rate_dps": np.degrees(fdm["velocities/psidot-rad_sec"]),
                      "g*tan(phi)/V [dps]": np.degrees(32.174 * np.tan(np.radians(phi)) / fdm["velocities/vtrue-fps"]),
                      "alpha_deg": fdm["aero/alpha-deg"], "throttle": fdm["fcs/throttle-cmd-norm[0]"]})
turns = pd.DataFrame(turns)
print("\nlevel turns at 90 KCAS\n" + turns.round(3).to_string(index=False))

# --- plots -------------------------------------------------------------------
fig, ax = plt.subplots(2, 2, figsize=(11, 8))
ax[0, 0].plot(lvl.KCAS, lvl.drag_lbf, "o-")
ax[0, 0].set(xlabel="KCAS", ylabel="drag [lbf]", title="drag bucket")
ax[0, 1].plot(lvl.KCAS, lvl.P_req_hp, "o-", label="power required (D V)")
ax[0, 1].plot(lvl.KCAS, lvl.shaft_hp, "s--", label="engine shaft power")
ax[0, 1].set(xlabel="KCAS", ylabel="hp", title="power curves")
ax[0, 1].legend()
cl = np.linspace(lvl.CL.min(), lvl.CL.max(), 50)
ax[1, 0].plot(lvl.CD, lvl.CL, "o", label="trim points")
ax[1, 0].plot(cd0 + k * cl ** 2, cl, label=f"fit CD0={cd0:.4f}, K={k:.4f}")
ax[1, 0].set(xlabel="CD", ylabel="CL", title="drag polar")
ax[1, 0].legend()
ax[1, 1].plot(lvl.KCAS, lvl.pitch_trim, "o-")
ax[1, 1].set(xlabel="KCAS", ylabel="pitch trim cmd [-]", title="trim vs speed (stability!)")
for a in ax.flat:
    a.grid(alpha=0.3)
fig.tight_layout()
save(fig, "05_trim_performance", "c172_performance", show=args.show)
