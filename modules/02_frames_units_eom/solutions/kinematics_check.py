"""Module 02 - check JSBSim's kinematics against hand-written frame math.

Fly a trimmed C172 through a rolling pull-up and verify, sample by sample:
  1. alpha, beta from (u, v, w)                       vs aero/alpha-rad, aero/beta-rad
  2. NED velocity = C_n/b [u v w]^T                   vs velocities/v-north/east/down-fps
  3. Euler-angle kinematics from (p, q, r)             vs velocities/phidot... -rad_sec
  4. flight-path angle: sin(gamma) = -vD / |V_ned|     vs flight-path/gamma-rad
  5. Newton in body axes: udot = (Fx_total + W_x)/m + r v - q w  vs accelerations/udot-ft_sec2

    python modules/02_frames_units_eom/solutions/kinematics_check.py [--show]
"""

import sys
from pathlib import Path

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402
from gnclab.signals import doublet, pulse  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
from frames import alpha_beta, dcm_body_to_ned, euler_rates  # noqa: E402

fdm = make_fdm("c172x")
initialize(fdm, {"lat-geod-deg": 33.67, "long-gc-deg": -117.99, "h-sl-ft": 4000,
                 "vc-kts": 100, "psi-true-deg": 45})
fdm.do_trim(1)
da0, de0 = fdm["fcs/aileron-cmd-norm"], fdm["fcs/elevator-cmd-norm"]


def pilot(f, t):
    f["fcs/aileron-cmd-norm"] = da0 + pulse(t, 1.0, 1.5, 0.4) + doublet(t, 6.0, 1.0, 0.2)
    f["fcs/elevator-cmd-norm"] = de0 + pulse(t, 3.0, 2.0, -0.15)


P = {k: k for k in [
    "velocities/u-fps", "velocities/v-fps", "velocities/w-fps",
    "velocities/p-rad_sec", "velocities/q-rad_sec", "velocities/r-rad_sec",
    "attitude/phi-rad", "attitude/theta-rad", "attitude/psi-rad",
    "aero/alpha-rad", "aero/beta-rad", "flight-path/gamma-rad",
    "velocities/v-north-fps", "velocities/v-east-fps", "velocities/v-down-fps",
    "velocities/phidot-rad_sec", "velocities/thetadot-rad_sec", "velocities/psidot-rad_sec",
    "accelerations/udot-ft_sec2", "forces/fbx-total-lbs", "forces/fbx-weight-lbs",
    "inertia/mass-slugs",
]}
df = run(fdm, 8.0 if args.fast else 15.0, P, callback=pilot)
u, v, w = (df[f"velocities/{c}-fps"].to_numpy() for c in "uvw")
p, q, r = (df[f"velocities/{c}-rad_sec"].to_numpy() for c in "pqr")
phi, th, psi = (df[f"attitude/{c}-rad"].to_numpy() for c in ("phi", "theta", "psi"))

# 1. alpha / beta (no wind here, so air-relative == ground-relative velocity)
a, b, V = alpha_beta(u, v, w)
err_alpha = np.max(np.abs(a - df["aero/alpha-rad"])) * 57.3
err_beta = np.max(np.abs(b - df["aero/beta-rad"])) * 57.3

# 2. body -> NED
ned = np.array([dcm_body_to_ned(phi[k], th[k], psi[k]) @ [u[k], v[k], w[k]] for k in range(len(df))])
jsb_ned = df[["velocities/v-north-fps", "velocities/v-east-fps", "velocities/v-down-fps"]].to_numpy()
err_ned = np.max(np.abs(ned - jsb_ned))

# 3. Euler kinematics
phid, thd, psid = euler_rates(phi, th, p, q, r)
err_eul = max(np.max(np.abs(phid - df["velocities/phidot-rad_sec"])),
              np.max(np.abs(thd - df["velocities/thetadot-rad_sec"])),
              np.max(np.abs(psid - df["velocities/psidot-rad_sec"]))) * 57.3
# ...and against a numerical derivative of the attitude itself
t = df.index.to_numpy()
err_eul_num = np.max(np.abs(np.gradient(phi, t) - phid)[2:-2]) * 57.3

# 4. flight path angle
gamma = np.arcsin(-jsb_ned[:, 2] / np.linalg.norm(jsb_ned, axis=1))
err_gamma = np.max(np.abs(gamma - df["flight-path/gamma-rad"])) * 57.3

# 5. Newton's 2nd law in a rotating (body) frame.  JSBSim's "total" force
#    excludes gravity, so add the body-axis weight component back in.
m = df["inertia/mass-slugs"].to_numpy()
udot_pred = (df["forces/fbx-total-lbs"] + df["forces/fbx-weight-lbs"]) / m + r * v - q * w
udot = df["accelerations/udot-ft_sec2"].to_numpy()
err_udot = np.max(np.abs(udot_pred - udot))
rel_udot = err_udot / np.max(np.abs(udot))

print(f"1. alpha err {err_alpha:.2e} deg, beta err {err_beta:.2e} deg")
print(f"2. NED velocity err {err_ned:.2e} ft/s")
print(f"3. Euler-rate err vs JSBSim {err_eul:.2e} deg/s; vs numerical d(phi)/dt {err_eul_num:.3f} deg/s")
print(f"4. gamma err {err_gamma:.2e} deg")
print(f"5. udot err {err_udot:.3f} ft/s^2 ({100 * rel_udot:.1f}% of max |udot|) - "
      "rotating-Earth terms + one-step staggering")

df["alpha_mine_deg"] = a * 57.3
df["alpha_jsb_deg"] = df["aero/alpha-rad"] * 57.3
df["udot_mine"] = udot_pred
df["udot_jsb"] = udot
df["phi_deg"] = phi * 57.3
df["theta_deg"] = th * 57.3
fig, _ = timehistory(df, ["phi_deg", "theta_deg", "alpha_jsb_deg", "alpha_mine_deg", "udot_jsb", "udot_mine"],
                     title="C172 rolling pull-up: JSBSim vs. hand kinematics")
save(fig, "02_frames_units_eom", "kinematics_check", show=args.show)
