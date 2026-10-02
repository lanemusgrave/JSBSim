"""Module 06 - linearize the C172 and identify its dynamic modes.

Compares the exact eigenvalues of JSBSim's linear model with the classic
textbook approximations (Nelson ch. 4-5, Etkin):

  short period : 2x2 [alpha, q] block of A
  phugoid      : Lanchester  wn = sqrt(2) g / V,  zeta = 1 / (sqrt(2) L/D)
  roll         : lambda ~ L_p  (the [p, p] element of A)
  Dutch roll   : 2x2 [beta, r] block of A
  spiral       : the slow real root left over

    python modules/06_linear_models_modes/solutions/c172_modes.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm  # noqa: E402
from gnclab.linear import LAT_STATES, LONG_STATES, linearize, modes  # noqa: E402
from gnclab.plotting import save  # noqa: E402
from gnclab.trim import trim  # noqa: E402

G = 32.174
fdm = make_fdm("c172x")
initialize(fdm, {"lat-geod-deg": 33, "h-sl-ft": 4000, "vc-kts": 100})
trim(fdm, "full")
V = fdm["velocities/vtrue-fps"]
LD = fdm["forces/fwz-aero-lbs"] / fdm["forces/fwx-aero-lbs"]
lin = linearize(fdm)

lon = lin.subsystem(LONG_STATES, ["DeCmd", "ThtlCmd"])
lat = lin.subsystem(LAT_STATES, ["DaCmd", "DrCmd"])
pd_opts = {"float_format": lambda x: f"{x:9.4f}"}
print("Longitudinal A (states Vt [ft/s], Alpha [rad], Theta [rad], Q [rad/s]):")
print(lon.as_frame("A").to_string(**pd_opts))
print("\nLateral A (states Beta, Phi [rad], P, R [rad/s]):")
print(lat.as_frame("A").to_string(**pd_opts))

m_lon = modes(lon.A, lon.x_names)
m_lat = modes(lat.A, lat.x_names)
cols = ["eig_real", "eig_imag", "wn", "zeta", "period_s", "t_half_or_double_s"]
print("\nLongitudinal modes\n" + m_lon[cols].round(4).to_string())
print("\nLateral modes\n" + m_lat[cols].round(4).to_string())


def eig2(A, names, pick):
    i = [names.index(p) for p in pick]
    lam = np.linalg.eigvals(A[np.ix_(i, i)])
    return lam[np.argmax(lam.imag)]


sp, ph = m_lon.iloc[0], m_lon.iloc[1]
sp_approx = eig2(lon.A, lon.x_names, ["Alpha", "Q"])
ph_wn, ph_zeta = np.sqrt(2) * G / V, 1 / (np.sqrt(2) * LD)
rows = [("short period wn", sp.wn, abs(sp_approx)), ("short period zeta", sp.zeta, -sp_approx.real / abs(sp_approx)),
        ("phugoid wn", ph.wn, ph_wn), ("phugoid zeta", ph.zeta, ph_zeta)]
dr = m_lat[m_lat.eig_imag > 0].iloc[0]
dr_approx = eig2(lat.A, lat.x_names, ["Beta", "R"])
roll = m_lat[(m_lat.eig_imag == 0)].iloc[0]
spiral = m_lat[(m_lat.eig_imag == 0)].iloc[-1]
rows += [("Dutch roll wn", dr.wn, abs(dr_approx)), ("Dutch roll zeta", dr.zeta, -dr_approx.real / abs(dr_approx)),
         ("roll time const [s]", 1 / roll.wn, -1 / lat.A[2, 2])]
print(f"\n{'quantity':<22s}{'exact':>10s}{'approx':>10s}")
for name, exact, approx in rows:
    print(f"{name:<22s}{exact:10.3f}{approx:10.3f}")
print(f"spiral: lambda = {spiral.eig_real:+.4f} 1/s ({'stable' if spiral.stable else 'UNSTABLE'}, "
      f"time to {'half' if spiral.stable else 'double'} {spiral.t_half_or_double_s:.0f} s)")
print(f"\n(V = {V:.1f} ft/s, L/D = {LD:.1f})")

# s-plane plot
fig, ax = plt.subplots(1, 2, figsize=(11, 4.5))
for a, A, title in ((ax[0], lon.A, "longitudinal"), (ax[1], lat.A, "lateral-directional")):
    lam = np.linalg.eigvals(A)
    a.plot(lam.real, lam.imag, "x", ms=10, mew=2)
    a.axvline(0, color="k", lw=0.8)
    a.axhline(0, color="k", lw=0.8)
    for z in (0.2, 0.5, 0.7):  # damping-ratio rays
        r = np.linspace(0, 8, 2)
        a.plot(-z * r, np.sqrt(1 - z ** 2) * r, ":", color="gray")
    a.set(xlabel="real [1/s]", ylabel="imag [rad/s]", title=f"C172 {title} poles")
    a.grid(alpha=0.3)
fig.tight_layout()
save(fig, "06_linear_models_modes", "c172_poles", show=args.show)
