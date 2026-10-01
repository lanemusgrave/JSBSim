"""Module 06 exercise - solution: T-38 modes across the envelope."""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from gnclab import initialize, make_fdm  # noqa: E402
from gnclab.linear import LAT_STATES, LONG_STATES, linearize, modes  # noqa: E402
from gnclab.trim import trim  # noqa: E402

CONDITIONS = [
    ("low/slow", 5000.0, 200.0),
    ("low/fast", 5000.0, 400.0),
    ("high/slow", 25000.0, 200.0),
    ("high/fast", 25000.0, 330.0),
]
if args.fast:
    CONDITIONS = CONDITIONS[:2]

rows = []
for name, alt, kcas in CONDITIONS:
    fdm = make_fdm("T38")
    initialize(fdm, {"lat-geod-deg": 29.5, "h-sl-ft": alt, "vc-kts": kcas})
    trim(fdm, "full")
    lin = linearize(fdm)
    lon = modes(lin.subsystem(LONG_STATES, ["DeCmd"]).A)
    lat = modes(lin.subsystem(LAT_STATES, ["DaCmd"]).A)
    osc_lon = lon[lon.eig_imag > 0].sort_values("wn", ascending=False)
    dr = lat[lat.eig_imag > 0].iloc[0]
    real = lat[lat.eig_imag == 0].sort_values("wn", ascending=False)
    V = fdm["velocities/vtrue-fps"]
    rows.append({
        "condition": name, "KTAS": fdm["velocities/vtrue-kts"], "qbar_psf": fdm["aero/qbar-psf"],
        "SP wn": osc_lon.iloc[0].wn, "SP zeta": osc_lon.iloc[0].zeta,
        "PH period": osc_lon.iloc[1].period_s, "PH est sqrt2*pi*V/g": np.sqrt(2) * np.pi * V / 32.174,
        "DR wn": dr.wn, "DR zeta": dr.zeta,
        "roll tau": 1 / real.iloc[0].wn,
        "spiral lambda": real.iloc[-1].eig_real,
    })
table = pd.DataFrame(rows).set_index("condition")
print(table.round(3).T.to_string())

# TODO 3 answers (see answers.md for discussion):
# a) SP wn rises with qbar: M_alpha ~ qbar S c Cm_alpha / Iyy, so wn ~ sqrt(qbar) ~ V
#    at fixed altitude, and at fixed TAS ~ sqrt(rho).
# b) The Lanchester phugoid period estimate is within ~10-30 % (ignores Mach and
#    thrust/drag variation with speed).
# c) See the table: the high/slow condition has the lowest DR damping.
