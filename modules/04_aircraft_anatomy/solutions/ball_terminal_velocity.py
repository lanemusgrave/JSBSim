"""Module 04 - the m04_ball: terminal velocity with and without a parachute.

Analytic terminal velocity (drag = weight):  V_t = sqrt(2 m g / (rho CD S))
It depends on altitude through rho, so V_t falls as the ball descends.

    python modules/04_aircraft_anatomy/solutions/ball_terminal_velocity.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402
from gnclab.units import FT2M, G0_MPS2  # noqa: E402

M, CD, S = 15.0, 0.47, np.pi * 0.2 ** 2
CDS_CHUTE = 4.5  # m^2
SLUGFT3_TO_KGM3 = 515.379
CHUTE_AGL_FT = 3000.0

fdm = make_fdm("m04_ball")
initialize(fdm, {"lat-geod-deg": 33.0, "h-sl-ft": 10000.0, "vt-fps": 0.0}, engines_running=False)


def deploy(f, t):
    if f["position/h-agl-ft"] < CHUTE_AGL_FT:
        f["fcs/chute-cmd-norm"] = 1.0


df = run(fdm, 60.0 if args.fast else 240.0,
         {"h_agl_ft": "position/h-agl-ft", "v_down_fps": "velocities/v-down-fps",
          "rho": "atmosphere/rho-slugs_ft3", "chute": "fcs/chute-pos-norm"},
         callback=deploy)
df = df[df.h_agl_ft > 1.0].copy()
rho = df.rho * SLUGFT3_TO_KGM3
cds = CD * S + df.chute * CDS_CHUTE
df["v_terminal_fps"] = np.sqrt(2 * M * G0_MPS2 / (rho * cds)) / FT2M

free = df[(df.chute == 0) & (df.index > 25)]
print(f"free fall, 25 s .. deploy: JSBSim {free.v_down_fps.iloc[-1]:.1f} ft/s vs analytic "
      f"{free.v_terminal_fps.iloc[-1]:.1f} ft/s at {free.h_agl_ft.iloc[-1]:.0f} ft AGL")
if (df.chute > 0.99).any():
    under = df[(df.chute > 0.99) & (df.h_agl_ft > 50)]       # steady descent, before touchdown
    late = under[under.index > under.index[0] + 10]
    if len(late):
        print(f"under canopy: JSBSim {late.v_down_fps.median():.2f} ft/s vs analytic "
              f"{late.v_terminal_fps.median():.2f} ft/s ({late.v_down_fps.median() * FT2M:.1f} m/s)")
fig, _ = timehistory(df, ["h_agl_ft", "v_down_fps", "v_terminal_fps", "chute"],
                     title="m04_ball: free fall from 10,000 ft, chute at 3,000 ft AGL")
save(fig, "04_aircraft_anatomy", "ball_terminal_velocity", show=args.show)
