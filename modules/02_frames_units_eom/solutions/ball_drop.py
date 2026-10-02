"""Module 02 - drop a ball over a rotating, oblate Earth.

The bundled 'ball' has no aerodynamic drag, so a drop is a pure test of the
gravity model and the equations of motion.  We compare JSBSim against:

  * flat-Earth free fall with *effective* gravity
        g_eff = g_gravitation(lat, h) - omega^2 * R * cos^2(lat)    (centrifugal)
  * the Coriolis drift: falling bodies drift EAST,
        v_east(t) ~= omega * g * t^2 * cos(lat)

Then we let it hit the ground to see what an integration time step that is
too large does to a stiff contact spring (Module 01's dt lesson, the hard way).

    python modules/02_frames_units_eom/solutions/ball_drop.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402

OMEGA = 7.292115e-5  # Earth rotation rate [rad/s]
R_EQ_FT = 20925646.3  # WGS84 equatorial radius [ft]
LAT = 0.0
H0 = 10000.0

fdm = make_fdm("ball")
initialize(fdm, {"lat-geod-deg": LAT, "long-gc-deg": 0.0, "h-sl-ft": H0, "vt-fps": 0.0},
           engines_running=False)
g_grav = fdm["accelerations/gravity-ft_sec2"]

props = {"h_ft": "position/h-sl-ft", "v_down_fps": "velocities/v-down-fps",
         "v_east_fps": "velocities/v-east-fps", "g_ftps2": "accelerations/gravity-ft_sec2"}
df = run(fdm, 20.0, props)
t = df.index.to_numpy()

g_eff = g_grav - OMEGA ** 2 * R_EQ_FT * np.cos(np.radians(LAT)) ** 2
df["v_down_flat"] = g_eff * t
df["h_flat"] = H0 - 0.5 * g_eff * t ** 2
df["v_east_coriolis"] = OMEGA * g_eff * t ** 2 * np.cos(np.radians(LAT))

print(f"gravitation at start: {g_grav:.4f} ft/s^2 (J2 model), effective g: {g_eff:.4f} ft/s^2")
print(f"after {t[-1]:.0f} s: h = {df.h_ft.iloc[-1]:.1f} ft (flat model {df.h_flat.iloc[-1]:.1f}), "
      f"v_down = {df.v_down_fps.iloc[-1]:.2f} ft/s (flat {df.v_down_flat.iloc[-1]:.2f})")
print(f"east drift velocity: {df.v_east_fps.iloc[-1]:.3f} ft/s "
      f"(Coriolis estimate {df.v_east_coriolis.iloc[-1]:.3f} ft/s)")
print("Remaining difference: g grows as the ball falls (closer to Earth's centre) - see g_ftps2.")

fig, _ = timehistory(df, ["h_ft", "h_flat", "v_down_fps", "v_down_flat", "v_east_fps", "v_east_coriolis", "g_ftps2"],
                     title="Ball dropped from 10,000 ft at the equator (no drag)")
save(fig, "02_frames_units_eom", "ball_drop", show=args.show)

# --- Part 2: hitting the ground at different time steps -------------------
print("\nGround impact from 1000 ft (contact: k = 1e4 lbf/ft, c = 2e5 lbf*s/ft, m ~ 622 slug)")
runs = {}
for hz in (120, 240, 480):
    f = make_fdm("ball", dt=1.0 / hz)
    initialize(f, {"lat-geod-deg": LAT, "h-sl-ft": 1000.0, "vt-fps": 0.0}, engines_running=False)
    d = run(f, 10.0, {"h_agl_ft": "position/h-agl-ft", "v_down_fps": "velocities/v-down-fps"})
    runs[f"{hz} Hz"] = d
    c_over_m = 2e5 / f["inertia/mass-slugs"]
    print(f"  {hz:4d} Hz: c/m*dt = {c_over_m / hz:4.2f}  impact {d.v_down_fps.max():6.1f} ft/s, "
          f"max rebound {max(0.0, -d.v_down_fps.min()):6.1f} ft/s")
print("A damper is a real eigenvalue -c/m.  Adams-Bashforth 2 (JSBSim's default for\n"
      "translational velocity) is only stable for (c/m)*dt < 1 on that axis, so at\n"
      "120 and 240 Hz the integrator itself manufactures the rebound.  The physical\n"
      "(heavily overdamped) answer is what 480 Hz gives: no bounce at all.")
fig, _ = timehistory(runs, ["h_agl_ft", "v_down_fps"], title="Ball hitting the ground: effect of dt")
save(fig, "02_frames_units_eom", "ball_impact_dt", show=args.show)
