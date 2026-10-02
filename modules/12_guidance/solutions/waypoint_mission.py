"""Module 12 - a waypoint mission, and path following vs. "direct-to" homing.

Mission: 4 waypoints with altitude changes, flown with gnclab.guidance's
WaypointManager (straight-line vector field legs, half-plane switching, loiter
orbit at the end) in an 6 m/s wind from the west, realistic sensors/actuators.

Comparison: a leg flown from 150 m off-track with the XML direct-to guidance
(JSBSim <waypoint_heading>) vs the vector field.  Direct-to has no notion of
the leg, so it never returns to it.  (Because our inner loop holds GPS *course*,
direct-to does NOT drift downwind in the crosswind; the classic curved
"homing" track appears when a *heading* hold chases the bearing.)

    python modules/12_guidance/solutions/waypoint_mission.py [--show]
"""

import math

from gnclab.cli import parse_args

args = parse_args(__doc__)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.autopilot import engage_lateral, engage_longitudinal, realism  # noqa: E402
from gnclab.guidance import WaypointManager, ne_from_latlon  # noqa: E402
from gnclab.plotting import save  # noqa: E402
from gnclab.trim import trim  # noqa: E402

FPS = 1 / 0.3048
LAT0, LON0 = 33.7, -117.9
WIND_E = 6.0
WPS = [(0, 0, 330), (800, 0, 500), (800, 800, 500), (0, 800, 400), (0, 1600, 400)]  # north m, east m, alt ft


def setup():
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": LAT0, "long-gc-deg": LON0, "h-sl-ft": 330, "vt-fps": 25 * FPS,
                     "psi-true-deg": 0.0})
    trim(fdm, "full")
    realism(fdm, True)
    engage_longitudinal(fdm)
    engage_lateral(fdm)
    fdm["atmosphere/wind-east-fps"] = WIND_E * FPS
    return fdm


PROPS = {"lat_rad": "position/lat-geod-rad", "lon_rad": "position/long-gc-rad",
         "alt_ft": "position/h-sl-ft", "alt_cmd_ft": "ap/alt-cmd-ft", "phi_deg": "attitude/phi-deg",
         "vt_fps": "velocities/vt-fps"}

# --- path-following mission ------------------------------------------------------
fdm = setup()
wm = WaypointManager(WPS, loiter_radius_m=150.0)
df = run(fdm, 160.0 if args.fast else 260.0, PROPS, callback=wm, record_every=12)
LAT0R, LON0R = math.radians(LAT0), math.radians(LON0)


def add_ne(d):
    d["north_m"], d["east_m"] = ne_from_latlon(d.lat_rad, d.lon_rad, LAT0R, LON0R)
    return d


add_ne(df)
leg = np.array(wm.log["leg"])
xt = np.array(wm.log["xtrack_m"])
tt = np.array(wm.log["t"])
for i in range(1, len(WPS)):
    sel = (leg == i) & (tt > tt[np.argmax(leg == i)] + 15) if np.any(leg == i) else np.zeros_like(leg, bool)
    if sel.any():
        print(f"leg {i} ({WPS[i - 1][:2]} -> {WPS[i][:2]}): steady |cross-track| mean {np.mean(np.abs(xt[sel])):.1f} m, "
              f"max {np.max(np.abs(xt[sel])):.1f} m")
print(f"mission altitude tracking: max |alt - cmd| after captures "
      f"{np.max(np.abs(df.alt_ft - df.alt_cmd_ft)[df.index > 30]):.0f} ft (during climbs/descents the error is the ramp)")

# --- direct-to (XML) vs path following, starting 150 m OFF the leg ----------------------
# The intended leg runs north along east = -150 m; the airplane starts at the origin.
LEG = [(0, -150, 330), (800, -150, 330)]
fdm = setup()
R_EARTH = 6378137.0
fdm["guidance/target-lat-rad"] = math.radians(LAT0) + LEG[1][0] / R_EARTH
fdm["guidance/target-lon-rad"] = math.radians(LON0) + LEG[1][1] / (R_EARTH * math.cos(math.radians(LAT0)))
fdm["ap/guidance-on"] = 1
d2 = add_ne(run(fdm, 30.0, {**PROPS, "dist_m": "guidance/wp-distance-m"}, record_every=12))
fdm2 = setup()
wm2 = WaypointManager(LEG, loiter_radius_m=150.0)
p2 = add_ne(run(fdm2, 30.0, PROPS, callback=wm2, record_every=12))
for name, d in (("direct-to", d2), ("path following", p2)):
    on = d[(d.north_m > 300) & (d.north_m < 750)]
    print(f"{name:15s}: |cross-track from the leg| for north 300..750 m: mean {np.mean(np.abs(on.east_m + 150)):6.1f} m")
print("Direct-to flies a NEW straight line from wherever it is to the waypoint - it never returns to the leg.")

fig, ax = plt.subplots(1, 2, figsize=(13, 6))
wp = np.array(WPS)
ax[0].plot(wp[:, 1], wp[:, 0], "ks--", lw=0.8, label="waypoints")
ax[0].plot(df.east_m, df.north_m, label="flown (path following)")
ax[0].set(xlabel="east [m]", ylabel="north [m]", title=f"mission in {WIND_E:.0f} m/s wind from the west")
ax[0].axis("equal")
ax[1].plot([-150, -150], [0, 800], "k--", lw=0.8, label="intended leg")
ax[1].plot(d2.east_m, d2.north_m, label="direct-to (XML waypoint_heading)")
ax[1].plot(p2.east_m, p2.north_m, label="path following (vector field)")
ax[1].set(xlabel="east [m]", ylabel="north [m]", title="starting 150 m off the leg: direct-to vs path following")
ax[1].axis("equal")
for a_ in ax:
    a_.grid(alpha=0.3)
    a_.legend(fontsize="small")
fig.tight_layout()
save(fig, "12_guidance", "waypoint_mission", show=args.show)
fig, a_ = plt.subplots(figsize=(10, 3.5))
a_.plot(df.index, df.alt_ft, label="altitude")
a_.plot(df.index, df.alt_cmd_ft, "--", label="command")
a_.set(xlabel="time [s]", ylabel="ft", title="mission altitude profile")
a_.grid(alpha=0.3)
a_.legend()
save(fig, "12_guidance", "mission_altitude", show=args.show)
