"""Module 00 exercise - solution."""

from gnclab.cli import parse_args

args = parse_args(__doc__)

from gnclab import load_script, run  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402

fdm = load_script("scripts/c1723.xml")

props = {
    "airspeed_kcas": "velocities/vc-kts",
    "alt_agl_ft": "position/h-agl-ft",
    "heading_deg": "attitude/psi-deg",
    "theta_deg": "attitude/theta-deg",
    "hdot_fps": "velocities/h-dot-fps",
}
df = run(fdm, 130.0 if args.fast else 200.0, props, record_every=12)

liftoff_time = df.index[df.alt_agl_ft > 10][0]
print(f"lift-off at {liftoff_time:.1f} s")

window = df.loc[60:120]
climb_fpm = (window.alt_agl_ft.iloc[-1] - window.alt_agl_ft.iloc[0]) / (
    window.index[-1] - window.index[0]) * 60.0
print(f"average climb rate 60-120 s: {climb_fpm:.0f} ft/min "
      f"(mean h-dot {window.hdot_fps.mean() * 60:.0f} ft/min)")

fig, _ = timehistory(df, list(props), title="C172 takeoff - exercise solution")
save(fig, "00_setup", "first_flight_ex", show=args.show)

# TODO 4 answer: nothing controls airspeed directly.  With throttle fixed at
# full, the elevator (driven by altitude hold) sets the flight path and the
# airspeed is whatever is left over from the energy balance: thrust - drag =
# W*sin(gamma) + (W/g)*dV/dt.  Every time the autopilot pitches up to fix an
# altitude error it trades airspeed for climb, and the lightly damped
# speed/altitude exchange you see is the phugoid poking through.  This
# "elevator for path, throttle for speed" coupling comes back in Module 09
# (and TECS in Module 12).  The ~530 ft/min climb is below the book ~700
# because the autopilot is flying ~50 KCAS, slower than Vy (~74 KIAS).
