"""Module 00 exercise: get comfortable running a script and reading its data.

Run:  python modules/00_setup/exercises/first_flight_ex.py --show

TODO 1: add pitch attitude ("attitude/theta-deg") and vertical speed
        ("velocities/h-dot-fps") to the recorded properties.
TODO 2: find the lift-off time: the first time altitude AGL exceeds 10 ft.
TODO 3: compute the average climb rate in ft/min between t = 60 s and t = 120 s.
        (Sanity check against the POH: a C172 at full power climbs ~700 fpm
        near sea level.  Is the autopilot getting that?  Why not?)
TODO 4: look at the airspeed trace.  The altitude-hold autopilot uses the
        elevator and the throttle is fixed at full - so what is controlling
        airspeed?  Write your answer as a comment at the bottom.
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

from gnclab import load_script, run  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402

fdm = load_script("scripts/c1723.xml")

props = {
    "airspeed_kcas": "velocities/vc-kts",
    "alt_agl_ft": "position/h-agl-ft",
    "heading_deg": "attitude/psi-deg",
    "pitch_attitude" : "attitude/theta-deg",
    "vertical_speed" : "velocities/h-dot-fps",

    # TODO 1: add theta and h-dot here
}
df = run(fdm, 60.0 if args.fast else 200.0, props, record_every=12)

# TODO 2: lift-off time.  Hint: df.index[df.alt_agl_ft > 10][0]
liftoff_time = df.index[df.alt_agl_ft > 10][0]
print("lift-off at", liftoff_time)

# TODO 3: average climb rate [ft/min] between 60 and 120 s
window = df.loc[60:120]
climb_fpm = (window.alt_agl_ft.iloc[-1] - window.alt_agl_ft.iloc[0]) / (window.index[-1] - window.index[0]) * 60.0
print("average climb rate", climb_fpm)

fig, _ = timehistory(df, list(props), title="C172 takeoff - exercise")
save(fig, "00_setup", "first_flight_ex", show=args.show)

# TODO 4: Airspeed is being controlled by pitch/elevator because altitude hold is on. the oscillation
# is due