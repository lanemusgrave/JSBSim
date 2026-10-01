"""Module 00 - first flight: run the bundled C172 takeoff-and-climb script.

The script (scripts/c1723.xml, bundled with JSBSim) starts a Cessna 172 on the
runway, releases the brakes, rotates at 51 kt, engages the altitude-hold
autopilot and then a heading-hold autopilot.  We let JSBSim fly it and record a
few properties at 10 Hz.

    python modules/00_setup/solutions/first_flight.py [--show] [--fast]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

from gnclab import load_script, run  # noqa: E402  (import after backend choice)
from gnclab.plotting import save, timehistory  # noqa: E402

fdm = load_script("scripts/c1723.xml")

props = {
    "airspeed_kcas": "velocities/vc-kts",
    "alt_agl_ft": "position/h-agl-ft",
    "heading_deg": "attitude/psi-deg",
    "roll_deg": "attitude/phi-deg",
    "throttle": "fcs/throttle-pos-norm[0]",
    "ap_alt_hold": "ap/altitude_hold",
}
duration = 60.0 if args.fast else 200.0
# record every 12th step: dt = 1/120 s -> 10 Hz logging
df = run(fdm, duration, props, record_every=12)

print(df.iloc[:: max(1, len(df) // 10)].round(2).to_string())
print(f"\nFinal: {df.airspeed_kcas.iloc[-1]:.1f} KCAS, {df.alt_agl_ft.iloc[-1]:.0f} ft AGL, "
      f"heading {df.heading_deg.iloc[-1]:.1f} deg")

fig, _ = timehistory(df, list(props), title="C172 scripted takeoff and climb (c1723.xml)")
save(fig, "00_setup", "first_flight", show=args.show)
