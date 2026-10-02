"""Module 04 exercise: add flaps to your own copy of the glider.

Setup (one time):
  1. Copy the folder aircraft/m04_glider to aircraft/my_glider, and rename the
     file inside to my_glider.xml.  Change <fdm_config name="..."> to my_glider.
  2. In my_glider.xml:
     TODO A: add a "Flaps" <channel> to <flight_control>: an <actuator> taking
             fcs/flap-cmd-norm (0..1), rate limit 0.5 /s, clipped 0..1, output
             fcs/flap-pos-norm.  (Optionally a pure_gain to fcs/flap-pos-deg.)
     TODO B: add flap terms to the aero model:
               LIFT  axis: delta CL = +0.50 * flap-pos-norm
               DRAG  axis: delta CD = +0.025 * flap-pos-norm
               PITCH axis: delta Cm = -0.02 * flap-pos-norm  (remember the cbar!)
  3. Run this script.  It glides the aircraft with flaps up and down and
     prints the steady glide speed, glide angle and alpha.

Then answer in a comment at the bottom:
  Q1: why does the glide speed drop with flaps, and why does alpha go DOWN?
  Q2: change Clbeta in YOUR copy from -0.10 to -0.06, glide 300 s with no
      flaps, and plot phi.  What happens and why?  (Spiral criterion:
      Clbeta*Cnr > Cnbeta*Clr for a stable spiral.)

    python modules/04_aircraft_anatomy/exercises/glider_ex.py [aircraft_name]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__, extra=lambda p: p.add_argument("aircraft", nargs="?", default="my_glider"))

from gnclab import AIRCRAFT_DIR, initialize, make_fdm, run  # noqa: E402

if not (AIRCRAFT_DIR / args.aircraft).is_dir():
    raise SystemExit(f"aircraft/{args.aircraft}/ not found - do setup step 1 first "
                     "(or run with m04_glider_flaps to see the solution)")

for flap in (0.0, 1.0):
    fdm = make_fdm(args.aircraft)
    initialize(fdm, {"lat-geod-deg": 33, "h-sl-ft": 3000, "vt-fps": 45.0}, engines_running=False)
    fdm["fcs/flap-cmd-norm"] = flap
    df = run(fdm, 120.0 if args.fast else 300.0,
             {"V": "velocities/vt-fps", "gamma": "flight-path/gamma-deg", "alpha": "aero/alpha-deg"},
             record_every=12)
    tail = df.iloc[int(0.6 * len(df)):]
    print(f"flaps {flap:.0f}: V = {tail.V.mean() * 0.3048:5.2f} m/s, gamma = {tail.gamma.mean():5.2f} deg, "
          f"alpha = {tail.alpha.mean():5.2f} deg")

# Q1:
# Q2:
