"""Module 11 - validate the Python autopilot against the XML one.

Before using a re-implementation of a control law for anything (SIL, flight
software, trade studies), show it reproduces the reference.  Same gains, same
120 Hz frame, same 100 ft altitude step: the two traces must lie on top of
each other.  (In a team this is a regression test that runs on every commit.)

    python modules/11_robustness/solutions/python_vs_xml.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.autopilot import engage_lateral, engage_longitudinal  # noqa: E402
from gnclab.controllers import LongitudinalAP  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402
from gnclab.trim import trim  # noqa: E402

PROPS = {"alt_ft": "position/h-sl-ft", "theta_deg": "attitude/theta-deg", "vt_fps": "velocities/vt-fps",
         "elevator_rad": "fcs/elevator-pos-rad", "throttle": "fcs/throttle-total-norm"}
T = 30.0 if args.fast else 45.0


def trimmed():
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048})
    trim(fdm, "full")
    return fdm


# XML autopilot
fdm = trimmed()
engage_longitudinal(fdm)
engage_lateral(fdm, course=False)
h0 = fdm["position/h-sl-ft"]
xml = run(fdm, T, PROPS, callback=lambda f, t: f.__setitem__("ap/alt-cmd-ft", h0 + 100) if t >= 2 else None)

# Python autopilot (XML longitudinal loops off; XML roll loop holds wings level)
fdm = trimmed()
ap = LongitudinalAP(fdm, rate_hz=120.0)


def py_cb(f, t):
    if t >= 2:
        ap.alt_cmd = h0 + 100
    ap(f, t)


py = run(fdm, T, PROPS, callback=py_cb)
for col in PROPS:
    err = np.max(np.abs(xml[col].to_numpy() - py[col].to_numpy()))
    rng = np.ptp(xml[col].to_numpy())
    print(f"{col:13s} max |XML - Python| = {err:.3g}  ({100 * err / max(rng, 1e-9):.2f} % of its range)")
fig, _ = timehistory({"XML autopilot": xml, "Python autopilot": py}, list(PROPS),
                     title="Python re-implementation vs XML autopilot (120 Hz)")
save(fig, "11_robustness", "python_vs_xml", show=args.show)
