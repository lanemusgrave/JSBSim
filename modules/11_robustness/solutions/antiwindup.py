"""Module 11 - integrator windup, and why every PI in the autopilot has a trigger.

A 400 ft altitude step saturates the pitch command (+/-15 deg) for a long
time.  Without anti-windup the altitude integrator keeps integrating the big
error, then has to "unwind" after the capture - a large overshoot.

    python modules/11_robustness/solutions/antiwindup.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.controllers import LongitudinalAP  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402
from gnclab.trim import trim  # noqa: E402

runs = {}
for aw in (True, False):
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048})
    trim(fdm, "full")
    ap = LongitudinalAP(fdm, anti_windup=aw)
    h0 = fdm["position/h-sl-ft"]

    log = {"t": [], "int_h": []}

    def cb2(f, t, ap=ap, log=log):
        if t >= 2:
            ap.alt_cmd = h0 + 400
        ap(f, t)
        log["t"].append(t)
        log["int_h"].append(ap.int_h)

    df = run(fdm, 60.0 if args.fast else 90.0, {"alt_ft": "position/h-sl-ft", "theta_deg": "attitude/theta-deg",
                                                "vt_fps": "velocities/vt-fps"}, callback=cb2)
    df["integrator_ft_s"] = np.interp(df.index, log["t"], log["int_h"])
    over = df.alt_ft.max() - (h0 + 400)
    t_cap = df.index[np.argmax(np.abs(df.alt_ft - (h0 + 400)) < 10)]
    print(f"anti-windup {'ON ' if aw else 'OFF'}: overshoot {over:6.1f} ft, first within 10 ft at {t_cap:5.1f} s, "
          f"peak integrator {df.integrator_ft_s.abs().max():8.1f} ft*s")
    runs[f"anti-windup {'ON' if aw else 'OFF'}"] = df
fig, _ = timehistory(runs, ["alt_ft", "theta_deg", "integrator_ft_s", "vt_fps"], title="400 ft climb: integrator windup")
save(fig, "11_robustness", "antiwindup", show=args.show)
