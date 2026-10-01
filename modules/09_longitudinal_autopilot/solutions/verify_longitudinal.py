"""Module 09 - fly the XML longitudinal autopilot and check it against requirements.

Requirements (we'll reuse them for the Monte Carlo in Module 15):
  R-L1 altitude step 100 ft : overshoot <= 10 %, settles within +/-10 ft in <= 30 s, |error| < 2 ft at end
  R-L2 airspeed step +10 ft/s: overshoot <= 20 %, settles within +/-1.5 ft/s in <= 20 s
  R-L3 coupling: during the airspeed step, altitude stays within +/-15 ft
  R-L4 elevator activity: elevator never saturates (|ap cmd| < 1) during R-L1..R-L3
Each test is flown twice: ideal (truth feedback, ideal surfaces) and realistic
(actuators + sensor noise/lag/delay on).

    python modules/09_longitudinal_autopilot/solutions/verify_longitudinal.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.autopilot import engage_longitudinal, realism  # noqa: E402
from gnclab.metrics import step_metrics  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402
from gnclab.trim import trim  # noqa: E402

PROPS = {"alt_ft": "position/h-sl-ft", "alt_cmd_ft": "ap/alt-cmd-ft", "vt_fps": "velocities/vt-fps",
         "vt_cmd_fps": "ap/vt-cmd-fps", "theta_deg": "attitude/theta-deg", "theta_cmd_deg": "ap/theta-cmd-rad",
         "de_ap": "ap/elevator-cmd-norm", "throttle_total": "fcs/throttle-total-norm",
         "alt_trigger": "ap/alt-int-trigger", "phi_deg": "attitude/phi-deg"}
T = 40.0 if args.fast else 60.0


def flight(realistic: bool, d_alt: float = 0.0, d_vt: float = 0.0):
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048})
    trim(fdm, "full")
    realism(fdm, realistic)
    engage_longitudinal(fdm)
    h0, v0 = fdm["position/h-sl-ft"], fdm["velocities/vt-fps"]
    da0 = fdm["fcs/aileron-cmd-norm"]

    def step_cmds(f, t):
        if t >= 2.0:
            f["ap/alt-cmd-ft"] = h0 + d_alt
            f["ap/vt-cmd-fps"] = v0 + d_vt
        # gnc_trainer is SPIRAL UNSTABLE: without lateral control it rolls off and
        # no pitch loop can hold altitude in a steep bank.  Module 10 builds the
        # real roll autopilot; until then, a minimal Python wing leveler:
        f["fcs/aileron-cmd-norm"] = da0 - 1.5 * f["attitude/phi-rad"] - 0.3 * f["velocities/p-rad_sec"]

    df = run(fdm, T, PROPS, callback=step_cmds, record_every=6)
    df["theta_cmd_deg"] = np.degrees(df["theta_cmd_deg"])
    return df, h0, v0


rows, runs = [], {}
for realistic in (False, True):
    tag = "realistic" if realistic else "ideal"
    a, h0, v0 = flight(realistic, d_alt=100.0)
    s, _, _ = flight(realistic, d_vt=10.0)
    runs[f"alt step, {tag}"], runs[f"speed step, {tag}"] = a, s
    ma = step_metrics(a.index, a.alt_ft, h0 + 100, h0, t_step=2.0, band=0.10)
    ms = step_metrics(s.index, s.vt_fps, v0 + 10, v0, t_step=2.0, band=0.15)
    coupling = (s.alt_ft - h0).abs().max()
    max_de = max(a.de_ap.abs().max(), s.de_ap.abs().max())
    checks = {
        "R-L1 alt overshoot <= 10 %": ma.overshoot_pct <= 10,
        "R-L1 alt settle (10 ft) <= 30 s": ma.settling_time <= 30,
        "R-L1 alt final error < 2 ft": abs(ma.steady_state_error) < 2,
        "R-L2 speed overshoot <= 20 %": ms.overshoot_pct <= 20,
        "R-L2 speed settle (1.5 ft/s) <= 20 s": ms.settling_time <= 20,
        "R-L3 alt excursion in speed step <= 15 ft": coupling <= 15,
        "R-L4 elevator cmd not saturated": max_de < 0.99,
    }
    print(f"\n[{tag}] altitude step: overshoot {ma.overshoot_pct:.1f} %, settle {ma.settling_time:.1f} s, "
          f"final err {ma.steady_state_error:+.2f} ft | speed step: overshoot {ms.overshoot_pct:.1f} %, "
          f"settle {ms.settling_time:.1f} s | coupling {coupling:.1f} ft | max |de_ap| {max_de:.2f}")
    for k, ok in checks.items():
        print(f"   {'PASS' if ok else 'FAIL'}  {k}")

fig, _ = timehistory({k: v for k, v in runs.items() if k.startswith("alt")},
                     ["alt_ft", "theta_deg", "theta_cmd_deg", "de_ap", "throttle_total", "vt_fps"],
                     title="100 ft altitude step (altitude + airspeed hold)")
save(fig, "09_longitudinal_autopilot", "alt_step", show=args.show)
fig, _ = timehistory({k: v for k, v in runs.items() if k.startswith("speed")},
                     ["vt_fps", "throttle_total", "alt_ft", "theta_deg", "de_ap"],
                     title="+10 ft/s airspeed step (altitude + airspeed hold)")
save(fig, "09_longitudinal_autopilot", "speed_step", show=args.show)
