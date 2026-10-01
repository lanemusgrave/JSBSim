"""Module 10 - verify the lateral autopilot (with the Module 09 longitudinal loops on).

Requirements:
  R-A1 roll step 20 deg (course hold off): rise (10-90 %) <= 1.0 s, overshoot <= 15 %
  R-A2 course change 90 deg: overshoot <= 5 deg, within +/-2 deg of command by 20 s
  R-A3 |beta| <= 3 deg throughout the course change
  R-A4 altitude within +/-30 ft during the course change
  R-A5 wrap-around: commanding 350 deg from 0 deg turns LEFT (never past +20 deg)
  R-A6 yaw damper: after a rudder doublet, Dutch-roll beta decays to < 0.2 deg in 4 s
Flown ideal and realistic (actuators + sensors).

    python modules/10_lateral_autopilot/solutions/verify_lateral.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.autopilot import engage_lateral, engage_longitudinal, realism  # noqa: E402
from gnclab.metrics import step_metrics  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402
from gnclab.signals import doublet  # noqa: E402
from gnclab.trim import trim  # noqa: E402

PROPS = {"chi_deg": "flight-path/psi-gt-rad", "phi_deg": "attitude/phi-rad", "phi_cmd_deg": "ap/phi-cmd-rad",
         "beta_deg": "aero/beta-deg", "alt_ft": "position/h-sl-ft", "da_ap": "ap/aileron-cmd-norm",
         "dr_ap": "ap/rudder-cmd-norm", "north_ft": "position/distance-from-start-lat-mt",
         "east_ft": "position/distance-from-start-lon-mt"}


def flight(realistic, course=True, yaw_damper=True, cb=None, T=40.0):
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": 33.7, "long-gc-deg": -117.9, "h-sl-ft": 330,
                     "vt-fps": 25 / 0.3048, "psi-true-deg": 0.0})
    trim(fdm, "full")
    realism(fdm, realistic)
    engage_longitudinal(fdm)
    engage_lateral(fdm, course=course, yaw_damper=yaw_damper)
    df = run(fdm, T, PROPS, callback=cb, record_every=6)
    # course is reported in [0, 2 pi): unwrap it (and start it near 0) before measuring anything
    df["chi_deg"] = np.unwrap(df["chi_deg"].to_numpy())
    df["chi_deg"] -= 2 * np.pi * np.round(df["chi_deg"].iloc[0] / (2 * np.pi))
    for c in ("chi_deg", "phi_deg", "phi_cmd_deg"):
        df[c] = np.degrees(df[c])
    return df, fdm


def at(t0, fn):
    return lambda f, t: fn(f) if t >= t0 else None


runs = {}
for realistic in (False, True):
    tag = "realistic" if realistic else "ideal"
    # R-A1 roll step
    r, _ = flight(realistic, course=False, cb=at(2.0, lambda f: f.__setitem__("ap/phi-cmd-ext-rad", np.radians(20))), T=10)
    mr = step_metrics(r.index, r.phi_deg, 20.0, r.phi_deg.iloc[0], t_step=2.0, band=0.05)
    # R-A2..4 course change
    c, _ = flight(realistic, cb=at(2.0, lambda f: f.__setitem__("ap/chi-cmd-rad", np.radians(90))))
    alt0 = c.alt_ft.iloc[0]
    over = c.chi_deg.max() - 90
    late = c.loc[22.0:]
    # R-A5 wrap
    w, _ = flight(realistic, cb=at(2.0, lambda f: f.__setitem__("ap/chi-cmd-rad", np.radians(350))), T=25)
    wrap_chi = w.chi_deg                                     # unwrapped: a left turn goes negative
    # R-A6 yaw damper on/off, rudder doublet
    def kick(f, t):
        f["fcs/rudder-cmd-norm"] = doublet(t, 2.0, 0.5, 0.4)
    dr_on, _ = flight(realistic, cb=kick, T=10)
    dr_off, _ = flight(realistic, yaw_damper=False, cb=kick, T=10)
    beta_after_on = dr_on.loc[7.0:, "beta_deg"].abs().max()
    beta_after_off = dr_off.loc[7.0:, "beta_deg"].abs().max()
    runs[tag] = (c, w, dr_on, dr_off)
    checks = {
        f"R-A1 roll rise {mr.rise_time:.2f} s <= 1.0 s": mr.rise_time <= 1.0,
        f"R-A1 roll overshoot {mr.overshoot_pct:.1f} % <= 15 %": mr.overshoot_pct <= 15,
        f"R-A2 course overshoot {over:.2f} deg <= 5 deg": over <= 5,
        f"R-A2 within 2 deg after 20 s (max err {np.abs(late.chi_deg - 90).max():.2f})": np.abs(late.chi_deg - 90).max() <= 2,
        f"R-A3 max |beta| {c.beta_deg.abs().max():.2f} deg <= 3": c.beta_deg.abs().max() <= 3,
        f"R-A4 altitude excursion {np.abs(c.alt_ft - alt0).max():.1f} ft <= 30": np.abs(c.alt_ft - alt0).max() <= 30,
        f"R-A5 wrap: course went {wrap_chi.min():.1f}..{wrap_chi.max():.1f} deg (left turn to -10)": wrap_chi.max() <= 20 and abs(wrap_chi.iloc[-1] + 10) < 2,
        f"R-A6 beta after doublet {beta_after_on:.3f} deg (damper off: {beta_after_off:.3f})": beta_after_on < 0.2,
    }
    print(f"\n[{tag}]")
    for k, ok in checks.items():
        print(f"   {'PASS' if ok else 'FAIL'}  {k}")

c, w, dr_on, dr_off = runs["realistic"]
fig, _ = timehistory({"ideal": runs["ideal"][0], "realistic": c},
                     ["chi_deg", "phi_deg", "phi_cmd_deg", "beta_deg", "alt_ft", "da_ap", "dr_ap"],
                     title="90 deg course change (full autopilot)")
save(fig, "10_lateral_autopilot", "course_change", show=args.show)
fig, _ = timehistory({"yaw damper ON": dr_on, "yaw damper OFF": dr_off}, ["beta_deg", "dr_ap", "phi_deg"],
                     title="Rudder doublet: yaw damper on vs off (realistic)")
save(fig, "10_lateral_autopilot", "yaw_damper", show=args.show)
fig, ax = plt.subplots(figsize=(6, 6))
for name, df in (("90 deg change", c), ("0 -> 350 deg (wrap test)", w)):
    ax.plot(df.east_ft, df.north_ft, label=name)
ax.set(xlabel="east [m]", ylabel="north [m]", title="ground tracks (start at origin, heading north)")
ax.axis("equal")
ax.grid(alpha=0.3)
ax.legend()
save(fig, "10_lateral_autopilot", "ground_tracks", show=args.show)
