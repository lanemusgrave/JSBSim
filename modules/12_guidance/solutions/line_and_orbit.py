"""Module 12 - vector-field path following: a straight line and an orbit, with wind.

The airplane starts 150 m left of a north-going line (heading north-east), then
captures it.  Second run: start 400 m from an orbit centre and capture a
150 m-radius clockwise orbit.  Each is flown in calm air and in a 6 m/s
crosswind (wind FROM the west), with the full autopilot (Modules 09-10) on and
realistic sensors/actuators.

    python modules/12_guidance/solutions/line_and_orbit.py [--show]
"""

import math

from gnclab.cli import parse_args

args = parse_args(__doc__)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.autopilot import engage_lateral, engage_longitudinal, realism  # noqa: E402
from gnclab.guidance import line_course, ne_position, orbit_course  # noqa: E402
from gnclab.plotting import save  # noqa: E402
from gnclab.trim import trim  # noqa: E402

FPS = 1 / 0.3048


def flight(guidance, wind_east_mps=0.0, T=80.0, psi0=45.0):
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": 33.7, "long-gc-deg": -117.9, "h-sl-ft": 330,
                     "vt-fps": 25 * FPS, "psi-true-deg": psi0})
    trim(fdm, "full")
    realism(fdm, True)
    engage_longitudinal(fdm)
    engage_lateral(fdm)
    fdm["atmosphere/wind-east-fps"] = wind_east_mps * FPS   # wind blowing toward the east (from the west)
    log = {"t": [], "n": [], "e": [], "err": [], "psi": [], "chi": []}
    state = {"next": 0.0}

    def cb(f, t):
        if t >= state["next"]:                               # 10 Hz guidance
            state["next"] += 0.1
            p = ne_position(f)
            chi_c, err = guidance(p, f["flight-path/psi-gt-rad"])
            f["ap/chi-cmd-rad"] = chi_c
            log["t"].append(t); log["n"].append(p[0]); log["e"].append(p[1]); log["err"].append(err)
            log["psi"].append(math.degrees(f["attitude/psi-rad"])); log["chi"].append(math.degrees(f["flight-path/psi-gt-rad"]))

    run(fdm, T, {"h": "position/h-sl-ft"}, callback=cb, record_every=60)
    return {k: np.array(v) for k, v in log.items()}


R0, Q = np.array([0.0, 150.0]), np.array([1.0, 0.0])        # line through (0, 150 m east) going north
CENTER, RHO = np.array([300.0, 300.0]), 150.0


def line(p, chi):
    return line_course(p, chi, R0, Q, chi_inf=math.radians(60), k_path=0.02)


def orbit(p, chi):
    return orbit_course(p, chi, CENTER, RHO, lam=1, k_orbit=2.0)


fig, ax = plt.subplots(1, 2, figsize=(12, 6))
for wind in (0.0, 6.0):
    L = flight(line, wind, T=40.0 if args.fast else 60.0)
    O = flight(orbit, wind, T=60.0 if args.fast else 100.0)
    late_L = L["t"] > L["t"][-1] - 20
    late_O = O["t"] > O["t"][-1] - 40
    crab = np.mean(np.abs(((L["psi"] - L["chi"])[late_L] + 180) % 360 - 180))
    print(f"wind {wind:3.1f} m/s: line   |cross-track| last 20 s: mean {np.mean(np.abs(L['err'][late_L])):5.2f} m, "
          f"max {np.max(np.abs(L['err'][late_L])):5.2f} m; crab angle {crab:4.1f} deg")
    print(f"               orbit  |radial error| last 40 s: mean {np.mean(np.abs(O['err'][late_O])):5.2f} m, "
          f"max {np.max(np.abs(O['err'][late_O])):5.2f} m")
    ax[0].plot(L["e"], L["n"], label=f"wind {wind:.0f} m/s")
    ax[1].plot(O["e"], O["n"], label=f"wind {wind:.0f} m/s")
ax[0].axvline(R0[1], color="k", ls="--", lw=0.8, label="path")
th = np.linspace(0, 2 * np.pi, 200)
ax[1].plot(CENTER[1] + RHO * np.sin(th), CENTER[0] + RHO * np.cos(th), "k--", lw=0.8, label="orbit")
for a_, title in zip(ax, ("straight-line following", "orbit following (CW)")):
    a_.set(xlabel="east [m]", ylabel="north [m]", title=title)
    a_.axis("equal")
    a_.grid(alpha=0.3)
    a_.legend(fontsize="small")
fig.tight_layout()
save(fig, "12_guidance", "line_and_orbit", show=args.show)
