"""Module 12 exercise A - solution: a fillet path manager (B&M Algorithm 6)."""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.autopilot import engage_lateral, engage_longitudinal, realism  # noqa: E402
from gnclab.guidance import FilletManager, WaypointManager, ne_from_latlon  # noqa: E402
# FilletManager lives in gnclab.guidance (the capstone uses it): read it there.
from gnclab.plotting import save  # noqa: E402
from gnclab.trim import trim  # noqa: E402

FPS = 1 / 0.3048
WPS = [(0, 0, 330), (800, 0, 330), (800, 800, 330), (0, 800, 330), (0, 1600, 330)]


def fly(manager):
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": 33.7, "long-gc-deg": -117.9, "h-sl-ft": 330, "vt-fps": 25 * FPS,
                     "psi-true-deg": 0.0})
    trim(fdm, "full")
    realism(fdm, True)
    engage_longitudinal(fdm)
    engage_lateral(fdm)
    df = run(fdm, 150.0 if args.fast else 190.0,
             {"lat": "position/lat-geod-rad", "lon": "position/long-gc-rad"}, callback=manager, record_every=12)
    df["n"], df["e"] = ne_from_latlon(df.lat, df.lon, fdm["ic/lat-geod-rad"], fdm["ic/long-gc-rad"])
    return df


def max_path_deviation(df):
    """Largest distance from the waypoint polyline, before the final loiter starts."""
    wp = np.array([w[:2] for w in WPS], float)
    pts = df[["n", "e"]].to_numpy()
    pts = pts[: np.argmax(pts[:, 1] > WPS[-1][1] - 150) or len(pts)]   # stop before the loiter region
    best = np.full(len(pts), np.inf)
    for a, b in zip(wp[:-1], wp[1:]):
        ab = b - a
        tt = np.clip(((pts - a) @ ab) / (ab @ ab), 0, 1)
        best = np.minimum(best, np.linalg.norm(pts - (a + tt[:, None] * ab), axis=1))
    return best.max()


plain = fly(WaypointManager(WPS, loiter_radius_m=150.0))
fil = fly(FilletManager(WPS, loiter_radius_m=150.0))
print(f"max deviation from the waypoint path: switching at waypoints {max_path_deviation(plain):6.1f} m, "
      f"fillets {max_path_deviation(fil):6.1f} m")

import matplotlib.pyplot as plt  # noqa: E402

fig, ax = plt.subplots(figsize=(7, 7))
wp = np.array(WPS)
ax.plot(wp[:, 1], wp[:, 0], "ks--", lw=0.8, label="waypoints")
ax.plot(plain.e, plain.n, label="switch at waypoint")
ax.plot(fil.e, fil.n, label="fillets (R = 120 m)")
ax.set(xlabel="east [m]", ylabel="north [m]", title="Waypoint switching vs fillets")
ax.axis("equal")
ax.grid(alpha=0.3)
ax.legend()
save(fig, "12_guidance", "fillets", show=args.show)
