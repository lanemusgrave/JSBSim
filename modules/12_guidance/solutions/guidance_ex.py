"""Module 12 exercise A - solution: a fillet path manager (B&M Algorithm 6)."""

import math

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.autopilot import engage_lateral, engage_longitudinal, realism  # noqa: E402
from gnclab.guidance import WaypointManager, line_course, ne_from_latlon, ne_position, orbit_course  # noqa: E402
from gnclab.plotting import save  # noqa: E402
from gnclab.trim import trim  # noqa: E402

FPS = 1 / 0.3048
WPS = [(0, 0, 330), (800, 0, 330), (800, 800, 330), (0, 800, 330), (0, 1600, 330)]


class FilletManager(WaypointManager):
    """Straight legs joined by orbit segments of radius R (B&M Algorithm 6)."""

    R: float = 120.0

    def _geom(self, i):
        w = [np.array(x[:2], float) for x in self.waypoints]
        q0 = (w[i] - w[i - 1]) / np.linalg.norm(w[i] - w[i - 1])
        q1 = (w[i + 1] - w[i]) / np.linalg.norm(w[i + 1] - w[i])
        rho = math.acos(np.clip(-q0 @ q1, -1, 1))
        z1 = w[i] - self.R / math.tan(rho / 2) * q0
        z2 = w[i] + self.R / math.tan(rho / 2) * q1
        c = w[i] - self.R / math.sin(rho / 2) * (q0 - q1) / np.linalg.norm(q0 - q1)
        lam = 1 if q0[0] * q1[1] - q0[1] * q1[0] > 0 else -1
        return w, q0, q1, z1, z2, c, lam

    def __call__(self, fdm, t):
        if t + 1e-9 < self._t_next:
            return
        self._t_next += 1.0 / self.rate_hz
        p = ne_position(fdm)
        chi = fdm["flight-path/psi-gt-rad"]
        last = self.i + 1 >= len(self.waypoints)
        if last:   # final leg: plain line to the last waypoint, then loiter
            w = [np.array(x[:2], float) for x in self.waypoints]
            q = (w[-1] - w[-2]) / np.linalg.norm(w[-1] - w[-2])
            if self.mode != "orbit" and np.dot(p - w[-1], q) < 0:
                chi_c, err = line_course(p, chi, w[-2], q, self.chi_inf, self.k_path)
            else:
                self.mode = "orbit"
                chi_c, err = orbit_course(p, chi, w[-1], self.loiter_radius_m, 1, self.k_orbit)
        else:
            w, q0, q1, z1, z2, c, lam = self._geom(self.i)
            if self.mode == "line":
                if np.dot(p - z1, q0) >= 0:
                    self.mode = "fillet"
                chi_c, err = line_course(p, chi, w[self.i - 1], q0, self.chi_inf, self.k_path)
            if self.mode == "fillet":
                if np.dot(p - z2, q1) >= 0:
                    self.mode = "line"
                    self.i += 1
                    chi_c, err = line_course(p, chi, w[self.i - 1], q1, self.chi_inf, self.k_path)
                else:
                    chi_c, err = orbit_course(p, chi, c, self.R, lam, self.k_orbit)
        fdm["ap/chi-cmd-rad"] = chi_c
        fdm["ap/alt-cmd-ft"] = self.waypoints[min(self.i, len(self.waypoints) - 1)][2]
        self.log["t"].append(t)
        self.log["leg"].append(self.i)
        self.log["xtrack_m"].append(err)
        self.log["chi_c"].append(chi_c)


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
