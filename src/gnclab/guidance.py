"""Path-following guidance (Beard & McLain ch. 10-11) for the gnc_trainer autopilot.

Positions are local North/East in metres relative to the initial-condition
point, computed from latitude/longitude (see :func:`ne_from_latlon`).  Guidance runs as an
outer loop (default 10 Hz) and writes the autopilot's course and altitude
commands (``ap/chi-cmd-rad``, ``ap/alt-cmd-ft``).

* :func:`line_course`  - vector field for a straight line
* :func:`orbit_course` - vector field for a circular orbit
* :class:`WaypointManager` - fly a list of waypoints, switching legs at the
  half-plane through each waypoint (B&M Algorithm 5), optionally ending in an orbit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

M2FT = 1.0 / 0.3048


def wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


R_EARTH_M = 6378137.0


def ne_from_latlon(lat_rad, lon_rad, lat0_rad, lon0_rad):
    """Flat-Earth local north/east [m] of (lat, lon) relative to (lat0, lon0).

    Good to well under 0.1 % over the few km of a UAS mission.  (Don't use
    JSBSim's position/distance-from-start-lat/lon-mt for this: they are
    UNSIGNED distances, so going west or south still reads positive.)
    """
    north = (np.asarray(lat_rad) - lat0_rad) * R_EARTH_M
    east = (np.asarray(lon_rad) - lon0_rad) * R_EARTH_M * np.cos(lat0_rad)
    return north, east


def ne_position(fdm) -> np.ndarray:
    """Local [north, east] in metres from the initial-condition point (ic/lat, ic/long)."""
    n, e = ne_from_latlon(fdm["position/lat-geod-rad"], fdm["position/long-gc-rad"],
                          fdm["ic/lat-geod-rad"], fdm["ic/long-gc-rad"])
    return np.array([float(n), float(e)])


def line_course(p, chi, r, q, chi_inf=math.radians(60), k_path=0.02):
    """Commanded course to converge onto the line through r with direction q.

    chi_c = chi_q - chi_inf * (2/pi) * atan(k_path * e_py)    (B&M eq. 10.8)
    e_py is the signed cross-track error (positive = right of the path).
    Returns (chi_c, e_py).
    """
    chi_q = math.atan2(q[1], q[0])
    chi_q += 2 * math.pi * round((chi - chi_q) / (2 * math.pi))     # unwrap toward current course
    d = np.asarray(p) - np.asarray(r)
    e_py = -math.sin(chi_q) * d[0] + math.cos(chi_q) * d[1]
    return chi_q - chi_inf * (2 / math.pi) * math.atan(k_path * e_py), e_py


def orbit_course(p, chi, c, rho, lam=1, k_orbit=2.0):
    """Commanded course to converge onto an orbit (centre c, radius rho, lam=+1 CW, -1 CCW).

    chi_c = phi + lam (pi/2 + atan(k_orbit (d - rho)/rho))   (B&M eq. 10.13)
    Returns (chi_c, radial error d - rho).
    """
    dn, de = p[0] - c[0], p[1] - c[1]
    d = math.hypot(dn, de)
    phi = math.atan2(de, dn)
    phi += 2 * math.pi * round((chi - phi) / (2 * math.pi))
    return phi + lam * (math.pi / 2 + math.atan(k_orbit * (d - rho) / rho)), d - rho


@dataclass
class WaypointManager:
    """Fly waypoints [(north_m, east_m, alt_ft), ...] with straight-line legs.

    Called as a ``gnclab.run`` callback; runs at ``rate_hz``.  When the last
    waypoint is reached it orbits it (radius ``loiter_radius_m``).
    """

    waypoints: list
    chi_inf: float = math.radians(60)
    k_path: float = 0.02
    k_orbit: float = 2.0
    loiter_radius_m: float = 120.0
    rate_hz: float = 10.0
    i: int = 1                                   # current leg is waypoints[i-1] -> waypoints[i]
    mode: str = "line"
    log: dict = field(default_factory=lambda: {"t": [], "leg": [], "xtrack_m": [], "chi_c": []})
    _t_next: float = 0.0

    def __call__(self, fdm, t):
        if t + 1e-9 < self._t_next:
            return
        self._t_next += 1.0 / self.rate_hz
        p = ne_position(fdm)
        chi = fdm["flight-path/psi-gt-rad"]
        wp = [np.array(w[:2], dtype=float) for w in self.waypoints]
        if self.mode == "line":
            r, w = wp[self.i - 1], wp[self.i]
            q = (w - r) / np.linalg.norm(w - r)
            if np.dot(p - w, q) >= 0:                # crossed the half-plane at w -> next leg
                if self.i + 1 < len(wp):
                    self.i += 1
                    r, w = wp[self.i - 1], wp[self.i]
                    q = (w - r) / np.linalg.norm(w - r)
                else:
                    self.mode = "orbit"
            fdm["ap/alt-cmd-ft"] = self.waypoints[self.i][2]
        if self.mode == "line":
            chi_c, err = line_course(p, chi, r, q, self.chi_inf, self.k_path)
        else:
            chi_c, err = orbit_course(p, chi, wp[-1], self.loiter_radius_m, 1, self.k_orbit)
            fdm["ap/alt-cmd-ft"] = self.waypoints[-1][2]
        fdm["ap/chi-cmd-rad"] = chi_c
        self.log["t"].append(t)
        self.log["leg"].append(self.i if self.mode == "line" else -1)
        self.log["xtrack_m"].append(err)
        self.log["chi_c"].append(chi_c)
