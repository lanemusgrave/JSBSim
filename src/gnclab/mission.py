"""The capstone mission (Module 18): the whole GNC stack in one function.

    WaypointManager (10 Hz)  ->  ap/chi-cmd-rad  ->  XML course/roll/yaw-damper loops (fb/* sensors)
         (guidance)          ->  ap/alt-cmd-ft   ->  LongitudinalAP (40 Hz, Python) on
                                                     AttitudeEKF + AltitudeKF estimates (Python sensors)
    gnc_trainer with realistic actuators, sensor noise and bias, wind, turbulence,
    and any dispersions from gnclab.montecarlo (mass, aero, actuators, seeds).

:func:`fly_mission` returns metrics; :data:`MISSION_REQUIREMENTS` plugs into
``gnclab.montecarlo.evaluate``.  Guidance uses true position: GPS error
(~1.5 m) is small next to the path-following errors being judged.
"""

from __future__ import annotations

import math
from typing import Mapping

import numpy as np
import pandas as pd

FT = 0.3048

# north m, east m, altitude ft: a 600 m box with a climb and a descent, back to the start
MISSION_WPS = [(0, 0, 330), (600, 0, 430), (600, 600, 430), (0, 600, 360), (0, 0, 360)]
T_MISSION = 130.0

# id: (metric, op, limit, text)  - same format as gnclab.montecarlo.REQUIREMENTS
MISSION_REQUIREMENTS: dict[str, tuple[str, str, float, str]] = {
    "MR-1": ("completed", ">=", 1.0, "all waypoints reached within the mission time"),
    "MR-2": ("xtrack_max_m", "<=", 30.0, "|cross-track| <= 30 m on every leg after a 15 s capture"),
    "MR-3": ("xtrack_mean_m", "<=", 8.0, "mean |cross-track| <= 8 m after capture"),
    "MR-4": ("alt_err_max_ft", "<=", 40.0, "|altitude error| <= 40 ft once 30 s past a command change"),
    "MR-5": ("vt_min_mps", ">=", 18.0, "airspeed never below 18 m/s"),
    "MR-6": ("phi_max_deg", "<=", 35.0, "|bank| <= 35 deg"),
    "MR-7": ("theta_est_err_rms_deg", "<=", 2.0, "pitch estimate error RMS <= 2 deg (nav health)"),
}

LOG = {"lat": "position/lat-geod-rad", "lon": "position/long-gc-rad", "alt_ft": "position/h-sl-ft",
       "alt_cmd_ft": "ap/alt-cmd-ft", "phi_deg": "attitude/phi-deg", "theta": "attitude/theta-rad",
       "vt_fps": "velocities/vt-fps", "elevator": "fcs/elevator-pos-rad", "throttle": "fcs/throttle-total-norm"}


def fly_mission(case: Mapping[str, float], T: float = T_MISSION, keep_history: bool = False,
                waypoints=None, manager: str = "line") -> dict:
    """Fly the mission for one case (dispersion dict as in gnclab.montecarlo).

    ``manager``: "line" (switch legs at the waypoint half-plane) or "fillet"
    (turn onto the next leg along a 120 m arc; Module 12 exercise A).
    The case may also carry ``"manager"``, which overrides the argument.
    """
    from gnclab import initialize, make_fdm
    from gnclab.autopilot import engage_lateral, realism
    from gnclab.controllers import LongitudinalAP
    from gnclab.estimation import AltitudeKF, AttitudeEKF, Sensors
    from gnclab.guidance import FilletManager, WaypointManager, ne_from_latlon
    from gnclab.montecarlo import apply_dispersions, turbulence_on
    from gnclab.trim import TrimError, trim

    wps = waypoints or MISSION_WPS
    try:
        fdm = make_fdm("gnc_trainer")
        apply_dispersions(fdm, case)
        initialize(fdm, {"lat-geod-deg": 33.7, "long-gc-deg": -117.9, "h-sl-ft": wps[0][2],
                         "vt-fps": 25 / FT, "psi-true-deg": 0.0})
        try:
            trim(fdm, "full")
        except TrimError:
            return {"status": "trim-fail"}
        realism(fdm, True)
        turbulence_on(fdm, case.get("turb_w20_fps", 0.0))
        engage_lateral(fdm, course=True)

        dt = fdm.get_delta_t()
        sens = Sensors(seed=int(case.get("seed", 0)) % (2**31))
        ekf = AttitudeEKF()
        ekf.x[:] = fdm["attitude/phi-rad"], fdm["attitude/theta-rad"]
        akf = AltitudeKF(fdm["position/h-sl-ft"] * FT)
        est = {}
        ap = LongitudinalAP(fdm, rate_hz=40.0, wing_leveler=False, feedback=lambda _f: est)
        kind = case.get("manager", manager)
        wm = (FilletManager if kind == "fillet" else WaypointManager)(wps, loiter_radius_m=150.0)
        lat0, lon0 = fdm["ic/lat-geod-rad"], fdm["ic/long-gc-rad"]
        rows, t_rows = [], []
        k = 0
        while fdm.get_sim_time() < T - 0.5 * dt:
            t = fdm.get_sim_time()
            m = sens.read(fdm, t)                                   # sensors, every frame
            phi_e, th_e = ekf.update(m["gyro"], m["accel"], dt, Va=m["Va"])
            h_e, _ = akf.update(m["accel"], phi_e, th_e, m["baro_h"], dt)
            est.update(theta=th_e, q=m["gyro"][1], h_ft=h_e / FT, vt_fps=m["Va"] / FT)
            wm(fdm, t)                                              # guidance, 10 Hz
            ap.alt_cmd = fdm["ap/alt-cmd-ft"]
            ap(fdm, t)                                              # longitudinal AP, 40 Hz
            if k % 12 == 0:                                         # log at 10 Hz
                t_rows.append(t)
                rows.append([fdm[p] for p in LOG.values()] + [th_e, wm.i if wm.mode == "line" else -1])
            if not fdm.run():
                break
            k += 1
    except Exception as exc:                                        # noqa: BLE001
        return {"status": f"error: {type(exc).__name__}: {exc}"[:120]}

    df = pd.DataFrame(rows, columns=list(LOG) + ["theta_est", "leg"], index=pd.Index(t_rows, name="t"))
    if not np.all(np.isfinite(df.to_numpy())):
        return {"status": "diverged"}
    df["north_m"], df["east_m"] = ne_from_latlon(df.lat.to_numpy(), df.lon.to_numpy(), lat0, lon0)

    # cross-track on legs, after a 15 s capture following each leg switch
    lt, leg, xt = (np.asarray(wm.log[c]) for c in ("t", "leg", "xtrack_m"))
    switch_t = {i: lt[np.argmax(leg == i)] for i in set(leg.tolist()) if i > 0}
    steady = np.array([g > 0 and tt >= switch_t[g] + 15.0 for g, tt in zip(leg, lt)])
    # altitude: once 30 s past the last command change
    cmd = df.alt_cmd_ft.to_numpy()
    t = df.index.to_numpy()
    change_t = np.r_[t[0], t[1:][np.abs(np.diff(cmd)) > 0.5]]
    last_change = change_t[np.searchsorted(change_t, t, side="right") - 1]
    settled = t >= last_change + 30.0
    out = {
        "status": "ok",
        "completed": float(wm.mode == "orbit"),
        "xtrack_max_m": float(np.abs(xt[steady]).max()) if steady.any() else np.nan,
        "xtrack_mean_m": float(np.abs(xt[steady]).mean()) if steady.any() else np.nan,
        "alt_err_max_ft": float(np.abs(df.alt_ft - df.alt_cmd_ft)[settled].max()) if settled.any() else np.nan,
        "vt_min_mps": float(df.vt_fps.min() * FT),
        "phi_max_deg": float(df.phi_deg.abs().max()),
        "theta_est_err_rms_deg": float(np.degrees(np.sqrt(np.mean((df.theta_est - df.theta) ** 2)))),
        "elevator_rms_deg": float(np.degrees(df.elevator.std())),
        "t_complete_s": float(lt[np.argmax(leg == -1)]) if (leg == -1).any() else np.nan,
    }
    if keep_history:
        out["history"] = df
        out["guidance_log"] = pd.DataFrame(wm.log)
    return out
