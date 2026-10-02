"""Engage / command the gnc_trainer autopilot (aircraft/gnc_trainer/fcs/gnc_autopilot.xml).

Engagement is *bumpless*: the current state becomes the reference, so the
autopilot starts with zero error and the airplane does not jump.
"""

from __future__ import annotations

import jsbsim


def engage_longitudinal(fdm: jsbsim.FGFDMExec, altitude: bool = True, speed: bool = True,
                        alt_cmd_ft: float | None = None, vt_cmd_fps: float | None = None) -> None:
    """Pitch hold always; altitude hold and airspeed hold optionally."""
    fdm["ap/theta-trim-rad"] = fdm["attitude/theta-rad"]
    fdm["ap/theta-cmd-ext-rad"] = 0.0
    fdm["ap/alt-cmd-ft"] = fdm["position/h-sl-ft"] if alt_cmd_ft is None else alt_cmd_ft
    fdm["ap/vt-cmd-fps"] = fdm["velocities/vt-fps"] if vt_cmd_fps is None else vt_cmd_fps
    fdm["ap/pitch-hold-on"] = 1
    fdm["ap/alt-hold-on"] = 1 if altitude else 0
    fdm["ap/speed-hold-on"] = 1 if speed else 0


def engage_lateral(fdm: jsbsim.FGFDMExec, course: bool = True, yaw_damper: bool = True,
                   course_cmd_rad: float | None = None) -> None:
    """Roll hold always; course hold and yaw damper optionally (Module 10)."""
    fdm["ap/phi-cmd-ext-rad"] = 0.0
    fdm["ap/chi-cmd-rad"] = fdm["flight-path/psi-gt-rad"] if course_cmd_rad is None else course_cmd_rad
    fdm["ap/roll-hold-on"] = 1
    fdm["ap/course-hold-on"] = 1 if course else 0
    fdm["ap/yaw-damper-on"] = 1 if yaw_damper else 0


def disengage_all(fdm: jsbsim.FGFDMExec) -> None:
    for p in ("ap/pitch-hold-on", "ap/alt-hold-on", "ap/speed-hold-on",
              "ap/roll-hold-on", "ap/course-hold-on", "ap/yaw-damper-on", "ap/guidance-on"):
        try:
            fdm[p] = 0
        except KeyError:
            pass


def realism(fdm: jsbsim.FGFDMExec, on: bool = True) -> None:
    """Switch realistic actuators and sensor models on (or off)."""
    fdm["fcs/actuators-on"] = 1 if on else 0
    fdm["sensors/enabled"] = 1 if on else 0
