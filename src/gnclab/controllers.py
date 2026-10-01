"""A Python implementation of the gnc_trainer longitudinal autopilot.

Same control law as the XML autopilot (Module 09), but running in Python so
you can change what flight software changes: frame rate, latency,
anti-windup, filtering, gain scheduling.  Use it as a ``gnclab.run`` callback:

    ap = LongitudinalAP(fdm, rate_hz=50, latency_s=0.02)
    df = run(fdm, 60, props, callback=ap)

It reads the ``fb/...`` feedback properties (so ``sensors/enabled`` applies)
and drives the *pilot* inputs ``fcs/elevator-cmd-norm`` / ``fcs/throttle-cmd-norm``
around their trimmed values.  Keep the XML longitudinal loops OFF while using it
(their switches write ``ap/...`` every frame).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

DEFAULT_GAINS = {  # from modules/09_longitudinal_autopilot/solutions/design_longitudinal.py
    "kp_theta": -3.81972, "kd_theta": -0.895563,
    "kp_h": 0.010291, "ki_h": 0.00070766,
    "kp_v": 0.01127, "ki_v": 0.00406191,
}


@dataclass
class LongitudinalAP:
    fdm: object
    rate_hz: float = 120.0                 # controller frame rate
    latency_s: float = 0.0                 # extra transport delay on the outputs
    gains: dict = field(default_factory=lambda: dict(DEFAULT_GAINS))
    anti_windup: bool = True
    q_filter_hz: float | None = None       # first-order low-pass on the gyro (None = off)
    schedule: Callable[[float], float] | None = None   # gain multiplier vs qbar [psf]
    theta_max: float = math.radians(15.0)
    alt_zone_ft: float = 60.0
    wing_leveler: bool = True              # hold wings level with the XML roll loop

    def __post_init__(self):
        f = self.fdm
        self.de_trim = f["fcs/elevator-cmd-norm"]
        self.dt_trim = f["fcs/throttle-cmd-norm[0]"]
        self.theta_trim = f["attitude/theta-rad"]
        self.alt_cmd = f["position/h-sl-ft"]
        self.vt_cmd = f["velocities/vt-fps"]
        self.int_h = self.int_v = 0.0
        self.q_filt = 0.0
        self.t_next = 0.0
        self.period = 1.0 / self.rate_hz
        self.out = (0.0, 0.0)
        n = max(0, int(round(self.latency_s * self.rate_hz)))
        self.queue = deque([(0.0, 0.0)] * n)
        self.saturated = False
        if self.wing_leveler:
            f["ap/phi-cmd-ext-rad"] = 0.0
            f["ap/roll-hold-on"] = 1
            f["ap/yaw-damper-on"] = 1

    # ------------------------------------------------------------------
    def step(self, dt: float) -> tuple[float, float]:
        """One controller frame: returns (elevator increment, throttle increment)."""
        f, g = self.fdm, self.gains
        k = self.schedule(f["aero/qbar-psf"]) if self.schedule else 1.0
        q = f["fb/q-rad_sec"]
        if self.q_filter_hz:
            a = dt * 2 * math.pi * self.q_filter_hz
            self.q_filt += a / (1 + a) * (q - self.q_filt)
            q = self.q_filt
        # altitude PI -> theta command (incremental about trim)
        e_h = self.alt_cmd - f["fb/h-ft"]
        thc_raw = g["kp_h"] * e_h + g["ki_h"] * self.int_h
        thc = max(-self.theta_max, min(self.theta_max, thc_raw))
        frozen = self.anti_windup and (abs(e_h) > self.alt_zone_ft or thc != thc_raw)
        if not frozen:
            self.int_h += e_h * dt
        # pitch PD
        e_th = self.theta_trim + thc - f["fb/theta-rad"]
        de = k * (g["kp_theta"] * e_th - g["kd_theta"] * q)
        de = max(-1.0, min(1.0, de))
        # airspeed PI on throttle
        e_v = self.vt_cmd - f["fb/vt-fps"]
        dthr_raw = g["kp_v"] * e_v + g["ki_v"] * self.int_v
        total = self.dt_trim + dthr_raw
        sat_v = total > 1.0 or total < 0.0
        if not (self.anti_windup and sat_v):
            self.int_v += e_v * dt
        return de, dthr_raw

    def __call__(self, fdm, t: float) -> None:
        if t + 1e-9 >= self.t_next:                 # controller frame
            self.t_next += self.period
            new = self.step(self.period)
            if self.queue is not None and len(self.queue) > 0:
                self.queue.append(new)
                self.out = self.queue.popleft()
            else:
                self.out = new
        de, dthr = self.out                          # zero-order hold between frames
        fdm["fcs/elevator-cmd-norm"] = self.de_trim + de
        fdm["fcs/throttle-cmd-norm"] = min(1.0, max(0.0, self.dt_trim + dthr))
