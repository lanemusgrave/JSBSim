"""Module 11 exercises - solutions for A (back-calculation) and B (latency budget)."""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.controllers import LongitudinalAP  # noqa: E402
from gnclab.trim import trim  # noqa: E402


class BackCalcAP(LongitudinalAP):
    """Altitude integrator with back-calculation instead of freezing."""

    Tt: float = 10.0  # tracking time constant [s]

    def step(self, dt):
        g = self.gains
        e_h = self.alt_cmd - self.fdm["fb/h-ft"]
        thc_raw = g["kp_h"] * e_h + g["ki_h"] * self.int_h
        thc = max(-self.theta_max, min(self.theta_max, thc_raw))
        self.int_h += (e_h + (thc - thc_raw) / (g["ki_h"] * self.Tt)) * dt
        saved, self.anti_windup = self.anti_windup, False   # let the parent do the rest without its freeze
        self.int_h -= e_h * dt                               # parent adds e_h*dt again
        out = super().step(dt)
        self.anti_windup = saved
        return out


def trimmed(realistic=False):
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048})
    trim(fdm, "full")
    if realistic:
        fdm["fcs/actuators-on"] = fdm["sensors/enabled"] = 1
    return fdm


print("A. 400 ft climb")
for name, cls, aw in (("no anti-windup", LongitudinalAP, False), ("freeze (clamping)", LongitudinalAP, True),
                      ("back-calculation", BackCalcAP, False)):
    fdm = trimmed()
    ap = cls(fdm, anti_windup=aw)
    h0 = fdm["position/h-sl-ft"]

    def cb(f, t, ap=ap):
        if t >= 2:
            ap.alt_cmd = h0 + 400
        ap(f, t)

    df = run(fdm, 70.0, {"h": "position/h-sl-ft"}, callback=cb, record_every=12)
    print(f"   {name:18s}: overshoot {df.h.max() - h0 - 400:6.1f} ft")

print("B. latency budget at 25 Hz")
# A frame-based controller can only delay its output by WHOLE frames (40 ms at 25 Hz),
# so sweep latency in frames.  With sensors on, there is a noise floor of residual
# oscillation even with zero latency - judge "unacceptable" relative to that baseline.
baseline, budget = None, None
for frames in range(0, 4):
    lat = frames / 25.0
    fdm = trimmed(realistic=True)
    ap = LongitudinalAP(fdm, rate_hz=25.0, latency_s=lat)
    h0 = fdm["position/h-sl-ft"]

    def cb(f, t, ap=ap):
        if t >= 2:
            ap.alt_cmd = h0 + 100
        ap(f, t)

    df = run(fdm, 20.0, {"q": "velocities/q-rad_sec"}, callback=cb, record_every=2)
    osc = np.degrees(df.q[df.index > 12].abs().max())
    baseline = baseline or osc
    verdict = "ok" if osc < 3 * baseline else "UNACCEPTABLE (limit cycle / unstable)"
    print(f"   {frames} frame(s) = {1000 * lat:3.0f} ms: residual q {osc:6.2f} deg/s  {verdict}")
    if verdict == "ok":
        budget = lat
print(f"   -> at 25 Hz the budget is {1000 * budget:.0f} ms of output latency "
      f"(effective delay {1000 * (budget + 0.02):.0f} ms incl. the half-frame hold),\n"
      "      consistent with the ~90 ms linear delay margin once sensor lag/noise are included.")
