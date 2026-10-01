"""Module 11 - how slow / how late can the autopilot be?

Fly the same 100 ft altitude step with the Python autopilot at different
controller frame rates and output latencies.  Compare with the linear delay
margin of the inner pitch loop (~90 ms, Module 09).  A sampled controller with
zero-order hold behaves like an extra delay of about half a frame (T/2).

    python modules/11_robustness/solutions/rate_and_delay.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.controllers import LongitudinalAP  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402
from gnclab.trim import trim  # noqa: E402

T = 25.0


def fly(rate_hz, latency_s, realistic=True):
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048})
    trim(fdm, "full")
    if realistic:
        fdm["fcs/actuators-on"] = 1
        fdm["sensors/enabled"] = 1
    ap = LongitudinalAP(fdm, rate_hz=rate_hz, latency_s=latency_s)
    h0 = fdm["position/h-sl-ft"]

    def cb(f, t):
        if t >= 2:
            ap.alt_cmd = h0 + 100
        ap(f, t)

    df = run(fdm, T, {"alt_ft": "position/h-sl-ft", "q_dps": "velocities/q-rad_sec",
                      "elevator_deg": "fcs/elevator-pos-rad"}, callback=cb, record_every=2)
    df["q_dps"] = np.degrees(df.q_dps)
    df["elevator_deg"] = np.degrees(df.elevator_deg)
    tail = df.loc[12.0:]
    return df, {"rate [Hz]": rate_hz, "latency [ms]": 1000 * latency_s,
                "effective delay ~ latency + T/2 [ms]": 1000 * (latency_s + 0.5 / rate_hz),
                "alt overshoot [ft]": df.alt_ft.max() - (h0 + 100),
                "residual q osc [deg/s]": tail.q_dps.abs().max(),
                "elevator rms [deg]": float(np.sqrt(np.mean((tail.elevator_deg - tail.elevator_deg.mean()) ** 2)))}


cases = [(120, 0.0), (50, 0.0), (25, 0.0), (10, 0.0), (50, 0.02), (50, 0.05), (50, 0.08), (50, 0.10)]
if args.fast:
    cases = [(120, 0.0), (25, 0.0), (50, 0.05)]
rows, runs = [], {}
for rate, lat in cases:
    df, row = fly(rate, lat)
    rows.append(row)
    runs[f"{rate} Hz, +{1000 * lat:.0f} ms"] = df
res = pd.DataFrame(rows)
print(res.round(2).to_string(index=False))
print("\nLinear inner-loop delay margin (Module 09): ~90 ms (includes the actuator, not the sensors).")
pick = {k: v for k, v in runs.items() if k in ("120 Hz, +0 ms", "25 Hz, +0 ms", "10 Hz, +0 ms", "50 Hz, +50 ms", "50 Hz, +80 ms")}
fig, _ = timehistory(pick, ["alt_ft", "q_dps", "elevator_deg"], title="Controller frame rate and latency")
save(fig, "11_robustness", "rate_and_delay", show=args.show)
