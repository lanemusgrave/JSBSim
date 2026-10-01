"""Module 08 exercises - solutions for A and B (C is discussed in answers.md)."""

from pathlib import Path

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.signals import doublet  # noqa: E402
from gnclab.trim import trim  # noqa: E402

# --- A: actuator characterization -------------------------------------------
def rig_step(amp):
    fdm = make_fdm("m08_fcs_rig")
    initialize(fdm, {"h-sl-ft": 50000.0, "vt-fps": 0.0}, engines_running=False)
    df = run(fdm, 2.0, ["rig/act-realistic"],
             callback=lambda f, t: f.__setitem__("rig/input", amp if t >= 0.5 else 0.0))
    return df.index.to_numpy(), df["rig/act-realistic"].to_numpy()


t, y = rig_step(5.0)                       # big step: saturates rate AND travel
moving = (y > 0.05) & (y < 0.7)
rate = np.polyfit(t[moving], y[moving], 1)[0]
delay = t[np.argmax(y > 1e-6)] - 0.5
print(f"A. large step: rate limit ~ {rate:.2f} /s, travel limit {y.max():.2f}, delay ~ {delay:.3f} s")
t, y = rig_step(0.02)                      # tiny step: stays below the rate limit -> pure lag
t63 = t[np.argmax(y >= 0.632 * 0.02)] - 0.5 - delay
print(f"   small step: 63% rise after the delay = {t63:.3f} s -> lag bandwidth ~ {1 / t63:.0f} rad/s")
print("   (XML: lag 30, rate_limit 2.0, clip +/-0.8, delay 0.05 s)")

# --- B: nonlinear stability limit with 0.1 s delay ---------------------------------
FCS = Path(__file__).with_name("fcs")


def residual(kq):
    fdm = make_fdm("gnc_trainer", systems_dir=FCS)
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048})
    trim(fdm, "full")
    for k, v in {"ap/pitch-damper-on": 1, "ap/kq": kq, "fcs/actuators-on": 1, "ap/q-delay-on": 1}.items():
        fdm[k] = v
    de0 = fdm["fcs/elevator-cmd-norm"]
    df = run(fdm, 8.0, {"q": "velocities/q-rad_sec"},
             callback=lambda f, t: f.__setitem__("fcs/elevator-cmd-norm", de0 + doublet(t, 0.5, 0.3, -0.2)))
    return np.degrees(df.q[df.index > 4].abs().max())


gains = np.arange(0.6, 1.31, 0.2 if args.fast else 0.05)
onset = None
for kq in gains:
    r = residual(kq)
    print(f"B. Kq = {kq:.2f}: max |q| between 4 and 8 s = {r:6.2f} deg/s")
    if r > 5 and onset is None:
        onset = kq
print(f"   sustained oscillation from Kq ~ {onset:.2f} (linear prediction ~0.97 with 3rd-order Pade)")
