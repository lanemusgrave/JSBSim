"""Module 11 - gyro noise vs phase lag: choosing the pitch-rate filter.

The pitch loop's derivative path (-kd * q) multiplies gyro noise straight into
the elevator.  A first-order low-pass on q cuts that activity but adds phase
lag at the loop crossover (~16 rad/s here), costing damping.  Sweep the filter
corner and look at both sides of the trade, with the sensor noise multiplied
by 4 to make it obvious.

    python modules/11_robustness/solutions/noise_filter.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.controllers import LongitudinalAP  # noqa: E402
from gnclab.plotting import save  # noqa: E402
from gnclab.signals import doublet  # noqa: E402
from gnclab.trim import trim  # noqa: E402

import control  # noqa: E402

from gnclab.controllers import DEFAULT_GAINS  # noqa: E402
from gnclab.linear import linearize_fd  # noqa: E402
from gnclab.metrics import loop_margins  # noqa: E402

# linear pitch loop at 25 m/s, for the phase-margin cost of each filter
_f = make_fdm("gnc_trainer")
initialize(_f, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048})
trim(_f, "full")
_lin = linearize_fd(_f, ["fcs/elevator-cmd-norm"]).subsystem(["Vt", "Alpha", "Theta", "Q"], ["fcs/elevator-cmd-norm"])


def _tf(i):
    C = np.zeros((1, 4))
    C[0, i] = 1
    return control.tf(control.ss(_lin.A, _lin.B, C, 0.0)) * control.tf([40], [1, 40])


_th, _q = _tf(2), _tf(3)


def pitch_pm(fc_hz):
    F = 1 if fc_hz is None else control.tf([2 * np.pi * fc_hz], [1, 2 * np.pi * fc_hz])
    m = loop_margins(DEFAULT_GAINS["kp_theta"] * _th + DEFAULT_GAINS["kd_theta"] * _q * F)
    return m.phase_margin_deg, 1000 * m.delay_margin_s


rng = np.random.default_rng(1)
corners = [None, 20.0, 10.0, 5.0, 2.0] if not args.fast else [None, 5.0]
rows = []
for fc in corners:
    fdm = make_fdm("gnc_trainer")
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048})
    trim(fdm, "full")
    fdm["fcs/actuators-on"] = 1
    ap = LongitudinalAP(fdm, q_filter_hz=fc)
    theta0 = ap.theta_trim

    # extra gyro noise injected in Python (4x the sensor model's 0.0025 rad/s)
    orig_step = ap.step

    def noisy_step(dt, ap=ap, orig=orig_step):
        q_true = fdm["fb/q-rad_sec"]
        fdm["fb/q-rad_sec"] = q_true + rng.normal(0, 0.01)
        out = orig(dt)
        return out
    ap.step = noisy_step

    def cb(f, t, ap=ap):
        ap.theta_trim = theta0 + np.radians(5) * (1.0 if 2 <= t < 6 else 0.0)  # pitch step via the bias
        ap(f, t)

    ap.gains["kp_h"] = ap.gains["ki_h"] = 0.0      # pure pitch-attitude hold for this test
    df = run(fdm, 10.0, {"theta": "attitude/theta-deg", "de": "fcs/elevator-pos-rad"}, callback=cb)
    seg = df.loc[2.0:6.0]
    final = seg.loc[5.0:6.0].theta.mean()        # the loop's DC gain is < 1: measure against what it achieves
    over = 100 * (seg.theta.max() - final) / (final - np.degrees(theta0))
    quiet = df.loc[7.0:10.0]
    rms_rate = np.degrees(np.sqrt(np.mean(np.diff(quiet.de.to_numpy()) ** 2))) * 120
    rows.append((fc, over, rms_rate))
    pm, dm = pitch_pm(fc)
    print(f"q filter {'none' if fc is None else f'{fc:4.0f} Hz'}: linear PM {pm:5.1f} deg, delay margin {dm:4.0f} ms | "
          f"pitch-step overshoot {over:4.1f} %, RMS elevator rate (noise) {rms_rate:5.1f} deg/s")

fig, ax = plt.subplots(figsize=(7, 4.5))
labels = ["none" if fc is None else f"{fc:.0f} Hz" for fc, _, _ in rows]
ax.plot([r[2] for r in rows], [r[1] for r in rows], "o-")
for (fc, o, r), lab in zip(rows, labels):
    ax.annotate(lab, (r, o), textcoords="offset points", xytext=(5, 5))
ax.set(xlabel="RMS elevator rate from noise [deg/s] (servo wear, power)", ylabel="pitch-step overshoot [%]",
       title="Gyro filter corner: noise vs damping")
ax.grid(alpha=0.3)
save(fig, "11_robustness", "noise_filter_tradeoff", show=args.show)
