"""Module 08 - drive every FCS component type with the same inputs and look.

Runs aircraft/m08_fcs_rig twice: a step-and-back input, then a 0.5 Hz sine.
One figure per family (actuators, sensors, filters, logic), plus a check of
JSBSim's discrete lag filter against the exact continuous step response.

    python modules/08_fcs_components/solutions/fcs_rig.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.plotting import save  # noqa: E402

FAMILIES = {
    "actuators": ["rig/act-lag", "rig/act-rate", "rig/act-deadband", "rig/act-hysteresis", "rig/act-realistic"],
    "sensors": ["rig/sens-noise", "rig/sens-bias-drift", "rig/sens-quantized", "rig/sens-lag", "rig/sens-delay"],
    "filters": ["rig/flt-lag", "rig/flt-leadlag", "rig/flt-washout", "rig/flt-notch", "rig/flt-integrator", "rig/pid"],
    "logic": ["rig/gain", "rig/sum", "rig/switch", "rig/function", "rig/kinematic"],
}
ALL = ["rig/input"] + [p for v in FAMILIES.values() for p in v]


def fly(signal, duration):
    fdm = make_fdm("m08_fcs_rig")
    initialize(fdm, {"h-sl-ft": 50000.0, "vt-fps": 0.0}, engines_running=False)
    return run(fdm, duration, ALL, callback=lambda f, t: f.__setitem__("rig/input", signal(t)))


T = 6.0 if args.fast else 10.0
step = fly(lambda t: 1.0 if 0.5 <= t < 5.0 else 0.0, T)
sine = fly(lambda t: np.sin(2 * np.pi * 0.5 * t), T)

for family, props in FAMILIES.items():
    fig, axes = plt.subplots(len(props), 2, sharex=True, figsize=(12, 1.9 * len(props) + 1))
    for row, prop in enumerate(props):
        for col, (df, title) in enumerate(((step, "step and back"), (sine, "0.5 Hz sine"))):
            ax = axes[row, col]
            ax.plot(df.index, df["rig/input"], color="0.7", lw=1, label="input")
            ax.plot(df.index, df[prop], label=prop)
            ax.grid(alpha=0.3)
            if row == 0:
                ax.set_title(title)
            if col == 0:
                ax.set_ylabel(prop.split("/")[1], fontsize="small")
    for ax in axes[-1]:
        ax.set_xlabel("time [s]")
    fig.suptitle(f"JSBSim FCS components: {family}")
    fig.tight_layout()
    save(fig, "08_fcs_components", f"rig_{family}", show=args.show)

# JSBSim's lag_filter is a Tustin (bilinear) discretization of C1/(s+C1).
t = step.index.to_numpy()
exact = np.where(t >= 0.5, 1 - np.exp(-5.0 * (t - 0.5)), 0.0)
mask = t < 5.0
err = np.max(np.abs(step["rig/flt-lag"].to_numpy()[mask] - exact[mask]))
print(f"lag_filter vs exact 1 - exp(-5 t): max error {err:.4f} (one-step timing + Tustin)")
d = step["rig/sens-delay"].to_numpy()
print(f"sensor delay: output first moves at t = {t[np.argmax(d > 0.5)]:.3f} s (input stepped at 0.5 s)")
print(f"realistic actuator: travel clipped at {step['rig/act-realistic'].max():.2f}, "
      f"reaches 0.79 at t = {t[np.argmax(step['rig/act-realistic'].to_numpy() > 0.79)]:.2f} s")
print(f"noise sensor std = {step['rig/sens-noise'][step.index < 0.5].std():.4f} (spec 0.05)")
