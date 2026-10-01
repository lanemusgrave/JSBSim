"""Module 06 - when does the linear model stop being right?

Part 1 (m04_glider): linearize with our own finite-difference linearizer
(gnclab.linear.linearize_fd - JSBSim's FGLinearization needs an engine), then
apply the same elevator doublet to the nonlinear JSBSim model and to the linear
model, small and large.  The linear model works in *perturbation* variables:
x = x_trim + dx.

Part 2 (c172x): a hard nonlinearity.  The C172's elevator actuator has a
0.05 rad hysteresis band.  Small commands never move the surface, and after a
large doublet the surface does not come back to its trim position.

    python modules/06_linear_models_modes/solutions/linear_vs_nonlinear.py [--show]
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import control  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.linear import linearize_fd, modes  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402
from gnclab.signals import doublet  # noqa: E402
from gnclab.trim import trim, trim_custom  # noqa: E402

T_END = 15.0 if args.fast else 30.0
LONG = ["Vt", "Alpha", "Theta", "Q"]


def trimmed_glider():
    fdm = make_fdm("m04_glider")
    initialize(fdm, {"lat-geod-deg": 33, "h-sl-ft": 3000, "vt-fps": 48.0}, engines_running=False)
    trim_custom(fdm, {"ic/vt-fps": 48.0},
                {"ic/alpha-rad": (0.05, -0.2, 0.3), "ic/gamma-rad": (-0.05, -0.5, 0.2),
                 "fcs/pitch-trim-cmd-norm": (0.0, -1.0, 1.0)})
    return fdm


fdm = trimmed_glider()
lin = linearize_fd(fdm, ["fcs/elevator-cmd-norm", "fcs/aileron-cmd-norm", "fcs/rudder-cmd-norm"])
print("glider modes (finite-difference linearization, 8 states):")
print(modes(lin.A, lin.x_names)[["eig_real", "eig_imag", "wn", "zeta", "period_s", "dominant_state"]].round(4))
lon = lin.subsystem(LONG, ["fcs/elevator-cmd-norm"])
sys = control.ss(lon.A, lon.B, np.eye(4), np.zeros((4, 1)))

fig, axes = plt.subplots(4, 2, sharex=True, figsize=(12, 9))
for col, amp in enumerate((0.05, 0.40)):
    fdm = trimmed_glider()
    x0 = {"Vt": fdm["velocities/vt-fps"], "Alpha": fdm["aero/alpha-rad"],
          "Theta": fdm["attitude/theta-rad"], "Q": fdm["velocities/q-rad_sec"]}
    de0 = fdm["fcs/elevator-cmd-norm"]

    def pilot(f, t, amp=amp):
        f["fcs/elevator-cmd-norm"] = de0 + doublet(t, 1.0, 0.5, -amp)

    nl = run(fdm, T_END, {"Vt": "velocities/vt-fps", "Alpha": "aero/alpha-rad",
                          "Theta": "attitude/theta-rad", "Q": "velocities/q-rad_sec"}, callback=pilot)
    t = nl.index.to_numpy()
    # Compare like with like: JSBSim holds the input over each step (zero-order
    # hold) and an input written before run() first affects the state ONE frame
    # later (run() integrates first, then evaluates FCS/aero).  So: discretize
    # the linear model with ZOH at the same dt and delay its input one sample.
    dt = fdm.get_delta_t()
    dsys = control.c2d(sys, dt, "zoh")
    u = np.r_[0.0, doublet(t, 1.0, 0.5, -amp)[:-1]]
    _, y = control.forced_response(dsys, np.arange(len(t)) * dt, u)
    print(f"\nelevator doublet +/-{amp}:")
    for row, (name, scale, unit) in enumerate([("Vt", 1, "ft/s"), ("Alpha", 57.3, "deg"),
                                               ("Theta", 57.3, "deg"), ("Q", 57.3, "deg/s")]):
        dnl = (nl[name] - x0[name]).to_numpy() * scale
        dlin = y[row] * scale
        err = np.max(np.abs(dnl - dlin)) / np.max(np.abs(dnl))
        print(f"   {name:6s} peak {np.max(np.abs(dnl)):8.3f} {unit:6s} linear-model error {100 * err:5.1f} % of peak")
        ax = axes[row, col]
        ax.plot(t, dnl, label="JSBSim (nonlinear)")
        ax.plot(t, dlin, "--", label="linear model")
        ax.set_ylabel(f"d{name} [{unit}]")
        ax.grid(alpha=0.3)
    axes[0, col].set_title(f"glider, elevator doublet +/-{amp}")
axes[0, 0].legend(fontsize="small")
for a in axes[-1]:
    a.set_xlabel("time [s]")
fig.tight_layout()
save(fig, "06_linear_models_modes", "linear_vs_nonlinear", show=args.show)

fig = plt.figure(figsize=(8, 6))
control.bode_plot(sys[3, 0], omega=np.logspace(-2, 2, 400), dB=True, deg=True)
plt.suptitle("glider q / elevator-cmd (linear model, 14.6 m/s)")
save(plt.gcf(), "06_linear_models_modes", "bode_q_de", show=args.show)

# --- Part 2: C172 actuator hysteresis -----------------------------------------
print("\nC172 elevator actuator (hysteresis_width 0.05 rad):")
runs = {}
for amp in (0.02, 0.15):
    fdm = make_fdm("c172x")
    initialize(fdm, {"lat-geod-deg": 33, "h-sl-ft": 4000, "vc-kts": 100})
    trim(fdm, "full")
    de0, pos0 = fdm["fcs/elevator-cmd-norm"], fdm["fcs/elevator-pos-rad"]

    def pilot(f, t, amp=amp, de0=de0):
        f["fcs/elevator-cmd-norm"] = de0 + doublet(t, 1.0, 1.0, -amp)

    df = run(fdm, 8.0, {"elev_cmd": "fcs/elevator-cmd-norm", "elev_pos_rad": "fcs/elevator-pos-rad",
                        "q_rps": "velocities/q-rad_sec"}, callback=pilot)
    runs[f"+/-{amp}"] = df
    print(f"   doublet +/-{amp}: surface moved {df.elev_pos_rad.max() - df.elev_pos_rad.min():.4f} rad, "
          f"final offset from trim {df.elev_pos_rad.iloc[-1] - pos0:+.4f} rad")
fig, _ = timehistory(runs, ["elev_cmd", "elev_pos_rad", "q_rps"], title="C172 elevator: hysteresis in the actuator")
save(fig, "06_linear_models_modes", "c172_hysteresis", show=args.show)
