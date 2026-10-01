"""Module 01 exercise - solution: T-38 aileron pulse with a hand-written loop."""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import jsbsim  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

from gnclab.plotting import outdir  # noqa: E402

jsbsim.FGJSBBase().debug_lvl = 0

# TODO 1
fdm = jsbsim.FGFDMExec(jsbsim.get_default_root_dir())
fdm.load_model("T38")
fdm.disable_output()
fdm["ic/lat-geod-deg"] = 29.5   # position first ...
fdm["ic/long-gc-deg"] = -98.6
fdm["ic/h-sl-ft"] = 10000.0
fdm["ic/vc-kts"] = 300.0        # ... then speed
fdm["ic/psi-true-deg"] = 0.0
fdm.run_ic()
fdm["propulsion/set-running"] = -1
fdm.do_trim(1)
da_trim = fdm["fcs/aileron-cmd-norm"]
print(f"trimmed: alpha {fdm['aero/alpha-deg']:.2f} deg, throttle {fdm['fcs/throttle-cmd-norm[0]']:.3f}, "
      f"KTAS {fdm['velocities/vtrue-kts']:.0f}")

# TODO 2-3
log = {"t": [], "phi": [], "p": [], "beta": [], "r": [], "thrust": []}
t_end = 8.0 if args.fast else 15.0
while fdm.get_sim_time() <= t_end:
    t = fdm.get_sim_time()
    fdm["fcs/aileron-cmd-norm"] = da_trim + (0.2 if 2.0 <= t < 3.0 else 0.0)
    log["t"].append(t)
    log["phi"].append(fdm["attitude/phi-deg"])
    log["p"].append(fdm["velocities/p-rad_sec"] * 57.2958)
    log["beta"].append(fdm["aero/beta-deg"])
    log["r"].append(fdm["velocities/r-rad_sec"] * 57.2958)
    log["thrust"].append(fdm["propulsion/engine/thrust-lbs"] + fdm["propulsion/engine[1]/thrust-lbs"])
    fdm.run()

# TODO 4
keys = ["phi", "p", "beta", "r", "thrust"]
labels = ["phi [deg]", "p [deg/s]", "beta [deg]", "r [deg/s]", "thrust [lbf]"]
fig, ax = plt.subplots(len(keys), 1, sharex=True, figsize=(9, 9))
for a, k, lab in zip(ax, keys, labels):
    a.plot(log["t"], log[k])
    a.set_ylabel(lab)
    a.grid(alpha=0.3)
ax[-1].set_xlabel("time [s]")
fig.suptitle("T-38, 10k ft / 300 KCAS: 1 s aileron pulse (+0.2)")
fig.tight_layout()
path = outdir("01_first_simulation") / "t38_roll.png"
fig.savefig(path, dpi=110)
print(f"final bank {log['phi'][-1]:.1f} deg; saved {path}")
if args.show:
    plt.show()

# TODO 5: ~30-38 deg of bank remains (it drifts slowly).  Roll *rate* is damped (Lp < 0: the
# roll subsidence mode, time constant ~0.2-0.5 s, kills p quickly) but there
# is no aerodynamic moment that depends on bank angle itself: phi is a pure
# integral of p.  Only the (slow, often neutral/unstable) spiral mode acts
# on phi, through sideslip and yaw rate in the resulting turn.  Rolling back
# to wings level is the pilot's (or wing leveler's, Module 10) job.
