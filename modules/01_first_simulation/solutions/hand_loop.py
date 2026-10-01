"""Module 01 - the JSBSim loop written out by hand (no gnclab helpers).

Everything gnclab.make_fdm / initialize / run do, spelled out.  Read this file
top to bottom; it is the whole JSBSim Python API you need for most work.

    python modules/01_first_simulation/solutions/hand_loop.py [--show] [--fast]
"""

import argparse

import matplotlib

parser = argparse.ArgumentParser()
parser.add_argument("--show", action="store_true")
parser.add_argument("--fast", action="store_true")
args = parser.parse_args()
if not args.show:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import jsbsim  # noqa: E402

# 1. Create the executive.  The argument is the JSBSim "root" folder holding
#    aircraft/, engine/, systems/.  get_default_root_dir() = the data that
#    ships inside the pip package.
jsbsim.FGJSBBase().debug_lvl = 0          # quiet: no banner / model reports
fdm = jsbsim.FGFDMExec(jsbsim.get_default_root_dir())

# 2. Load an aircraft by folder name: aircraft/c172x/c172x.xml
fdm.load_model("c172x")
fdm.disable_output()                      # don't write the model's CSV file
print("Loaded", fdm.get_model_name(), "| dt =", fdm.get_delta_t(), "s")

# 3. Initial conditions are *properties* under ic/.  Units are in the name.
#    ORDER MATTERS: set position first, then speed.  Writing ic/lat... after
#    ic/vc-kts keeps the *true* airspeed and silently changes the calibrated
#    airspeed (try swapping the blocks and watch the KCAS trace start at 106).
fdm["ic/lat-geod-deg"] = 33.67            # somewhere over Orange County
fdm["ic/long-gc-deg"] = -117.99
fdm["ic/h-sl-ft"] = 4000.0                # altitude above sea level
fdm["ic/vc-kts"] = 100.0                  # calibrated airspeed
fdm["ic/psi-true-deg"] = 90.0             # heading east
fdm["ic/gamma-deg"] = 0.0                 # flight path angle
fdm.run_ic()                              # apply the ICs -> sim time = 0

# 4. Engine running, then TRIM: find throttle/elevator/aileron/rudder and
#    attitude for steady, wings-level flight at these ICs.  (Module 05 covers
#    trim properly.  Try commenting the do_trim line out and see what happens.)
fdm["propulsion/set-running"] = -1        # -1 = all engines
fdm.do_trim(1)                            # 1 = "full" trim
de_trim = fdm["fcs/elevator-cmd-norm"]
print(f"trimmed: throttle {fdm['fcs/throttle-cmd-norm[0]']:.3f}, "
      f"pitch trim {fdm['fcs/pitch-trim-cmd-norm']:.3f}, alpha {fdm['aero/alpha-deg']:.2f} deg")

# 5. The loop: each fdm.run() advances one dt through every model.
#    At t = 5 s we "pull" for half a second: elevator command -0.1
#    (normalized -1..1; NEGATIVE = stick aft = nose up), then let go.
t_end = 40.0 if args.fast else 120.0
log = {"t": [], "alt": [], "kcas": [], "theta": [], "phi": [], "q": []}
while fdm.get_sim_time() <= t_end:
    t = fdm.get_sim_time()
    fdm["fcs/elevator-cmd-norm"] = de_trim - 0.1 if 5.0 <= t < 5.5 else de_trim
    log["t"].append(t)
    log["alt"].append(fdm["position/h-sl-ft"])
    log["kcas"].append(fdm["velocities/vc-kts"])
    log["theta"].append(fdm["attitude/theta-deg"])
    log["phi"].append(fdm["attitude/phi-deg"])
    log["q"].append(fdm["velocities/q-rad_sec"] * 57.2958)
    fdm.run()

print(f"after {log['t'][-1]:.0f} s: {log['alt'][-1]:.0f} ft, {log['kcas'][-1]:.1f} KCAS")

# 6. Plot
fig, ax = plt.subplots(5, 1, sharex=True, figsize=(9, 10))
for a, key, label in zip(ax, ["alt", "kcas", "theta", "q", "phi"],
                         ["altitude [ft]", "KCAS", "theta [deg]", "q [deg/s]", "phi [deg]"]):
    a.plot(log["t"], log[key])
    a.set_ylabel(label)
    a.grid(alpha=0.3)
ax[-1].set_xlabel("time [s]")
fig.suptitle("C172 trimmed, 0.5 s elevator pulse, then hands off")
fig.tight_layout()

from gnclab.plotting import outdir  # noqa: E402  (just for the output folder)

path = outdir("01_first_simulation") / "hand_loop.png"
fig.savefig(path, dpi=110)
print("saved", path)
if args.show:
    plt.show()
