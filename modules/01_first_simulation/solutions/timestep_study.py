"""Module 01 - how much does the integration time step matter?

Run the same elevator pulse at several dt and compare with a very fine
reference.  Also time how fast each runs.

    python modules/01_first_simulation/solutions/timestep_study.py [--show]
"""

import time

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402
from gnclab.signals import pulse  # noqa: E402
from gnclab.trim import trim  # noqa: E402

DURATION = 10.0 if args.fast else 20.0
RATES_HZ = [30, 60, 120, 480] if args.fast else [20, 30, 60, 120, 480, 1200]


def elevator_pulse(rate_hz):
    fdm = make_fdm("c172x", dt=1.0 / rate_hz)
    initialize(fdm, {"h-sl-ft": 4000, "vc-kts": 100})
    trim(fdm, "longitudinal")
    de0 = fdm["fcs/elevator-cmd-norm"]

    def pilot(f, t):
        f["fcs/elevator-cmd-norm"] = de0 + pulse(t, 1.0, 1.0, -0.2)

    t0 = time.perf_counter()
    df = run(fdm, DURATION, {"q_rps": "velocities/q-rad_sec",
                             "theta_deg": "attitude/theta-deg",
                             "alt_ft": "position/h-sl-ft"}, callback=pilot)
    wall = time.perf_counter() - t0
    return df, wall


runs, walls = {}, {}
for hz in RATES_HZ:
    runs[f"{hz} Hz"], walls[hz] = elevator_pulse(hz)

ref = runs[f"{RATES_HZ[-1]} Hz"]
print(f"{'rate':>8s} {'dt [ms]':>8s} {'max |q err| [deg/s]':>20s} {'alt err @end [ft]':>18s} {'x real time':>12s}")
for hz in RATES_HZ:
    df = runs[f"{hz} Hz"]
    q_ref = np.interp(df.index, ref.index, ref.q_rps)
    err_q = np.max(np.abs(df.q_rps - q_ref)) * 57.2958
    err_h = df.alt_ft.iloc[-1] - ref.alt_ft.iloc[-1]
    print(f"{hz:8d} {1000/hz:8.2f} {err_q:20.4f} {err_h:18.3f} {DURATION / walls[hz]:12.0f}")

fig, _ = timehistory(runs, ["q_rps", "theta_deg", "alt_ft"],
                     title="Same elevator pulse at different integration rates")
save(fig, "01_first_simulation", "timestep_study", show=args.show)
