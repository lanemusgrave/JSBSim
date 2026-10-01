"""Module 03 - run the test-card script, then analyze its CSV log with pandas.

This mirrors a real flight-test workflow: the *card* (XML script) defines the
maneuvers, the *log* (CSV) is the data product, and the analysis script turns
the log into numbers for the test report.

    python modules/03_scripts_events_output/solutions/analyze_testcard.py [--show]
"""

import gc
from pathlib import Path

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from gnclab import OUTPUT_DIR, load_script  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402

CARD = Path(__file__).with_name("c172_testcard.xml")
LOG = OUTPUT_DIR / "c172_testcard.csv"

# --- 1. Fly the card -------------------------------------------------------
fdm = load_script(CARD, output=True)   # output=True: honour the script's <output>
while fdm.run():                        # run() returns False at the script's end time
    pass
del fdm                                 # destroying the executive flushes/closes the CSV
gc.collect()

# --- 2. Load the log ---------------------------------------------------------
log = pd.read_csv(LOG).set_index("Time")
print(f"log: {len(log)} rows x {len(log.columns)} columns, {log.index[-1]:.1f} s at "
      f"{1 / np.median(np.diff(log.index)):.0f} Hz -> {LOG.relative_to(OUTPUT_DIR.parent)}")
for c in ("p_rps", "q_rps", "r_rps"):
    log[c.replace("rps", "dps")] = np.degrees(log[c])

# --- 3. Segment and reduce ---------------------------------------------------
CARD_POINTS = {  # name: (start, end) [s]
    "TP1 elevator doublet": (10, 25),
    "TP2 aileron doublet": (30, 45),
    "TP3 rudder doublet": (50, 65),
    "TP4 full power": (70, 90),
}


def zero_crossing_period(t, y):
    """Mean period from successive upward zero crossings of a (detrended) signal."""
    y = y - np.mean(y)
    up = np.where((y[:-1] < 0) & (y[1:] >= 0))[0]
    return float(np.mean(np.diff(t[up]))) if len(up) >= 2 else np.nan


rows = []
for name, (t0, t1) in CARD_POINTS.items():
    seg = log.loc[t0:t1]
    base = log.loc[t0 - 1:t0]  # 1 s of pre-maneuver data = the reference condition
    row = {"test point": name, "KCAS at start": base.kcas.mean()}
    if name.startswith("TP1"):
        row["peak |q| [deg/s]"] = seg.q_dps.abs().max()
        row["peak dNz [g]"] = (seg.nz_g - base.nz_g.mean()).abs().max()
        row["alt change [ft]"] = seg.alt_ft.iloc[-1] - base.alt_ft.mean()
    elif name.startswith("TP2"):
        row["peak |p| [deg/s]"] = seg.p_dps.abs().max()
        row["peak |phi| [deg]"] = seg.phi_deg.abs().max()
    elif name.startswith("TP3"):
        row["peak |beta| [deg]"] = seg.beta_deg.abs().max()
        after = seg.loc[t0 + 2:]  # free response after the doublet
        row["Dutch roll period [s]"] = zero_crossing_period(after.index.to_numpy(), after.r_dps.to_numpy())
    else:
        late = seg.loc[t1 - 10:]
        row["climb rate [ft/min]"] = np.polyfit(late.index, late.alt_ft, 1)[0] * 60
    rows.append(row)

report = pd.DataFrame(rows).set_index("test point")
print("\nTest report\n" + report.round(2).to_string())
report.round(3).to_csv(OUTPUT_DIR / "03_scripts_events_output_report.csv")

fig, _ = timehistory(log, ["de_cmd", "q_dps", "nz_g", "da_cmd", "p_dps", "phi_deg",
                           "dr_cmd", "beta_deg", "r_dps", "throttle", "alt_ft"],
                     title="C172 test card (CSV log)")
for ax in fig.axes:
    for t0, _ in CARD_POINTS.values():
        ax.axvline(t0, color="k", alpha=0.15)
save(fig, "03_scripts_events_output", "testcard", show=args.show)
