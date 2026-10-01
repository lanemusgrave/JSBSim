"""Module 07 exercise: review a colleague's model - find the three bugs.

This script copies aircraft/gnc_trainer to aircraft/gnc_trainer_review with
THREE deliberate mistakes of the kind that really happen when someone types an
aero/mass model into JSBSim XML.  It then trims and linearizes both models
and prints the comparison you'd do in a model review.

Your job:
  1. Run it.  From the trim and mode comparison alone, form a hypothesis for
     each discrepancy (which coefficient / property / sign?).
  2. Open aircraft/gnc_trainer_review/gnc_trainer_review.xml and diff it
     against aircraft/gnc_trainer/gnc_trainer.xml to confirm.
     (VS Code: right-click one file -> "Select for Compare", then the other.)
  3. Write each bug, the symptom that gave it away, and the review check that
     would catch it automatically, in a comment at the bottom.

Do NOT look at the BUGS list below until you've done steps 1-2.

    python modules/07_build_uas_model/exercises/bug_hunt.py
"""

import shutil

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import AIRCRAFT_DIR, initialize, make_fdm  # noqa: E402
from gnclab.linear import linearize_fd, modes  # noqa: E402
from gnclab.trim import TrimError, trim  # noqa: E402

SRC, DST = AIRCRAFT_DIR / "gnc_trainer", AIRCRAFT_DIR / "gnc_trainer_review"

BUGS = [  # (find, replace) on the XML text - spoilers!
    ('<mass_balance negated_crossproduct_inertia="false">', "<mass_balance>"),
    ("<property> aero/bi2vel </property>\n          <property> velocities/p-aero-rad_sec </property>\n"
     "          <value> -0.51 </value>",
     "<property> velocities/p-aero-rad_sec </property>\n          <value> -0.51 </value>"),
    ('<chord unit="M"> 0.18994 </chord>', '<chord unit="FT"> 0.18994 </chord>'),
]

if DST.exists():
    shutil.rmtree(DST)
shutil.copytree(SRC, DST)
xml = (SRC / "gnc_trainer.xml").read_text().replace('name="gnc_trainer"', 'name="gnc_trainer_review"')
for find, repl in BUGS:
    assert find in xml, f"bug template not found: {find[:40]}"
    xml = xml.replace(find, repl, 1)
(DST / "gnc_trainer.xml").unlink()
(DST / "gnc_trainer_review.xml").write_text(xml)

INPUTS = ["fcs/elevator-cmd-norm", "fcs/aileron-cmd-norm", "fcs/rudder-cmd-norm"]
print(f"{'':16s}{'gnc_trainer':>28s}{'gnc_trainer_review':>28s}")
results = {}
for name in ("gnc_trainer", "gnc_trainer_review"):
    fdm = make_fdm(name)
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048})
    try:
        trim(fdm, "full")
    except TrimError as e:
        print(name, "TRIM FAILED", e)
        continue
    m = modes(linearize_fd(fdm, INPUTS).A)
    osc = m[m.eig_imag > 0].sort_values("wn", ascending=False)
    real = m[m.eig_imag == 0].sort_values("wn", ascending=False)
    results[name] = {
        "alpha [deg]": fdm["aero/alpha-deg"],
        "elevator [deg]": np.degrees(fdm["fcs/elevator-pos-rad"]),
        "aileron [deg]": np.degrees(fdm["fcs/aileron-pos-rad"]),
        "short period": f"wn {osc.iloc[0].wn:6.2f} z {osc.iloc[0].zeta:5.2f}",
        "Dutch roll": f"wn {osc.iloc[1].wn:6.2f} z {osc.iloc[1].zeta:5.2f}",
        "phugoid": f"wn {osc.iloc[2].wn:6.2f} z {osc.iloc[2].zeta:5.2f}",
        "roll mode": f"lambda {real.iloc[0].eig_real:8.2f}",
        "spiral": f"lambda {real.iloc[-1].eig_real:8.3f}",
    }
for key in results["gnc_trainer"]:
    a, b = (results.get(n, {}).get(key, "-") for n in ("gnc_trainer", "gnc_trainer_review"))
    fa = f"{a:28.3f}" if isinstance(a, float) else f"{a:>28s}"
    fb = f"{b:28.3f}" if isinstance(b, float) else f"{b:>28s}"
    print(f"{key:16s}{fa}{fb}")

# Your findings:
# Bug 1:
# Bug 2:
# Bug 3:
