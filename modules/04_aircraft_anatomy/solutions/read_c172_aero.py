"""Module 04 - read an aircraft's aerodynamic build-up like a reviewer would.

1. Parse aircraft/c172x/c172x.xml and list every aerodynamic function per axis
   (name + description), so you see the structure of the model.
2. Trim the airplane and evaluate every function at that flight condition.
   Each function is also a *property* holding its current value (a force in
   lbf or a moment in lbf*ft), so dividing by qbar*S (and b or cbar) gives its
   coefficient contribution.  This "contribution table" is the quickest way to
   sanity-check an aero model or to find the term that dominates a behaviour.

    python modules/04_aircraft_anatomy/solutions/read_c172_aero.py [aircraft]
"""

import os
import xml.etree.ElementTree as ET

from gnclab.cli import parse_args

args = parse_args(__doc__, extra=lambda p: p.add_argument("aircraft", nargs="?", default="c172x"))

import jsbsim  # noqa: E402

from gnclab import AIRCRAFT_DIR, initialize, make_fdm  # noqa: E402
from gnclab.trim import trim  # noqa: E402

name = args.aircraft
path = AIRCRAFT_DIR / name / f"{name}.xml"
if not path.exists():
    path = os.path.join(jsbsim.get_default_root_dir(), "aircraft", name, f"{name}.xml")
root = ET.parse(path).getroot()

fdm = make_fdm(name)
initialize(fdm, {"lat-geod-deg": 33, "h-sl-ft": 4000, "vc-kts": 100 if name == "c172x" else 30})
s = trim(fdm, "longitudinal")
qS = fdm["aero/qbar-psf"] * fdm["metrics/Sw-sqft"]
b, cbar = fdm["metrics/bw-ft"], fdm["metrics/cbarw-ft"]
print(f"{name}: trimmed at {s['V_kts']:.0f} KTAS, alpha {s['alpha_deg']:.2f} deg, qbar {fdm['aero/qbar-psf']:.1f} psf\n")

ref = {"DRAG": qS, "SIDE": qS, "LIFT": qS, "X": qS, "Y": qS, "Z": qS,
       "ROLL": qS * b, "PITCH": qS * cbar, "YAW": qS * b}
for axis in root.find("aerodynamics").findall("axis"):
    ax = axis.get("name")
    total = 0.0
    print(f"== {ax} axis (coefficient contributions at trim)")
    for fn in axis.findall("function"):
        prop = fn.get("name")
        desc = (fn.findtext("description") or "").strip().replace("\n", " ")
        try:
            coeff = fdm[prop] / ref[ax]
        except KeyError:
            coeff = float("nan")
        total += coeff
        print(f"   {coeff:+9.4f}  {prop:<34s} {desc[:60]}")
    print(f"   {total:+9.4f}  = total C{ax.lower()}\n")
