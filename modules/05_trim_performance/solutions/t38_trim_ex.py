"""Module 05 exercise - solution: T-38 envelope at 15,000 ft."""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import pandas as pd  # noqa: E402

from gnclab import initialize, make_fdm  # noqa: E402
from gnclab.trim import TrimError, trim  # noqa: E402
from gnclab.units import G0_FPS2  # noqa: E402


def trimmed_t38(kcas, mode="longitudinal", **extra_ic):
    fdm = make_fdm("T38")
    initialize(fdm, {"lat-geod-deg": 29.5, "h-sl-ft": 15000.0, "vc-kts": kcas, **extra_ic})
    try:
        trim(fdm, mode)
    except TrimError:
        return None
    return fdm


# TODO 1
rows = []
for kcas in [150, 165] + list(range(180, 451, 90 if args.fast else 30)):
    fdm = trimmed_t38(float(kcas))
    rows.append({"KCAS": kcas, "alpha_deg": fdm["aero/alpha-deg"], "throttle": fdm["fcs/throttle-cmd-norm[0]"],
                 "mach": fdm["velocities/mach"], "drag_lbf": fdm["forces/fwx-aero-lbs"]})
table = pd.DataFrame(rows)
print(table.round(3).to_string(index=False))
print(f"minimum drag of the points tested at {table.KCAS[table.drag_lbf.idxmin()]} KCAS")

# TODO 2: bisection between a speed that trims and one that doesn't
lo, hi = 450.0, 500.0
assert trimmed_t38(lo) is not None and trimmed_t38(hi) is None
while hi - lo > 1.0:
    mid = 0.5 * (lo + hi)
    if trimmed_t38(mid) is None:
        hi = mid
    else:
        lo = mid
fdm = trimmed_t38(lo)
print(f"\nmax level speed at 15,000 ft: {lo:.0f} KCAS (Mach {fdm['velocities/mach']:.3f}), "
      f"throttle {fdm['fcs/throttle-cmd-norm[0]']:.3f}")

# TODO 3
fdm = trimmed_t38(350.0, mode="pullup", targetNlf=2.0)
V = fdm["velocities/vtrue-fps"]
q_est = G0_FPS2 * (2.0 - 1.0) / V
print(f"\n2 g pull-up at 350 KCAS: Nz {fdm['accelerations/Nz']:.3f}, alpha {fdm['aero/alpha-deg']:.2f} deg, "
      f"q {fdm['velocities/q-rad_sec']:.4f} rad/s vs g(n-1)/V = {q_est:.4f} rad/s")

# TODO 4: drag falls from 150 to ~180 KCAS and rises after it, so the
# minimum-drag speed at this weight/altitude is about 175-185 KCAS (alpha ~12-13
# deg).  Below it you are on the "back side of the drag curve": slowing down
# takes MORE thrust - the regime of a power-approach, where speed is held with
# throttle and path with pitch.  Trims below ~150 KCAS fail (near stall alpha).
