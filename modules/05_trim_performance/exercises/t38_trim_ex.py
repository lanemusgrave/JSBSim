"""Module 05 exercise: the T-38's level-flight envelope at 15,000 ft.

TODO 1: trim the T38 in level flight (mode "longitudinal") at 150, 165 and
        180..450 KCAS in 30 kt steps; tabulate alpha, throttle, Mach and drag
        (drag = forces/fwx-aero-lbs).
TODO 2: find the maximum level-flight speed at 15,000 ft to +/- 1 kt by
        BISECTION on KCAS (a trim that fails = too fast).  Hint: wrap trim()
        in try/except gnclab.trim.TrimError.
TODO 3: trim a 2 g pull-up at 350 KCAS (mode "pullup", set ic/targetNlf = 2
        before run_ic) and compare the pitch rate with the kinematic
        estimate q = g (n - 1) / V.
TODO 4: (thinking) trim alpha falls with speed while throttle rises - sketch
        the drag curve from your table.  Where is the T-38's minimum-drag
        speed at this altitude?

    python modules/05_trim_performance/exercises/t38_trim_ex.py
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

from gnclab import initialize, make_fdm  # noqa: E402
from gnclab.trim import TrimError, trim  # noqa: E402


def trimmed_t38(kcas, mode="longitudinal", **extra_ic):
    """Return a trimmed FGFDMExec, or None if the trim fails."""
    fdm = make_fdm("T38")
    initialize(fdm, {"lat-geod-deg": 29.5, "h-sl-ft": 15000.0, "vc-kts": kcas, **extra_ic})
    try:
        trim(fdm, mode)
    except TrimError:
        return None
    return fdm


# TODO 1

# TODO 2

# TODO 3
