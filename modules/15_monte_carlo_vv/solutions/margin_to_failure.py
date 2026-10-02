"""Module 15 - margin to failure: where does each driver break the requirements?

A 300-run Monte Carlo at 98 % pass says little about the cases it didn't
draw.  So, for the drivers it found, sweep ONE parameter at a time (all
others nominal, calm air except where noted) until a requirement fails, and
express the cliff in standard deviations of the dispersion.  A design with
cliffs at 2 sigma is fragile; at 5+ sigma it is robust.

    python modules/15_monte_carlo_vv/solutions/margin_to_failure.py [--show] [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from gnclab.cli import parse_args
from gnclab.montecarlo import DEFAULT_DISPERSIONS, Normal, REQUIREMENTS, evaluate, run_cases, sample_cases
from gnclab.plotting import save

# parameter -> sweep values, from nominal toward failure.  sigma is only
# quoted for Normal dispersions (uniform ones have no sigma).
SWEEPS = {
    "scale_Clda": np.linspace(1.0, 0.1, 19),        # aileron power
    "scale_Cnb": np.linspace(1.0, -0.2, 13),        # weathercock stability
    "scale_Cmde": np.linspace(1.0, 0.2, 17),        # elevator power
    "payload_x_m": np.linspace(0.40, 1.40, 21),     # 2 kg payload moved aft: CG -> neutral point
    "scale_thrust": np.linspace(1.0, 0.3, 15),      # weak motor / prop
    "turb_w20_fps": np.linspace(0.0, 100.0, 21),    # turbulence intensity
    "act_rate_dps": np.array([250, 150, 100, 70, 50, 35, 25, 18, 12, 8, 5]),
}


def main():
    args = parse_args(__doc__)
    nominal = sample_cases(1).iloc[0].to_dict()       # case 0 = nominal, calm
    rows = []
    for p, values in SWEEPS.items():
        vals = values[::3] if args.fast else values
        for v in vals:
            c = dict(nominal, **{p: v}, sweep=p)
            if p == "payload_x_m":
                c["payload_kg"] = 2.0
            rows.append(c)
    cases = pd.DataFrame(rows)
    res = evaluate(run_cases(cases.drop(columns="sweep"), workers=2 if args.fast else None))
    res["sweep"] = cases["sweep"]
    rids = list(REQUIREMENTS)

    print(f"\n{'parameter':14s} {'nominal':>8s} {'first failure':>14s} {'sigma':>7s}  failed requirements")
    fig, axes = plt.subplots(len(SWEEPS), 1, figsize=(9, 2.0 * len(SWEEPS)))
    for ax, (p, _) in zip(axes, SWEEPS.items()):
        r = res[res.sweep == p]
        nom = DEFAULT_DISPERSIONS[p].nominal
        fail = r[~r["pass"]]
        if len(fail):
            v = fail[p].iloc[0]
            d = DEFAULT_DISPERSIONS[p]
            sig = f"{abs(v - nom) / d.sigma:5.1f}" if isinstance(d, Normal) and p != "payload_x_m" else "  -"
            which = ",".join(k for k in rids if not fail[k].iloc[0]) or fail.status.iloc[0]
            print(f"{p:14s} {nom:8.3f} {v:14.3f} {sig:>7s}  {which}  (status {fail.status.iloc[0]})")
        else:
            print(f"{p:14s} {nom:8.3f} {'none in sweep':>14s}")
        # normalized margin of the tightest requirement: metric / limit
        for rid in ("MC-1", "MC-3", "MC-4", "MC-6", "MC-9"):
            metric, op, lim, _ = REQUIREMENTS[rid]
            x = r[metric].abs() if op == "abs<=" else r[metric]
            ratio = lim / x if op == ">=" else x / lim
            ax.plot(r[p], ratio, marker=".", label=rid)
        ax.axhline(1.0, color="r", ls="--")
        ax.set_ylabel(p, fontsize=8)
        ax.set_ylim(0, 2)
        ax.grid(alpha=0.3)
    axes[0].legend(ncol=5, fontsize="small")
    axes[0].set_title("metric / limit along each one-at-a-time sweep (> 1 = requirement violated)")
    fig.tight_layout()
    save(fig, "15_monte_carlo_vv", "margin_to_failure", show=args.show)


if __name__ == "__main__":
    main()
