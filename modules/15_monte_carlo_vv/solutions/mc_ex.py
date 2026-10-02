"""Module 15 exercises A-C (solution).  A: stratified Monte Carlo on turbulence.

Instead of letting turbulence fall where the random draw puts it, fix it to
bins and run the same number of cases in each.  That gives a pass rate (with
a confidence interval) per *condition*, which is what a requirement written
as "in light turbulence, ..." needs, and what a single overall pass rate hides.
B: runs needed for a P(fail) claim.  C: gyro-bias margin to failure.

    python modules/15_monte_carlo_vv/solutions/mc_ex.py [--show] [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from gnclab.cli import parse_args
from gnclab.montecarlo import (REQUIREMENTS, evaluate, pass_rate_ci, run_cases, runs_for_confidence, sample_cases,
                               unusual)
from gnclab.plotting import save

BINS = [(0, 10), (10, 15), (15, 20), (20, 25), (25, 35)]


def main():
    args = parse_args(__doc__)
    per_bin = 4 if args.fast else 60
    frames = []
    for i, (lo, hi) in enumerate(BINS):
        c = sample_cases(per_bin, seed=100 + i, include_nominal=False)
        c["turb_w20_fps"] = np.random.default_rng(i).uniform(lo, hi, per_bin)
        c["bin"] = f"{lo}-{hi}"
        frames.append(c)
    cases = pd.concat(frames, ignore_index=True)
    res = evaluate(run_cases(cases.drop(columns="bin"), workers=2 if args.fast else None))
    res["bin"] = cases["bin"]

    print(f"\n{'W20 [ft/s]':>11s}  {'MC-3 pass':>10s}  {'95 % CI':>15s}  {'all reqs':>9s}  "
          f"{'|alt err| p95 [ft]':>18s}  {'beta max p95':>12s}")
    rows = []
    for b, g in res.groupby("bin", sort=False):
        n = len(g)
        p3, lo3, hi3 = pass_rate_ci(int(g["MC-3"].sum()), n)
        pa = g["pass"].mean()
        e95 = g.alt_err_end_ft.abs().quantile(0.95)
        b95 = g.beta_max_deg.quantile(0.95)
        rows.append((b, p3, lo3, hi3, e95))
        print(f"{b:>11s}  {100 * p3:9.1f}%  [{100 * lo3:5.1f}, {100 * hi3:5.1f}] %  {100 * pa:8.1f}%  "
              f"{e95:18.2f}  {b95:12.2f}")
    print("\nFailures per requirement:", {k: int((~res[k]).sum()) for k in REQUIREMENTS if (~res[k]).any()})
    calm = res[(~res["pass"]) & (res.turb_w20_fps < 15)]
    print(f"Failures in turbulence below 15 ft/s ({len(calm)}): what is unusual about them?")
    for i, r in calm.iterrows():
        failed = ",".join(k for k in REQUIREMENTS if not r[k])
        print(f"  case {i} [{failed}] W20 {r.turb_w20_fps:.1f}: {unusual(r)}")

    # Exercise B
    print("\nB: P(fail) < 0.5 % at 95 % confidence")
    print(f"   zero failures: {runs_for_confidence(0.005)} runs")
    for n in range(runs_for_confidence(0.005), 5000):
        if 1 - pass_rate_ci(n - 1, n, conf=0.90)[1] < 0.005:     # one-sided 95 % = two-sided 90 % bound
            print(f"   one failure  : {n} runs")
            break

    # Exercise C: gyro biases (sensors/gyro-bias-*-rad_sec, added to gnc_sensors.xml)
    print("\nC: gyro bias, one axis at a time, nominal vehicle, calm air")
    nominal = sample_cases(1).iloc[0].to_dict()
    biases = (0.0, 0.05, 0.2, 0.5) if args.fast else (0.0, 0.02, 0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0)
    rows_c = [dict(nominal, **{f"gyro_bias_{ax}": b}) for ax in "pqr" for b in biases]
    cases_c = pd.DataFrame(rows_c).fillna(0.0)
    rc = evaluate(run_cases(cases_c, workers=2 if args.fast else None, progress=False))
    for ax in "pqr":
        others = [f"gyro_bias_{o}" for o in "pqr" if o != ax]
        g = rc[rc[others].eq(0).all(axis=1)]                # this axis' sweep (incl. the zero-bias run)
        fail = g[~g["pass"]]
        first = (f"first failure at {fail[f'gyro_bias_{ax}'].iloc[0]:.2f} rad/s "
                 f"({','.join(k for k in REQUIREMENTS if not fail.iloc[0][k])})" if len(fail) else "no failure in sweep")
        spread = g.alt_err_end_ft.max() - g.alt_err_end_ft.min()
        print(f"   {ax}: {first}; alt error spread across the sweep {spread:.2f} ft")

    fig, ax = plt.subplots(figsize=(7, 4))
    for b, p3, lo3, hi3, _ in rows:
        ax.errorbar(b, 100 * p3, yerr=[[100 * (p3 - lo3)], [100 * (hi3 - p3)]], fmt="o", capsize=5)
    ax.set(xlabel="turbulence W20 bin [ft/s]", ylabel="MC-3 pass rate [%]", ylim=(0, 105),
           title="Stratified Monte Carlo: MC-3 pass rate vs turbulence (95 % CI)")
    ax.grid(alpha=0.3)
    save(fig, "15_monte_carlo_vv", "stratified", show=args.show)


if __name__ == "__main__":
    main()
