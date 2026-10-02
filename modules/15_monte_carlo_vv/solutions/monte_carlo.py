"""Module 15 - requirements Monte Carlo for the gnc_trainer autopilot.

Samples DEFAULT_DISPERSIONS (mass/CG, 11 aero/propulsion multipliers, wind,
turbulence, actuator bandwidth/rate, sensor-noise seed), flies the test card
in gnclab.montecarlo.requirements_flight for every case in parallel worker
processes, then reports:
  * pass rate per requirement with 95 % Clopper-Pearson intervals
  * the drivers of each metric (Spearman rank correlation)
  * pass-rate convergence vs number of runs
  * a flight-readiness summary (outputs/15_monte_carlo_vv/summary.md)

    python modules/15_monte_carlo_vv/solutions/monte_carlo.py [--show] [--fast] [-n 300] [--workers 4]

Windows note: worker processes are *spawned*, and each one re-imports this
file.  Everything that does work lives in main(), behind the
if __name__ == "__main__": guard at the bottom.  Delete the guard and every
worker would start its own Monte Carlo.
"""

import time

import matplotlib.pyplot as plt
import numpy as np

from gnclab.cli import parse_args
from gnclab.montecarlo import (DEFAULT_DISPERSIONS, REQUIREMENTS, evaluate, pass_rate_ci, run_cases,
                               runs_for_confidence, sample_cases, sensitivity)
from gnclab.plotting import outdir, save

MODULE = "15_monte_carlo_vv"


def requirement_table(res):
    n = len(res)
    lines = [f"{'req':5s} {'pass':>9s} {'rate':>7s}  {'95 % CI':>15s}  {'worst':>8s} {'limit':>7s}  description"]
    for rid, (metric, op, lim, text) in REQUIREMENTS.items():
        k = int(res[rid].sum())
        p, lo, hi = pass_rate_ci(k, n)
        x = res[metric]
        worst = x.min() if op == ">=" else (x.abs().max() if op == "abs<=" else x.max())
        lines.append(f"{rid:5s} {k:4d}/{n:<4d} {100 * p:6.1f}%  [{100 * lo:5.1f}, {100 * hi:5.1f}] %  "
                     f"{worst:8.2f} {lim:7.1f}  {text}")
    k = int(res["pass"].sum())
    p, lo, hi = pass_rate_ci(k, n)
    lines.append(f"{'ALL':5s} {k:4d}/{n:<4d} {100 * p:6.1f}%  [{100 * lo:5.1f}, {100 * hi:5.1f}] %")
    return "\n".join(lines)


def main():
    args = parse_args(__doc__, extra=lambda p: (p.add_argument("-n", type=int, default=300),
                                                p.add_argument("--workers", type=int, default=0),
                                                p.add_argument("--seed", type=int, default=2026)))
    n = 16 if args.fast else args.n
    workers = 2 if args.fast else (args.workers or None)
    out = outdir(MODULE)

    cases = sample_cases(n, seed=args.seed)
    t0 = time.time()
    res = evaluate(run_cases(cases, workers=workers))
    print(f"{n} runs in {time.time() - t0:.1f} s")
    res.to_csv(out / "results.csv")

    bad = res[res.status != "ok"]
    print(f"status: {res.status.value_counts().to_dict()}")
    if 0 in res.index and not res.loc[0, "pass"]:
        print("!! the NOMINAL case (0) fails - fix that before reading anything else")

    table = requirement_table(res)
    print("\n" + table)

    # --- what drives each metric that has failures (or is closest to its limit)
    drivers = {}
    print("\nTop drivers (Spearman rank correlation with the metric):")
    for rid, (metric, op, lim, _) in REQUIREMENTS.items():
        m = res[metric].abs() if op == "abs<=" else res[metric]
        res[f"_{metric}"] = m
        s = sensitivity(res, f"_{metric}").head(3)
        drivers[rid] = s
        print(f"  {rid} {metric:18s} " + "  ".join(f"{k} {v:+.2f}" for k, v in s.items()))

    failed = res[~res["pass"]]
    if len(failed):
        print(f"\nFailed cases ({len(failed)}):")
        fail_reqs = failed[list(REQUIREMENTS)].apply(lambda r: ",".join(k for k, v in r.items() if not v), axis=1)
        show = failed.assign(failed=fail_reqs)[["failed", "turb_w20_fps", "wind_mps", "scale_thrust", "payload_kg"]]
        print(show.round(2).to_string())

    # --- side observation: throttle saturation (not a requirement, but a finding)
    sat = (res.throttle_max >= 0.999).mean()
    print(f"\nthrottle hit 100 % during the climb in {100 * sat:.0f} % of runs")

    # --- how many runs do we need?
    for p in (0.1, 0.01, 0.001):
        print(f"  zero failures in {runs_for_confidence(p):5d} runs -> P(fail) < {p:g} with 95 % confidence")

    # --- plots: metric histograms with limits
    rids = list(REQUIREMENTS)
    fig, axes = plt.subplots(3, 3, figsize=(12, 9))
    for ax, rid in zip(axes.flat, rids):
        metric, op, lim, text = REQUIREMENTS[rid]
        x = res[metric].dropna()
        ax.hist(x, bins=30, color="tab:blue", alpha=0.75)
        for L in ([lim, -lim] if op == "abs<=" else [lim]):
            ax.axvline(L, color="r", ls="--")
        if 0 in res.index:
            ax.axvline(res.loc[0, metric], color="k", lw=1, label="nominal")
        ax.set_title(f"{rid}: {100 * res[rid].mean():.1f} % pass", fontsize=9)
        ax.set_xlabel(metric, fontsize=8)
    fig.suptitle(f"Monte Carlo, {n} runs: metric distributions vs requirement limits (red)")
    fig.tight_layout()
    save(fig, MODULE, "histograms", show=args.show)

    # --- the driver of the failing requirement, as a scatter
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    for ax, rid in zip(axes, ("MC-3", "MC-4", "MC-6")):
        metric = REQUIREMENTS[rid][0]
        top = drivers[rid].index[0]
        ax.scatter(res[top], res[metric], s=8, c=np.where(res[rid], "tab:blue", "tab:red"))
        ax.set(xlabel=top, ylabel=metric, title=f"{rid}: top driver {top} (r = {drivers[rid].iloc[0]:+.2f})")
        ax.grid(alpha=0.3)
    fig.tight_layout()
    save(fig, MODULE, "drivers", show=args.show)

    # --- convergence: running failure-rate estimate with its 95 % interval
    fails = (~res["pass"]).to_numpy().astype(int)
    ns = np.arange(1, n + 1)
    k = np.cumsum(fails)
    est = np.array([pass_rate_ci(int(ns[i] - k[i]), int(ns[i])) for i in range(n)])
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(ns, 100 * (1 - est[:, 0]), label="failure rate estimate")
    ax.fill_between(ns, 100 * (1 - est[:, 2]), 100 * (1 - est[:, 1]), alpha=0.3, label="95 % interval")
    ax.set(xlabel="runs", ylabel="P(fail) [%]", title="How many runs? The interval shrinks like 1/sqrt(n)",
           ylim=(0, 30))
    ax.grid(alpha=0.3)
    ax.legend()
    save(fig, MODULE, "convergence", show=args.show)

    # --- flight-readiness summary
    k_all = int(res["pass"].sum())
    p, lo, hi = pass_rate_ci(k_all, n)
    summary = f"""# Monte Carlo summary: gnc_trainer autopilot test card

* Runs: {n} (seed {args.seed}); crashed / trim failures: {len(bad)}
* Nominal case: {"PASS" if res.loc[0, "pass"] else "FAIL"}
* Overall pass rate: {100 * p:.1f} % (95 % CI {100 * lo:.1f} to {100 * hi:.1f} %)
* Throttle saturated during the climb in {100 * sat:.0f} % of runs (thrust-limited climb)

## Requirements

```text
{table}
```

## Dispersions

| parameter | distribution |
|---|---|
""" + "\n".join(f"| {k} | {v} |" for k, v in DEFAULT_DISPERSIONS.items()) + """

## Findings

(Write these yourself: which requirements failed, under which conditions, what
drives them, and whether the fix belongs in the design or the requirement.)
"""
    (out / "summary.md").write_text(summary)
    print(f"\nwrote {out / 'results.csv'} and {out / 'summary.md'}")


if __name__ == "__main__":
    main()
