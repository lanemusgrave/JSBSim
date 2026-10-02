"""Module 18 - capstone: from design model to a flight-readiness recommendation.

  1. Design check: fly the mission on the design model.  Fix what fails.
  2. Flight test: fly 3-2-1-1s on the AS-BUILT airplane (hidden in as_built.py)
     and identify its derivatives by equation error (Module 14).
  3. Model update: re-centre the uncertainty model on what was identified.
  4. Monte Carlo the mission (Module 15) on the design model and on the
     updated model; compare with the "first flight" of the as-built airplane.
  5. Write the flight-readiness memo (outputs/18_capstone/flight_readiness_memo.md).

    python modules/18_capstone/solutions/capstone.py [--show] [--fast]
"""

import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from gnclab.cli import parse_args

HERE = Path(__file__).resolve().parent
MODULES = HERE.parents[1]
sys.path.insert(0, str(MODULES / "14_system_id" / "solutions"))
sys.path.insert(0, str(MODULES / "07_build_uas_model" / "solutions"))
sys.path.insert(0, str(HERE))

MODULE = "18_capstone"
WEATHER = {"wind_mps": 5.0, "wind_from_deg": 270.0, "turb_w20_fps": 10.0}   # the first-flight forecast
# Equation-error accuracy seen in Module 14 (vs truth): the honest 1-sigma for an identified
# derivative is that, not the (optimistic) OLS standard error.
EE_SIGMA = {"Cma": 0.05, "Cmq": 0.20, "Cmde": 0.08, "Clb": 0.05, "Clp": 0.05, "Clda": 0.05,
            "Cnb": 0.05, "Cnr": 0.10}


# --------------------------------------------------------------------------- 1
def design_check(fly_mission, evaluate, REQ, nominal):
    print("1. Design check: mission on the design model, forecast weather")
    rows = []
    for mgr in ("line", "fillet"):
        r = fly_mission(dict(nominal, **WEATHER, manager=mgr))
        rows.append(dict(r, manager=mgr))
    res = evaluate(pd.DataFrame(rows), REQ)
    for _, r in res.iterrows():
        failed = [k for k in REQ if not r[k]]
        print(f"   {r.manager:6s}: max |xtrack| {r.xtrack_max_m:5.1f} m, mean {r.xtrack_mean_m:4.1f} m, "
              f"done at {r.t_complete_s:5.1f} s -> {'PASS' if r['pass'] else 'FAIL ' + ','.join(failed)}")
    return res


# --------------------------------------------------------------------------- 2
def flight_test_and_identify(fast):
    from build_gnc_trainer import P
    from flight_test import A, maneuver
    from gnclab import OUTPUT_DIR
    from gnclab.signals import multistep_3211
    from gnclab.sysid import EQUATION_ERROR_MODELS, measured_coefficients, ols
    from as_built import AS_BUILT

    print("\n2. Flight test on the as-built airplane: 3-2-1-1s, equation error")
    props = {f"uncertainty/{k[6:]}-scale": v for k, v in AS_BUILT.items()}
    maneuver("long_3211", de_sig=lambda t: multistep_3211(t, 2.0, 0.25, A), props=props, module=MODULE)
    maneuver("lat_3211", da_sig=lambda t: multistep_3211(t, 2.0, 0.25, A),
             dr_sig=lambda t: multistep_3211(t, 6.0, 0.4, 1.5 * A), props=props, module=MODULE)
    logs = {k: measured_coefficients(pd.read_csv(OUTPUT_DIR / MODULE / f"{k}_3211.csv", index_col="t"), P)
            for k in ("long", "lat")}
    ident = {}
    for y in ("Cm", "Cl", "Cn"):
        cols, names, kind = EQUATION_ERROR_MODELS[y]
        res = ols(logs[kind][cols].to_numpy(), logs[kind][y].to_numpy(), names)
        for name, est in res.table.estimate.items():
            if name in EE_SIGMA:
                ident[name] = est / P[name]                     # identified scale vs the design value
    rows = []
    for name, scale in ident.items():
        rows.append({"derivative": name, "design": P[name], "identified": scale * P[name],
                     "scale": scale, "sigma (EE accuracy)": EE_SIGMA[name]})
    table = pd.DataFrame(rows).set_index("derivative")
    print(table.round(3).to_string())
    return table


# --------------------------------------------------------------------------- 3/4
def updated_dispersions(table):
    from gnclab.montecarlo import DEFAULT_DISPERSIONS, Normal

    disp = dict(DEFAULT_DISPERSIONS)
    for name, row in table.iterrows():
        key = f"scale_{name}"
        if key in disp:
            disp[key] = Normal(row.scale, row["sigma (EE accuracy)"] * abs(row.scale), 0.2)
    return disp


def monte_carlo(label, disp, n, workers, fly_mission, run_cases, evaluate, sample_cases, REQ):
    cases = sample_cases(n, disp, seed=18)
    for k, v in WEATHER.items():                       # the forecast, +/- a little
        if k != "turb_w20_fps":
            cases[k] = v
    cases["turb_w20_fps"] = np.random.default_rng(18).uniform(5.0, 15.0, n)
    cases["manager"] = "fillet"
    t0 = time.time()
    res = evaluate(run_cases(cases, fn=fly_mission, workers=workers, progress=False), REQ)
    print(f"   {label}: {n} missions in {time.time() - t0:.0f} s, all requirements met in "
          f"{100 * res['pass'].mean():.0f} %")
    return res


def main():
    args = parse_args(__doc__)
    from as_built import AS_BUILT
    from gnclab.mission import MISSION_REQUIREMENTS as REQ
    from gnclab.mission import fly_mission
    from gnclab.montecarlo import evaluate, pass_rate_ci, run_cases, sample_cases, sensitivity
    from gnclab.plotting import outdir, save

    out = outdir(MODULE)
    n = 4 if args.fast else 60
    workers = 2 if args.fast else None
    nominal = sample_cases(1).iloc[0].to_dict()

    design = design_check(fly_mission, evaluate, REQ, nominal)
    table = flight_test_and_identify(args.fast)

    print("\n3-4. Monte Carlo of the mission (fillets, forecast wind, W20 5-15 ft/s)")
    from gnclab.montecarlo import DEFAULT_DISPERSIONS
    pre = monte_carlo("design model + handbook uncertainty", DEFAULT_DISPERSIONS, n, workers,
                      fly_mission, run_cases, evaluate, sample_cases, REQ)
    post = monte_carlo("sysID-updated model           ", updated_dispersions(table), n, workers,
                       fly_mission, run_cases, evaluate, sample_cases, REQ)
    pre.to_csv(out / "mc_design_model.csv")
    post.to_csv(out / "mc_updated_model.csv")
    print(f"   {'req':5s} {'design model':>13s} {'updated':>9s}")
    for rid in REQ:
        print(f"   {rid:5s} {100 * pre[rid].mean():12.0f}% {100 * post[rid].mean():8.0f}%   {REQ[rid][3]}")

    # the first flight: the real (as-built) airplane, forecast weather, its own noise
    flights = []
    for seed in (101, 102, 103):
        f = fly_mission(dict(nominal, **AS_BUILT, **WEATHER, manager="fillet", seed=seed), keep_history=(seed == 101))
        flights.append(f)
    hist = flights[0].pop("history")
    flights[0].pop("guidance_log")
    flight = evaluate(pd.DataFrame(flights), REQ)

    metrics = ["xtrack_max_m", "xtrack_mean_m", "alt_err_max_ft", "phi_max_deg", "elevator_rms_deg",
               "theta_est_err_rms_deg"]
    print("\n   first flight (as-built airplane, 3 runs) vs the predictions: percentile of the flight value")
    print(f"   {'metric':22s} {'flight':>8s} {'design-model MC':>18s} {'updated-model MC':>18s}")
    comp = []
    for m in metrics:
        fv = flight[m].mean()
        pp = 100 * (pre[m] < fv).mean()
        pq = 100 * (post[m] < fv).mean()
        comp.append((m, fv, pre[m].median(), pp, post[m].median(), pq))
        print(f"   {m:22s} {fv:8.2f}   median {pre[m].median():6.2f} (p{pp:3.0f})   "
              f"median {post[m].median():6.2f} (p{pq:3.0f})")

    # --- plots
    fig, axes = plt.subplots(2, 3, figsize=(13, 7))
    for ax, m in zip(axes.flat, metrics):
        bins = np.linspace(min(pre[m].min(), post[m].min(), flight[m].min()),
                           max(pre[m].max(), post[m].max(), flight[m].max()), 25)
        ax.hist(pre[m], bins=bins, alpha=0.5, label="design-model MC")
        ax.hist(post[m], bins=bins, alpha=0.5, label="updated-model MC")
        for v in flight[m]:
            ax.axvline(v, color="k", lw=1.5)
        ax.set_title(m, fontsize=9)
    axes[0, 0].legend(fontsize="small")
    fig.suptitle("Predicted (histograms) vs first flight of the as-built airplane (black lines)")
    fig.tight_layout()
    save(fig, MODULE, "prediction_vs_flight", show=args.show)

    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    wp = np.array([w[:2] for w in __import__("gnclab.mission", fromlist=["MISSION_WPS"]).MISSION_WPS])
    ax.plot(wp[:, 1], wp[:, 0], "ks--", lw=0.8, label="waypoints")
    ax.plot(hist.east_m, hist.north_m, label="first flight (as-built, fillets)")
    ax.set(xlabel="east [m]", ylabel="north [m]", title="Capstone mission, 5 m/s wind from the west")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize="small")
    save(fig, MODULE, "first_flight_track", show=args.show)

    # --- memo
    write_memo(out, design, table, pre, post, flight, comp, REQ, pass_rate_ci, sensitivity, n)
    print(f"\n5. wrote {out / 'flight_readiness_memo.md'}")
    print("\nDebrief (now you may open as_built.py): identified vs actual scale")
    for name, row in table.iterrows():
        actual = AS_BUILT.get(f"scale_{name}", 1.0)
        print(f"   {name:5s} identified {row.scale:5.2f}   actual {actual:4.2f}")


def write_memo(out, design, table, pre, post, flight, comp, REQ, pass_rate_ci, sensitivity, n):
    def req_rows(res):
        lines = []
        for rid, (metric, op, lim, text) in REQ.items():
            k = int(res[rid].sum())
            p, lo, hi = pass_rate_ci(k, len(res))
            worst = res[metric].min() if op == ">=" else res[metric].max()
            text = text.replace("|", "\\|")                     # a bare | would split the table cell
            lines.append(f"| {rid} | {text} | {k}/{len(res)} | {100 * lo:.0f}–{100 * hi:.0f} % | {worst:.2f} |")
        return "\n".join(lines)

    k_post = int(post["pass"].sum())
    p, lo, hi = pass_rate_ci(k_post, len(post))
    flight_ok = bool(flight["pass"].all())
    rec = ("GO, with the operating limits below" if (lo >= 0.90 and flight_ok)
           else "NO-GO until the open items are closed")
    failing = [rid for rid in REQ if not (pre[rid].all() and post[rid].all())]
    drivers = {(lab, rid): sensitivity(res, REQ[rid][0]).head(3)
               for rid in failing for lab, res in (("design", pre), ("updated", post))}
    memo = f"""# Flight-readiness memo: gnc_trainer waypoint mission

*Generated by `modules/18_capstone/solutions/capstone.py`. The numbers are computed; the
judgment sections are yours to write.*

## 1. Recommendation

**{rec}** (rule used: updated-model Monte Carlo lower 95 % bound ≥ 90 % and every
first-flight run meets every requirement. Replace it with your program's criteria.)

## 2. Configuration

- Vehicle: gnc_trainer as built; model updated from flight-test system ID (section 4)
- GNC: fillet path manager (10 Hz) → XML course/roll/yaw-damper loops; Python
  longitudinal autopilot (40 Hz) on attitude EKF + altitude KF estimates
- Mission: 600 m box, 330 → 430 → 360 ft; forecast wind 5 m/s from 270°, W20 5–15 ft/s

## 3. Design check (design model, forecast weather)

| path manager | max cross-track [m] | result |
|---|---|---|
""" + "\n".join(f"| {r.manager} | {r.xtrack_max_m:.1f} | {'PASS' if r['pass'] else 'FAIL'} |"
                for _, r in design.iterrows()) + f"""

Finding: switching legs at the waypoint overshoots 90° corners by about a turn radius
(MR-2). Fillets (Module 12) fix it and are the configuration flown below.

## 4. Model status: system identification of the as-built airplane

| derivative | design | identified | scale | 1σ used |
|---|---|---|---|---|
""" + "\n".join(f"| {i} | {r.design:.3f} | {r.identified:.3f} | {r.scale:.2f} | {100 * r['sigma (EE accuracy)']:.0f} % |"
                for i, r in table.iterrows()) + f"""

Equation error from one 3-2-1-1 per axis. Cmq is known to be biased low by EE (Module
14), so its 1σ is set to 20 %. CL, CD and thrust were not identified and keep the handbook uncertainty.

## 5. Requirements: Monte Carlo ({n} missions each)

Design model, handbook uncertainty:

| req | requirement | pass | 95 % CI | worst |
|---|---|---|---|---|
{req_rows(pre)}

**Updated model:**

| req | requirement | pass | 95 % CI | worst |
|---|---|---|---|---|
{req_rows(post)}

Top drivers of the requirements that failed anywhere (Spearman rank correlation with the metric):

""" + "\n".join(f"- {rid} ({lab} model): " + ", ".join(f"{k} {v:+.2f}" for k, v in d.items())
                for (lab, rid), d in drivers.items()) + f"""

## 6. First flight vs prediction

| metric | flight (mean of 3) | design-model MC median (flight percentile) | updated-model MC median (flight percentile) |
|---|---|---|---|
""" + "\n".join(f"| {m} | {fv:.2f} | {a:.2f} (p{pa:.0f}) | {b:.2f} (p{pb:.0f}) |" for m, fv, a, pa, b, pb in comp) + """

A model is validated for a metric when the flight falls well inside its predicted
distribution (not beyond p5–p95).

## 7. Operating limits (to brief)

- Wind ≤ 8 m/s; turbulence W20 ≤ 15 ft/s (MC-3 restated, Module 15)
- Payload ≤ 1.5 kg, CG within ±3 cm of the design CG (Module 15 margin to failure)
- Bank limit 30° → loiter radius ≥ 150 m in wind (Module 12)

## 8. What this analysis does not cover

AHRS failure modes and GPS dropouts; actuator failures; structural modes; propwash
and ground effect; launch and recovery; the real flight computer's timing (only
emulated in SIL, Module 16); a rate-dependent, OE-identified Cmq.

## 9. Findings and open items (write these)

- …
"""
    (out / "flight_readiness_memo.md").write_text(memo)


if __name__ == "__main__":
    main()
