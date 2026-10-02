"""Monte Carlo verification for gnc_trainer (Module 15).

The pieces of a requirements Monte Carlo:

* **Dispersions**: :class:`Normal` / :class:`Uniform` distributions keyed by
  name, sampled reproducibly by :func:`sample_cases` (case 0 = nominal).
* **One run**: :func:`requirements_flight` flies a test card (altitude step,
  then a 90 deg course change) with one set of dispersions applied, with
  realistic actuators and sensors, and returns metrics plus a status.
* **Many runs**: :func:`run_cases` farms cases out to worker processes.  It
  uses the *spawn* start method on every OS so it behaves the way it will on
  Windows: the calling script MUST protect its entry point with
  ``if __name__ == "__main__":``.
* **Verdict**: :data:`REQUIREMENTS`, :func:`evaluate`, :func:`pass_rate_ci`
  (Clopper-Pearson), :func:`runs_for_confidence` (the "rule of three"), and
  :func:`sensitivity` (rank correlation of a metric with each dispersion:
  which parameter drives the failures?) and :func:`unusual` (what is odd
  about one failed case?).

Dispersion names understood by :func:`apply_dispersions`:

=================  ===========================================================
``payload_kg``      payload point mass [kg] (0 = none)
``payload_x_m``     payload x station [m]; nominal CG is 0.40 (x aft)
``scale_<name>``    ``uncertainty/<name>-scale`` (CL, CD, Cma, Cmq, Cmde, Clb, Clp,
                    Clda, Cnb, Cnr, thrust); 1 = nominal
``wind_mps``        steady wind speed [m/s]
``wind_from_deg``   direction the wind blows FROM [deg true]
``turb_w20_fps``    MIL-F-8785C wind speed at 20 ft [ft/s] (sets turbulence intensity)
``act_bw``          actuator bandwidth [rad/s]
``act_rate_dps``    actuator rate limit [deg/s]
``gyro_bias_<p|q|r>`` gyro bias [rad/s] (``sensors/gyro-bias-<p|q|r>-rad_sec``)
``seed``            random seed for sensor noise and turbulence
=================  ===========================================================
"""

from __future__ import annotations

import math
import multiprocessing
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from typing import Callable, Mapping

import numpy as np
import pandas as pd

FT = 0.3048


# ---------------------------------------------------------------------------
# Dispersions
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Normal:
    mean: float
    sigma: float
    lo: float = -math.inf       # truncation (samples are clipped to [lo, hi])
    hi: float = math.inf

    def draw(self, rng, n):
        return np.clip(rng.normal(self.mean, self.sigma, n), self.lo, self.hi)

    @property
    def nominal(self):
        return self.mean


@dataclass(frozen=True)
class Uniform:
    lo: float
    hi: float
    nom: float | None = None    # value used for the nominal case (default: lo)

    def draw(self, rng, n):
        return rng.uniform(self.lo, self.hi, n)

    @property
    def nominal(self):
        return self.lo if self.nom is None else self.nom


# 1-sigma aero uncertainties are typical of a wind-tunnel-less small UAS
# database (handbook/VLM estimates): damping and cross-derivatives are the
# least certain, the lift curve the most.
DEFAULT_DISPERSIONS: dict[str, Normal | Uniform] = {
    "payload_kg": Uniform(0.0, 1.5),
    "payload_x_m": Normal(0.40, 0.03, 0.32, 0.48),
    "scale_CL": Normal(1.0, 0.05),
    "scale_CD": Normal(1.0, 0.15, 0.6),
    "scale_Cma": Normal(1.0, 0.15, 0.4),
    "scale_Cmq": Normal(1.0, 0.20, 0.3),
    "scale_Cmde": Normal(1.0, 0.15, 0.5),
    "scale_Clb": Normal(1.0, 0.20, 0.2),
    "scale_Clp": Normal(1.0, 0.15, 0.4),
    "scale_Clda": Normal(1.0, 0.15, 0.5),
    "scale_Cnb": Normal(1.0, 0.20, 0.3),
    "scale_Cnr": Normal(1.0, 0.20, 0.3),
    "scale_thrust": Normal(1.0, 0.10, 0.6),
    "wind_mps": Uniform(0.0, 8.0),
    "wind_from_deg": Uniform(0.0, 360.0),
    "turb_w20_fps": Uniform(0.0, 25.0),           # 0 .. ~15 kt at 20 ft: calm to light/moderate
    "act_bw": Uniform(25.0, 50.0, nom=40.0),
    "act_rate_dps": Uniform(150.0, 300.0, nom=250.0),
}


def sample_cases(n: int, dispersions: Mapping[str, Normal | Uniform] | None = None,
                 seed: int = 0, include_nominal: bool = True) -> pd.DataFrame:
    """``n`` cases (rows) of dispersed parameters.  Same ``seed`` -> same cases.

    Case 0 is the nominal vehicle in calm air when ``include_nominal`` (always
    fly it: if the nominal case fails, nothing else matters).
    """
    dispersions = DEFAULT_DISPERSIONS if dispersions is None else dispersions
    rng = np.random.default_rng(seed)
    cols = {k: d.draw(rng, n) for k, d in dispersions.items()}
    cols["seed"] = rng.integers(1, 2**31 - 1, n)
    df = pd.DataFrame(cols)
    if include_nominal and n > 0:
        for k, d in dispersions.items():
            df.loc[0, k] = d.nominal
    df.index.name = "case"
    return df


def apply_dispersions(fdm, case: Mapping[str, float]) -> None:
    """Write a case's parameters into JSBSim.  Call BEFORE ``run_ic()`` (mass,
    aero, actuators); the wind is applied here too, so trim accounts for it.
    Turbulence is switched on separately, after trim (see :func:`turbulence_on`)."""
    c = dict(case)
    if "seed" in c:
        fdm["simulation/randomseed"] = int(c["seed"])        # sensor noise
        fdm["atmosphere/randomseed"] = int(c["seed"]) + 1    # turbulence
    if "payload_kg" in c:
        fdm["inertia/pointmass-weight-lbs[0]"] = c["payload_kg"] * 2.20462
    if "payload_x_m" in c:
        fdm["inertia/pointmass-location-X-inches[0]"] = c["payload_x_m"] / 0.0254
    for k, v in c.items():
        if k.startswith("scale_"):
            fdm[f"uncertainty/{k[6:]}-scale"] = v
    if "act_bw" in c:
        fdm["fcs/actuator-bw-rad_sec"] = c["act_bw"]
    if "act_rate_dps" in c:
        fdm["fcs/actuator-rate-rad_sec"] = math.radians(c["act_rate_dps"])
    for ax in "pqr":
        if f"gyro_bias_{ax}" in c:
            fdm[f"sensors/gyro-bias-{ax}-rad_sec"] = c[f"gyro_bias_{ax}"]
    w = c.get("wind_mps", 0.0) / FT
    frm = math.radians(c.get("wind_from_deg", 0.0))
    fdm["atmosphere/wind-north-fps"] = -w * math.cos(frm)   # blows TOWARD from + 180 deg
    fdm["atmosphere/wind-east-fps"] = -w * math.sin(frm)


def turbulence_on(fdm, w20_fps: float) -> None:
    """MIL-F-8785C turbulence (Dryden form).  Below 1000 ft AGL the intensity
    is sigma_w = 0.1 * W20; the ``severity`` index only matters above 2000 ft,
    BUT an index of 0 disables the model entirely, so it must be set."""
    if w20_fps <= 0:
        fdm["atmosphere/turb-type"] = 0
        return
    fdm["atmosphere/turb-type"] = 3                       # ttMilspec
    fdm["atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps"] = w20_fps
    fdm["atmosphere/turbulence/milspec/severity"] = 3


# ---------------------------------------------------------------------------
# One run: the requirements test card
# ---------------------------------------------------------------------------
T_ALT, T_TURN, T_END = 2.0, 35.0, 65.0
ALT_STEP_FT, TURN_DEG = 100.0, 90.0
CARD_PROPS = {
    "alt_ft": "position/h-sl-ft", "vt_fps": "velocities/vt-fps", "chi": "flight-path/psi-gt-rad",
    "beta_deg": "aero/beta-deg", "alpha_deg": "aero/alpha-deg", "phi_deg": "attitude/phi-deg",
    "de_ap": "ap/elevator-cmd-norm", "da_ap": "ap/aileron-cmd-norm", "throttle": "fcs/throttle-total-norm",
}


def requirements_flight(case: Mapping[str, float], keep_history: bool = False) -> dict:
    """Fly the test card for one case; return {metrics..., "status": ...}.

    Test card (realistic actuators + sensors, full autopilot, 330 ft, 25 m/s, heading north):
      t = 2 s   altitude command +100 ft                      (R-L1)
      t = 35 s  course command +90 deg                         (R-A2..R-A4)
      t = 65 s  end
    Any exception (trim failure, NaN) is caught and reported as a status, not
    raised: in a Monte Carlo a crash is a *result*.
    """
    from gnclab import initialize, make_fdm, run          # imported here: cheap for workers
    from gnclab.autopilot import engage_lateral, engage_longitudinal, realism
    from gnclab.trim import TrimError, trim

    out: dict = {"status": "ok"}
    try:
        fdm = make_fdm("gnc_trainer")
        apply_dispersions(fdm, case)
        initialize(fdm, {"lat-geod-deg": 33.7, "long-gc-deg": -117.9, "h-sl-ft": 330,
                         "vt-fps": 25 / FT, "psi-true-deg": 0.0})
        try:
            trim(fdm, "full")
        except TrimError:
            return {"status": "trim-fail"}
        realism(fdm, True)
        turbulence_on(fdm, case.get("turb_w20_fps", 0.0))
        engage_longitudinal(fdm)
        engage_lateral(fdm)
        h0, chi0 = fdm["position/h-sl-ft"], fdm["flight-path/psi-gt-rad"]

        def card(f, t):
            if t >= T_ALT:
                f["ap/alt-cmd-ft"] = h0 + ALT_STEP_FT
            if t >= T_TURN:
                f["ap/chi-cmd-rad"] = (chi0 + math.radians(TURN_DEG)) % (2 * math.pi)

        df = run(fdm, T_END, CARD_PROPS, callback=card, record_every=6)
    except Exception as exc:                               # noqa: BLE001
        return {"status": f"error: {type(exc).__name__}: {exc}"[:120]}
    if not np.all(np.isfinite(df.to_numpy())):
        return {"status": "diverged"}

    chi = np.degrees(np.unwrap(df.chi.to_numpy()))
    chi = chi - 360 * np.round((chi[0] - math.degrees(chi0)) / 360) - math.degrees(chi0)  # relative, deg
    df["chi_rel_deg"] = chi
    t = df.index.to_numpy()
    h_cmd = h0 + ALT_STEP_FT
    step = (t >= T_ALT) & (t < T_TURN)
    turn = t >= T_TURN
    late = t >= T_END - 8.0
    e_h = df.alt_ft.to_numpy() - h_cmd
    outside = step & (np.abs(e_h) > 10.0)
    out.update(
        alt_overshoot_ft=float(e_h[step].max()),
        alt_settle_s=float(t[outside].max() - T_ALT) if outside.any() else 0.0,
        alt_err_end_ft=float(e_h[(t >= T_TURN - 5) & (t < T_TURN)].mean()),
        chi_overshoot_deg=float(chi[turn].max() - TURN_DEG),
        chi_err_late_deg=float(np.abs(chi[late] - TURN_DEG).max()),
        beta_max_deg=float(df.beta_deg[turn].abs().max()),
        alt_dev_turn_ft=float(np.abs(e_h[turn]).max()),
        de_sat_s=float((df.de_ap.abs() >= 0.99).sum() * 6 / 120.0),
        da_sat_s=float((df.da_ap.abs() >= 0.99).sum() * 6 / 120.0),
        vt_min_mps=float(df.vt_fps.min() * FT),
        alpha_max_deg=float(df.alpha_deg.max()),
        throttle_max=float(df.throttle.max()),
    )
    if keep_history:
        out["history"] = df
    return out


# ---------------------------------------------------------------------------
# Requirements and statistics
# ---------------------------------------------------------------------------
# id: (metric, "<=" or ">=", limit, text).  Derived from Modules 09/10 (R-L*, R-A*),
# restated for a disturbed environment: steady-state errors are averaged over
# 5 s and every number is judged on TRUTH, not on the noisy sensors.
REQUIREMENTS: dict[str, tuple[str, str, float, str]] = {
    "MC-1": ("alt_overshoot_ft", "<=", 15.0, "altitude step overshoot <= 15 ft (R-L1)"),
    "MC-2": ("alt_settle_s", "<=", 30.0, "altitude within +/-10 ft by 30 s (R-L1)"),
    "MC-3": ("alt_err_end_ft", "abs<=", 5.0, "mean altitude error over 28-33 s <= 5 ft (R-L1)"),
    "MC-4": ("chi_overshoot_deg", "<=", 10.0, "course overshoot <= 10 deg (R-A2)"),
    "MC-5": ("chi_err_late_deg", "<=", 5.0, "course within +/-5 deg, last 8 s (R-A2)"),
    "MC-6": ("beta_max_deg", "<=", 5.0, "|beta| <= 5 deg in the turn (R-A3)"),
    "MC-7": ("alt_dev_turn_ft", "<=", 30.0, "altitude within +/-30 ft in the turn (R-A4)"),
    "MC-8": ("de_sat_s", "<=", 0.5, "elevator command saturated <= 0.5 s total (R-L4)"),
    "MC-9": ("vt_min_mps", ">=", 18.0, "airspeed never below 18 m/s (stall margin)"),
}


def evaluate(results: pd.DataFrame, requirements=None) -> pd.DataFrame:
    """Add a boolean column per requirement plus ``pass`` (all requirements
    met AND status ok).  A run that crashed fails every requirement."""
    requirements = REQUIREMENTS if requirements is None else requirements
    df = results.copy()
    ok = df["status"].eq("ok")
    for rid, (metric, op, lim, _) in requirements.items():
        x = df[metric] if metric in df else pd.Series(np.nan, index=df.index)
        if op == "<=":
            met = x <= lim
        elif op == ">=":
            met = x >= lim
        else:                                              # "abs<="
            met = x.abs() <= lim
        df[rid] = met & ok
    df["pass"] = df[list(requirements)].all(axis=1)
    return df


def pass_rate_ci(k: int, n: int, conf: float = 0.95) -> tuple[float, float, float]:
    """Pass rate k/n and its two-sided Clopper-Pearson (exact binomial) interval."""
    from scipy.stats import beta
    a = 1 - conf
    lo = 0.0 if k == 0 else beta.ppf(a / 2, k, n - k + 1)
    hi = 1.0 if k == n else beta.ppf(1 - a / 2, k + 1, n - k)
    return k / n, float(lo), float(hi)


def runs_for_confidence(p_fail_max: float, conf: float = 0.95) -> int:
    """Failure-free runs needed to claim P(fail) < p_fail_max at ``conf``.
    (1 - p)^n <= 1 - conf  ->  n >= ln(1 - conf) / ln(1 - p);  ~3/p at 95 %."""
    return math.ceil(math.log(1 - conf) / math.log(1 - p_fail_max))


def sensitivity(df: pd.DataFrame, metric: str, params: list[str] | None = None) -> pd.Series:
    """Spearman rank correlation of ``metric`` with each dispersed parameter,
    sorted by magnitude: the top entries are what drives that metric."""
    params = [c for c in df.columns if c in DEFAULT_DISPERSIONS] if params is None else params
    ok = df[df["status"] == "ok"]
    r = ok[params].corrwith(ok[metric], method="spearman")
    return r.reindex(r.abs().sort_values(ascending=False).index)


def unusual(case: Mapping[str, float], k: int = 4, dispersions=None) -> str:
    """The ``k`` parameters of a case farthest from nominal, as a string.
    Normal dispersions in sigmas; uniform ones as a fraction of their range."""
    dispersions = DEFAULT_DISPERSIONS if dispersions is None else dispersions
    dev = {}
    for name, d in dispersions.items():
        if name not in case:
            continue
        if isinstance(d, Normal):
            dev[name] = ((case[name] - d.mean) / d.sigma, f"{case[name] - d.mean:+.2f} ({(case[name] - d.mean) / d.sigma:+.1f} sig)")
        elif name != "wind_from_deg":                   # (a direction is never "unusual")
            # rank the edge of a uniform range like a 2-sigma value
            frac = (case[name] - d.lo) / (d.hi - d.lo)
            dev[name] = (4 * abs(frac - 0.5), f"{case[name]:.3g} ({100 * frac:.0f} % of range)")
    top = sorted(dev.items(), key=lambda kv: -abs(kv[1][0]))[:k]
    return ", ".join(f"{n} {txt}" for n, (_, txt) in top)


# ---------------------------------------------------------------------------
# Parallel execution
# ---------------------------------------------------------------------------
def _call(args):
    fn, case = args
    return fn(case)


def run_cases(cases: pd.DataFrame, fn: Callable[[dict], dict] = requirements_flight,
              workers: int | None = None, progress: bool = True) -> pd.DataFrame:
    """Run ``fn(case_dict)`` for every row of ``cases``; return cases + results.

    ``workers=1`` runs serially in this process (easiest to debug).  Otherwise
    a spawn-based process pool is used, exactly as on Windows, so ``fn`` must be
    importable (defined at module top level) and the calling script must use
    ``if __name__ == "__main__":``.
    """
    rows = [dict(r) for _, r in cases.iterrows()]
    workers = workers or max(1, min(len(rows), (os.cpu_count() or 2) - 1))
    results = []
    every = max(1, len(rows) // 10)

    def report(i, extra=""):
        if progress and ((i + 1) % every == 0 or i + 1 == len(rows)):
            print(f"  case {i + 1}/{len(rows)}{extra}", flush=True)

    if workers == 1:
        for i, r in enumerate(rows):
            results.append(fn(r))
            report(i)
    else:
        ctx = multiprocessing.get_context("spawn")
        with ProcessPoolExecutor(max_workers=workers, mp_context=ctx) as pool:
            for i, res in enumerate(pool.map(_call, [(fn, r) for r in rows], chunksize=1)):
                results.append(res)
                report(i, f" ({workers} workers)")
    res = pd.DataFrame([{k: v for k, v in r.items() if k != "history"} for r in results], index=cases.index)
    return pd.concat([cases, res], axis=1)
