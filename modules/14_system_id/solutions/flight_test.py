"""Module 14 - fly a simulated system-ID flight test and write the logs.

Maneuvers on gnc_trainer at 25 m/s, each from trim, surfaces through the
realistic actuators, measurements corrupted like real instrumentation:
  long_3211   elevator 3-2-1-1                 (identification)
  long_doublet elevator doublet                (validation - held out)
  long_sweep  elevator frequency sweep 0.1-4 Hz (frequency response)
  lat_3211    aileron 3-2-1-1 then rudder 3-2-1-1 (identification)
  lat_doublet aileron + rudder doublets         (validation)
During longitudinal maneuvers the XML roll hold keeps the wings level
(the airframe is spiral unstable); during lateral ones the pitch hold keeps
the nose steady.  Logs -> outputs/14_system_id/<maneuver>.csv at 100 Hz.

    python modules/14_system_id/solutions/flight_test.py
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.plotting import outdir, save, timehistory  # noqa: E402
from gnclab.signals import chirp, doublet, multistep_3211  # noqa: E402
from gnclab.trim import trim  # noqa: E402

FT, DEG = 0.3048, np.pi / 180
rng = np.random.default_rng(14)

# instrumentation noise (1 sigma) - typical small-UAS flight-test package
NOISE = {"alpha": 0.2 * DEG, "beta": 0.2 * DEG, "p": 0.003, "q": 0.003, "r": 0.003,
         "ax": 0.05, "ay": 0.05, "az": 0.05, "V": 0.2, "de": 0.1 * DEG, "da": 0.1 * DEG, "dr": 0.1 * DEG,
         "phi": 0.2 * DEG, "theta": 0.2 * DEG}

PROPS = {"alpha": "aero/alpha-rad", "beta": "aero/beta-rad", "p": "velocities/p-rad_sec",
         "q": "velocities/q-rad_sec", "r": "velocities/r-rad_sec", "phi": "attitude/phi-rad",
         "theta": "attitude/theta-rad", "V_fps": "velocities/vt-fps", "qbar_psf": "aero/qbar-psf",
         "fx_lbs": "forces/fbx-total-lbs", "fy_lbs": "forces/fby-total-lbs", "fz_lbs": "forces/fbz-total-lbs",
         "thrust_lbs": "external_reactions/propeller/magnitude", "mass_slug": "inertia/mass-slugs",
         "de": "fcs/elevator-pos-rad", "da": "fcs/aileron-pos-rad", "dr": "fcs/rudder-pos-rad"}


def maneuver(name, de_sig=None, da_sig=None, dr_sig=None, T=20.0, props=None, module="14_system_id"):
    """Fly one maneuver; ``props`` are written before run_ic() (the capstone uses
    them to fly a different "as-built" airplane); the log goes to outputs/<module>/."""
    fdm = make_fdm("gnc_trainer")
    for k, v in (props or {}).items():
        fdm[k] = v
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 500, "vt-fps": 25 / FT})
    trim(fdm, "full")
    fdm["fcs/actuators-on"] = 1
    lateral = da_sig is not None or dr_sig is not None
    if lateral:                                    # hold pitch, free roll/yaw
        fdm["ap/theta-trim-rad"] = fdm["attitude/theta-rad"]
        fdm["ap/pitch-hold-on"] = 1
    else:                                          # hold wings level, free pitch
        fdm["ap/phi-cmd-ext-rad"] = 0.0
        fdm["ap/roll-hold-on"] = 1
    c0 = {k: fdm[f"fcs/{k}-cmd-norm"] for k in ("elevator", "aileron", "rudder")}

    def cb(f, t):
        if de_sig:
            f["fcs/elevator-cmd-norm"] = c0["elevator"] + de_sig(t)
        if da_sig:
            f["fcs/aileron-cmd-norm"] = c0["aileron"] + da_sig(t)
        if dr_sig:
            f["fcs/rudder-cmd-norm"] = c0["rudder"] + dr_sig(t)

    props = dict(PROPS)
    try:
        fdm[props["thrust_lbs"]]
    except KeyError:
        props.pop("thrust_lbs")
        props["thrust_N"] = "propulsion/thrust-N"
    df = run(fdm, T, props, callback=cb)
    # resample to 100 Hz like a data recorder
    t = np.arange(0, df.index[-1], 0.01)
    df = pd.DataFrame({c: np.interp(t, df.index, df[c]) for c in df.columns}, index=pd.Index(t, name="t"))
    m = df.mass_slug.to_numpy()
    # accelerometers measure specific force [m/s^2]
    df["ax"], df["ay"], df["az"] = (df.fx_lbs / m * FT, df.fy_lbs / m * FT, df.fz_lbs / m * FT)
    df["V"] = df.V_fps * FT
    for k, s in NOISE.items():
        df[k] = df[k] + rng.normal(0, s, len(df))
    keep = ["alpha", "beta", "p", "q", "r", "phi", "theta", "V", "qbar_psf", "ax", "ay", "az", "de", "da", "dr"]
    path = outdir(module) / f"{name}.csv"
    df[keep].to_csv(path)
    print(f"{name:13s}: {len(df)} samples -> {path.name}")
    return df[keep]


A = 0.12  # normalized surface command amplitude (~3 deg)


def main():
    logs = {
        "long_3211": maneuver("long_3211", de_sig=lambda t: multistep_3211(t, 2.0, 0.25, A)),
        "long_doublet": maneuver("long_doublet", de_sig=lambda t: doublet(t, 2.0, 0.35, A)),
        "long_sweep": maneuver("long_sweep", de_sig=lambda t: chirp(t, 2.0, 40.0 if not args.fast else 15.0,
                                                                    0.1, 4.0, 0.06, taper=1.0),
                               T=45.0 if not args.fast else 20.0),
        "lat_3211": maneuver("lat_3211", da_sig=lambda t: multistep_3211(t, 2.0, 0.25, A),
                             dr_sig=lambda t: multistep_3211(t, 6.0, 0.4, 1.5 * A)),
        "lat_doublet": maneuver("lat_doublet", da_sig=lambda t: doublet(t, 2.0, 0.4, A),
                                dr_sig=lambda t: doublet(t, 5.0, 0.6, 1.5 * A)),
    }
    d = logs["long_3211"].copy()
    for c in ("alpha", "q", "theta", "de"):
        d[c] = np.degrees(d[c])
    fig, _ = timehistory(d, ["de", "alpha", "q", "theta", "az", "V"], title="Flight-test log: elevator 3-2-1-1 (with noise)")
    save(fig, "14_system_id", "long_3211_log", show=args.show)


NAMES = ("long_3211", "long_doublet", "long_sweep", "lat_3211", "lat_doublet")


def ensure_logs():
    """Fly the flight test if any log is missing (so analysis scripts run on a fresh checkout)."""
    if not all((outdir("14_system_id") / f"{n}.csv").exists() for n in NAMES):
        main()


if __name__ == "__main__":
    main()
