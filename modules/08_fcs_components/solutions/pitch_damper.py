"""Module 08 - a pitch damper, and what actuators, sensors and delay do to it.

1. Linear design: root locus of the short period under q -> elevator feedback;
   pick Kq for zeta_sp ~ 0.7.  Then predict what a 40 rad/s actuator and an
   extra 0.1 s delay (Pade) do to the closed-loop poles.
2. Nonlinear check: the same elevator doublet flown six ways in JSBSim.

    python modules/08_fcs_components/solutions/pitch_damper.py [--show]
"""

from pathlib import Path

from gnclab.cli import parse_args

args = parse_args(__doc__)

import control  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from gnclab import initialize, make_fdm, run  # noqa: E402
from gnclab.linear import linearize_fd  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402
from gnclab.signals import doublet  # noqa: E402
from gnclab.trim import trim  # noqa: E402

FCS_DIR = Path(__file__).with_name("fcs")
LONG = ["Vt", "Alpha", "Theta", "Q"]


def trimmed(**props):
    fdm = make_fdm("gnc_trainer", systems_dir=FCS_DIR)
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048})
    trim(fdm, "full")          # trim with everything off/ideal ...
    for k, v in props.items():  # ... then switch things on
        fdm[k] = v
    return fdm


# --- 1. linear design ------------------------------------------------------------
lin = linearize_fd(trimmed(), ["fcs/elevator-cmd-norm"]).subsystem(LONG, ["fcs/elevator-cmd-norm"])
G = control.ss(lin.A, lin.B, lin.C, lin.D)
q_de = G[3, 0]                       # q / elevator_norm


def sp_poles(L_open, k):
    """Closed-loop short-period pole (largest |imag| pair) for positive feedback gain k
    on q (elevator = +k q).  control.feedback uses negative feedback, so pass -k."""
    cl = control.feedback(L_open, -k)
    p = cl.poles()
    return p[np.argmax(np.abs(p.imag) * (np.abs(p) > 2))]


def zeta(p):
    return -p.real / abs(p)


gains = np.linspace(0, 2.5, 251)
act = control.tf([40], [1, 40])
num, den = control.pade(0.1, 3)
delay = control.tf(num, den)
loops = {"ideal": q_de, "+ actuator (40 rad/s)": q_de * act, "+ actuator + 0.1 s delay": q_de * act * delay}
zetas = {name: [zeta(sp_poles(L, k)) for k in gains] for name, L in loops.items()}
k_design = gains[np.argmin(np.abs(np.array(zetas["ideal"]) - 0.7))]
print(f"open-loop short period zeta = {zeta(sp_poles(q_de, 0)):.2f}")
print(f"design gain for zeta_sp = 0.7 (ideal): Kq = {k_design:.2f} norm per rad/s "
      f"= {k_design * 25:.1f} deg elevator per rad/s")
for name, zs in zetas.items():
    print(f"   at Kq = {k_design:.2f}: zeta_sp {name:28s} = {np.interp(k_design, gains, zs):.2f}")
for name, L in loops.items():
    unstable = [k for k in gains[1:] if np.any(control.feedback(L, -k).poles().real > 0)]
    print(f"   first unstable gain {name:28s}: {unstable[0] if unstable else '> 2.5'}")

fig, ax = plt.subplots(figsize=(7, 4.5))
for name, zs in zetas.items():
    ax.plot(gains, zs, label=name)
ax.axvline(k_design, color="k", ls=":")
ax.set(xlabel="Kq [elevator norm per rad/s]", ylabel="short-period damping ratio", ylim=(-0.2, 1.05),
       title="Pitch damper: damping vs gain")
ax.grid(alpha=0.3)
ax.legend()
save(fig, "08_fcs_components", "damper_zeta_vs_gain", show=args.show)

# --- 2. nonlinear flights -------------------------------------------------------
cases = {
    "no damper": {},
    "damper, ideal": {"ap/pitch-damper-on": 1, "ap/kq": k_design},
    "+ actuators": {"ap/pitch-damper-on": 1, "ap/kq": k_design, "fcs/actuators-on": 1},
    "+ actuators + sensors": {"ap/pitch-damper-on": 1, "ap/kq": k_design, "fcs/actuators-on": 1, "sensors/enabled": 1},
    "+ ... + 0.1 s delay": {"ap/pitch-damper-on": 1, "ap/kq": k_design, "fcs/actuators-on": 1, "sensors/enabled": 1,
                            "ap/q-delay-on": 1},
    "4x gain + act + delay": {"ap/pitch-damper-on": 1, "ap/kq": 4 * k_design, "fcs/actuators-on": 1,
                              "ap/q-delay-on": 1},
}
runs = {}
for name, props in cases.items():
    fdm = trimmed(**props)
    de0 = fdm["fcs/elevator-cmd-norm"]
    df = run(fdm, 4.0 if args.fast else 8.0,
             {"q_dps": "velocities/q-rad_sec", "theta_deg": "attitude/theta-deg", "alpha_deg": "aero/alpha-deg",
              "elevator_deg": "fcs/elevator-pos-rad", "damper_cmd": "ap/elevator-cmd-norm"},
             callback=lambda f, t: f.__setitem__("fcs/elevator-cmd-norm", de0 + doublet(t, 0.5, 0.3, -0.2)))
    df["q_dps"] *= 57.2958
    df["elevator_deg"] *= 57.2958
    runs[name] = df
    tail = df.loc[1.5:]
    print(f"{name:26s} peak |q| {df.q_dps.abs().max():6.1f} deg/s, residual q after 1.5 s "
          f"{tail.q_dps.abs().max():6.2f} deg/s, elevator travel {df.elevator_deg.max() - df.elevator_deg.min():5.1f} deg")

fig, _ = timehistory(runs, ["q_dps", "theta_deg", "elevator_deg", "damper_cmd"],
                     title="gnc_trainer elevator doublet: pitch damper variants")
save(fig, "08_fcs_components", "pitch_damper", show=args.show)
