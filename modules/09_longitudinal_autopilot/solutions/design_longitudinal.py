"""Module 09 - design the longitudinal autopilot by successive loop closure.

Beard & McLain ch. 6, with the plant numbers taken from OUR JSBSim model
(linearize_fd at 25 m/s) instead of the book's formulas:

  inner  pitch hold     de = kp_th (th_c - th) - kd_th q            (PD)
  outer  altitude hold  th_c = th_trim + kp_h e_h + ki_h int(e_h)   (PI)
  speed  airspeed hold  dt = kp_V e_V + ki_V int(e_V)               (PI on throttle)

Each loop is designed on a 1st/2nd-order approximation, then checked on the
full linear model: closed-loop poles, step response, and gain / phase / DELAY
margins broken at the actuator.  Units: elevator & throttle normalized,
angles rad, altitude ft, speed ft/s - the units the XML autopilot uses.

    python modules/09_longitudinal_autopilot/solutions/design_longitudinal.py [--show]
"""

import json

from gnclab.cli import parse_args

args = parse_args(__doc__)

import control  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from gnclab import OUTPUT_DIR, initialize, make_fdm  # noqa: E402
from gnclab.linear import linearize_fd  # noqa: E402
from gnclab.metrics import loop_margins, step_metrics  # noqa: E402
from gnclab.plotting import save  # noqa: E402
from gnclab.trim import trim  # noqa: E402

VA_MPS = 25.0
fdm = make_fdm("gnc_trainer")
initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": VA_MPS / 0.3048})
trim(fdm, "full")
Va = fdm["velocities/vt-fps"]
lin = linearize_fd(fdm, ["fcs/elevator-cmd-norm", "fcs/throttle-cmd-norm"]).subsystem(
    ["Vt", "Alpha", "Theta", "Q"], ["fcs/elevator-cmd-norm", "fcs/throttle-cmd-norm"])
A, B = lin.A, lin.B
iV, iA, iT, iQ = range(4)

# ---- plant approximations (B&M sec. 5.4) ------------------------------------------
a_th1, a_th2, a_th3 = -A[iQ, iQ], -A[iQ, iA], B[iQ, 0]   # th/de ~ a3 / (s^2 + a1 s + a2)
a_V1, a_V2 = -A[iV, iV], B[iV, 1]                         # Va/dt ~ a2 / (s + a1)
print(f"plant: a_th1 = {a_th1:.3f}, a_th2 = {a_th2:.3f}, a_th3 = {a_th3:.3f} (per elevator norm)")
print(f"       a_V1 = {a_V1:.4f} 1/s, a_V2 = {a_V2:.2f} ft/s^2 per throttle norm")

# ---- inner loop: pitch attitude (PD) ----------------------------------------------
DE_MAX, E_TH_MAX, ZETA_TH = 1.0, np.radians(15.0), 0.75
kp_th = DE_MAX / E_TH_MAX * np.sign(a_th3)                # saturate at a 20 deg error
w_th = np.sqrt(a_th2 + kp_th * a_th3)
kd_th = (2 * ZETA_TH * w_th - a_th1) / a_th3
K_th_dc = kp_th * a_th3 / (a_th2 + kp_th * a_th3)
print(f"\npitch: kp = {kp_th:.3f}, kd = {kd_th:.3f}  ->  w = {w_th:.2f} rad/s, DC gain {K_th_dc:.3f}")

# ---- the inner loop's REAL DC gain, from the full linear model --------------------------
def tf_of(row, col):
    C = np.zeros((1, 4))
    C[0, row] = 1.0
    return control.tf(control.ss(A, B[:, [col]], C, 0.0))


th_de, q_de = tf_of(iT, 0), tf_of(iQ, 0)
K_th_full = float(np.real(control.dcgain(kp_th * th_de / (1 + kp_th * th_de + kd_th * q_de))))
print(f"       pitch-loop DC gain: 2nd-order approximation {K_th_dc:.3f}, full 4-state model {K_th_full:.3f}"
      "  <- use the real one for the outer loop")

# ---- outer loop: altitude from pitch (PI) ------------------------------------------
W_H, ZETA_H = 60.0, 1.5     # bandwidth separation / damping: see the sweep in the README
w_h = w_th / W_H
ki_h = w_h ** 2 / (K_th_full * Va)
kp_h = 2 * ZETA_H * w_h / (K_th_full * Va)
print(f"altitude: kp = {kp_h:.5f} rad/ft, ki = {ki_h:.6f} rad/ft/s  ->  w = {w_h:.3f} rad/s")

# ---- airspeed from throttle (PI) -------------------------------------------------
W_V, ZETA_V = 0.35, 0.9
ki_V = W_V ** 2 / a_V2
kp_V = (2 * ZETA_V * W_V - a_V1) / a_V2
print(f"airspeed: kp = {kp_V:.5f} per ft/s, ki = {ki_V:.6f} per ft  ->  w = {W_V:.2f} rad/s")


# ---- check: ALL loops closed on the full linear model ----------------------------------------
def closed_loop(break_at=None):
    """State-space model of airframe + 40 rad/s elevator actuator + all three loops.

    States  [Vt, alpha, theta, q, h, de_act, z_h, z_V]   (z = integrator states)
    Inputs  [h_c, V_c, u_brk]   Outputs [h, Vt, theta, y_brk]
    break_at in {None, "elevator", "theta_c", "throttle"} opens that one loop:
    the controller's signal there comes out as y_brk and the plant takes u_brk
    instead, so the loop gain with every OTHER loop closed is  L = -y_brk/u_brk.
    """
    n = 8
    Acl, Bcl, Ccl = np.zeros((n, n)), np.zeros((n, 3)), np.zeros((4, n))
    Acl[:4, :4] = A
    Acl[4, 1], Acl[4, 2] = -Va, Va                          # h_dot = Va (theta - alpha)
    # controller signals as rows over the state: theta_c = kp_h (h_c - h) + ki_h z_h
    thc_x = np.zeros(n); thc_x[4], thc_x[6] = -kp_h, ki_h
    thc_hc = kp_h
    if break_at == "theta_c":                              # theta_c replaced by u_brk
        thc_y_x, thc_x, thc_hc, thc_u = thc_x.copy(), np.zeros(n), 0.0, 1.0
    else:
        thc_u = 0.0
    de_x = kp_th * thc_x; de_x[2] -= kp_th; de_x[3] -= kd_th   # de_cmd = kp(th_c - th) - kd q
    de_hc, de_u = kp_th * thc_hc, kp_th * thc_u
    dt_x = np.zeros(n); dt_x[0], dt_x[7] = -kp_V, ki_V     # dt = kp_V (V_c - Vt) + ki_V z_V
    dt_vc, dt_u = kp_V, 0.0
    if break_at == "elevator":
        brk_row, de_x, de_hc, de_u = de_x.copy(), np.zeros(n), 0.0, 1.0
    if break_at == "throttle":
        brk_row, dt_x, dt_vc, dt_u = dt_x.copy(), np.zeros(n), 0.0, 1.0
    if break_at == "theta_c":
        brk_row = thc_y_x
    # actuator: de_act' = 40 (de_cmd - de_act)
    Acl[5, :] += 40 * de_x; Acl[5, 5] -= 40
    Bcl[5, 0] += 40 * de_hc; Bcl[5, 2] += 40 * de_u
    # airframe inputs
    Acl[:4, 5] += B[:, 0]
    Acl[:4, :] += np.outer(B[:, 1], dt_x)
    Bcl[:4, 1] += B[:, 1] * dt_vc; Bcl[:4, 2] += B[:, 1] * dt_u
    # integrators
    Acl[6, 4] = -1; Bcl[6, 0] = 1                           # z_h' = h_c - h
    Acl[7, 0] = -1; Bcl[7, 1] = 1                           # z_V' = V_c - Vt
    Ccl[0, 4] = Ccl[1, 0] = Ccl[2, 2] = 1
    if break_at:
        Ccl[3, :] = brk_row
    return control.ss(Acl, Bcl, Ccl, np.zeros((4, 3)))


CL = closed_loop()
print(f"\nclosed-loop poles (all loops): max real part {max(CL.poles().real):+.3f}")
results = {}
for name, brk in (("elevator (inner)", "elevator"), ("theta_c (altitude)", "theta_c"), ("throttle (airspeed)", "throttle")):
    sysb = closed_loop(brk)
    L = -control.tf(sysb[3, 2])
    m = loop_margins(L)
    results[name] = m.as_dict()
    print(f"loop at {name:22s} GM {m.gain_margin_db:6.1f} dB  PM {m.phase_margin_deg:5.1f} deg  "
          f"wc {m.w_gain_crossover:6.3f} rad/s  delay margin {1000 * m.delay_margin_s:6.0f} ms")

t = np.linspace(0, 40, 4001)
fig, ax = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
for col, (inp, label) in enumerate(((0, "altitude command step"), (1, "airspeed command step"))):
    tt, y = control.step_response(CL[:3, inp], t)
    y = np.squeeze(y)
    tgt = 0 if inp == 0 else 1
    ax[0, col].plot(tt, y[tgt], label="h / h_c" if inp == 0 else "V / V_c")
    ax[0, col].plot(tt, y[1 - tgt], "--", label="V / h_c (coupling)" if inp == 0 else "h / V_c (coupling)")
    ax[1, col].plot(tt, np.degrees(y[2]), color="C2", label="theta [deg per unit cmd]")
    mtr = step_metrics(tt, y[tgt], 1.0, 0.0, band=0.05)
    print(f"linear {label:22s}: rise {mtr.rise_time:5.2f} s, overshoot {mtr.overshoot_pct:5.1f} %, "
          f"settling(5%) {mtr.settling_time:5.1f} s, peak cross-coupling {np.max(np.abs(y[1 - tgt])):.3f}")
    ax[0, col].set_title(f"{label}: OS {mtr.overshoot_pct:.0f}%, ts(5%) {mtr.settling_time:.1f} s")
    for a_ in ax[:, col]:
        a_.grid(alpha=0.3)
        a_.legend(fontsize="small")
ax[1, 0].set_xlabel("time [s]")
ax[1, 1].set_xlabel("time [s]")
fig.tight_layout()
save(fig, "09_longitudinal_autopilot", "linear_design_steps", show=args.show)

gains = {"ap/gains/kp-theta": kp_th, "ap/gains/kd-theta": kd_th,
         "ap/gains/kp-h": kp_h, "ap/gains/ki-h": ki_h,
         "ap/gains/kp-v": kp_V, "ap/gains/ki-v": ki_V}
(OUTPUT_DIR / "09_longitudinal_autopilot").mkdir(parents=True, exist_ok=True)
(OUTPUT_DIR / "09_longitudinal_autopilot" / "gains.json").write_text(json.dumps(gains, indent=2))
print("\nXML-ready gains:")
for k, v in gains.items():
    print(f'    <property value="{v:.6g}"> {k} </property>')
