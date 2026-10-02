"""Module 10 - design the lateral-directional autopilot (B&M sec. 6.3).

  roll hold     da = kp_phi (phi_c - phi) - kd_phi p              (PD; phi/da ~ a2 / (s (s + a1)))
  course hold   phi_c = kp_chi e_chi + ki_chi int(e_chi)          (PI; chi/phi ~ (g/V) / s)
  yaw damper    dr = kr * washout(r),  washout = s / (s + 1/tau)  (damps Dutch roll, not steady turns)

Checked on the full 4-state lateral model + actuators with ALL loops closed;
margins measured with one loop broken at a time.

    python modules/10_lateral_autopilot/solutions/design_lateral.py [--show]
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

G0 = 32.174
fdm = make_fdm("gnc_trainer")
initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048})
trim(fdm, "full")
Vg = fdm["velocities/vg-fps"]
lin = linearize_fd(fdm, ["fcs/aileron-cmd-norm", "fcs/rudder-cmd-norm"]).subsystem(
    ["Beta", "Phi", "P", "R"], ["fcs/aileron-cmd-norm", "fcs/rudder-cmd-norm"])
A, B = lin.A, lin.B
iB, iPhi, iP, iR = range(4)

# ---- roll ---------------------------------------------------------------------
a_phi1, a_phi2 = -A[iP, iP], B[iP, 0]
DA_MAX, E_PHI_MAX, ZETA_PHI = 1.0, np.radians(40.0), 0.8
kp_phi = DA_MAX / E_PHI_MAX * np.sign(a_phi2)
w_phi = np.sqrt(abs(a_phi2) * abs(kp_phi))
kd_phi = max(0.0, (2 * ZETA_PHI * w_phi - a_phi1) / a_phi2)   # B&M: if negative, the airframe's own
print(f"roll: a1 = {a_phi1:.2f} (= -Lp), a2 = {a_phi2:.2f}; kp = {kp_phi:.3f}, kd = {kd_phi:.3f} "
      f"-> w = {w_phi:.2f} rad/s")                               # roll damping already suffices

# ---- course -------------------------------------------------------------------
W_CHI, ZETA_CHI = 25.0, 1.5   # chosen from a small sweep (see README)
w_chi = w_phi / W_CHI
kp_chi = 2 * ZETA_CHI * w_chi * Vg / G0
ki_chi = w_chi ** 2 * Vg / G0
print(f"course: kp = {kp_chi:.3f}, ki = {ki_chi:.4f} -> w = {w_chi:.3f} rad/s")

# ---- yaw damper: sweep kr for Dutch-roll damping ------------------------------------
TAU_WO = 1.0


def dutch_roll_zeta(kr):
    """Dutch-roll damping with rudder = kr * washout(r) (plus 40 rad/s actuator)."""
    # states: [beta, phi, p, r, x_wo, dr_act]; washout: x_wo' = -x_wo/tau + r ... output r - x_wo/tau
    n = 6
    Acl = np.zeros((n, n))
    Acl[:4, :4] = A
    Acl[4, 3], Acl[4, 4] = 1.0, -1.0 / TAU_WO                 # x_wo' = r - x_wo/tau
    # washout output y = r - x_wo/tau  (= s/(s+1/tau) r)
    Acl[5, 3], Acl[5, 4], Acl[5, 5] = 40 * kr, -40 * kr / TAU_WO, -40.0
    Acl[:4, 5] = B[:, 1]
    lam = np.linalg.eigvals(Acl)
    osc = lam[(lam.imag > 0.5) & (abs(lam) < 15)]
    return float(-osc.real[0] / abs(osc[0])) if len(osc) else 1.0


krs = np.linspace(0, 0.6, 61)
zs = [dutch_roll_zeta(k) for k in krs]
kr = float(krs[np.argmin(np.abs(np.array(zs) - 0.5))])
print(f"yaw damper: kr = {kr:.3f} rudder norm per rad/s, washout tau = {TAU_WO} s "
      f"-> Dutch roll zeta {zs[0]:.2f} -> {dutch_roll_zeta(kr):.2f}")


# ---- full closed loop -------------------------------------------------------------
def closed_loop(break_at=None):
    """States [beta, phi, p, r, psi, da_act, dr_act, z_chi, x_wo]; inputs [chi_c, u_brk];
    outputs [psi, phi, beta, y_brk].  Course ~ heading (no wind): psi_dot = r."""
    n = 9
    Acl, Bcl, Ccl = np.zeros((n, n)), np.zeros((n, 2)), np.zeros((4, n))
    Acl[:4, :4] = A
    Acl[4, 3] = 1.0                                          # psi_dot = r
    phic_x = np.zeros(n); phic_x[4], phic_x[7] = -kp_chi, ki_chi
    phic_c, phic_u = kp_chi, 0.0
    if break_at == "phi_c":
        brk, phic_x, phic_c, phic_u = phic_x.copy(), np.zeros(n), 0.0, 1.0
    da_x = kp_phi * phic_x; da_x[1] -= kp_phi; da_x[2] -= kd_phi
    da_c, da_u = kp_phi * phic_c, kp_phi * phic_u
    dr_x = np.zeros(n); dr_x[3], dr_x[8] = kr, -kr / TAU_WO
    dr_u = 0.0
    if break_at == "aileron":
        brk, da_x, da_c, da_u = da_x.copy(), np.zeros(n), 0.0, 1.0
    if break_at == "rudder":
        brk, dr_x, dr_u = dr_x.copy(), np.zeros(n), 1.0
    Acl[5, :] += 40 * da_x; Acl[5, 5] -= 40; Bcl[5, 0] += 40 * da_c; Bcl[5, 1] += 40 * da_u
    Acl[6, :] += 40 * dr_x; Acl[6, 6] -= 40; Bcl[6, 1] += 40 * dr_u
    Acl[:4, 5] += B[:, 0]
    Acl[:4, 6] += B[:, 1]
    Acl[7, 4] = -1.0; Bcl[7, 0] = 1.0                        # z_chi' = chi_c - psi
    Acl[8, 3], Acl[8, 8] = 1.0, -1.0 / TAU_WO                # washout state
    Ccl[0, 4] = Ccl[1, 1] = Ccl[2, 0] = 1.0
    if break_at:
        Ccl[3, :] = brk
    return control.ss(Acl, Bcl, Ccl, np.zeros((4, 2)))


CL = closed_loop()
print(f"\nclosed-loop poles: max real part {max(CL.poles().real):+.4f}")
margins = {}
for name, brk in (("aileron (roll)", "aileron"), ("phi_c (course)", "phi_c"), ("rudder (yaw damper)", "rudder")):
    L = -control.tf(closed_loop(brk)[3, 1])
    m = loop_margins(L)
    margins[name] = m.as_dict()
    print(f"loop at {name:20s} GM {m.gain_margin_db:6.1f} dB  PM {m.phase_margin_deg:6.1f} deg  "
          f"wc {m.w_gain_crossover:6.3f} rad/s  delay margin {1000 * m.delay_margin_s:6.0f} ms")

t = np.linspace(0, 30, 3001)
tt, y = control.step_response(CL[:3, 0], t)
y = np.squeeze(y)
mtr = step_metrics(tt, y[0], 1.0, 0.0, band=0.05)
print(f"linear course step: rise {mtr.rise_time:.2f} s, overshoot {mtr.overshoot_pct:.1f} %, "
      f"settling(5%) {mtr.settling_time:.1f} s; peak beta per rad of course {np.max(np.abs(y[2])):.3f}")

fig, ax = plt.subplots(1, 2, figsize=(12, 4))
ax[0].plot(krs, zs)
ax[0].axvline(kr, color="k", ls=":")
ax[0].set(xlabel="kr [rudder norm per rad/s]", ylabel="Dutch-roll damping", title="yaw damper gain")
ax[1].plot(tt, y[0], label="course / course_c")
ax[1].plot(tt, y[1], label="phi [rad per rad]")
ax[1].plot(tt, y[2], label="beta [rad per rad]")
ax[1].set(xlabel="time [s]", title="linear course step (small-angle)")
ax[1].legend()
for a_ in ax:
    a_.grid(alpha=0.3)
fig.tight_layout()
save(fig, "10_lateral_autopilot", "linear_design", show=args.show)

gains = {"ap/gains/kp-phi": kp_phi, "ap/gains/kd-phi": kd_phi, "ap/gains/kp-chi": kp_chi,
         "ap/gains/ki-chi": ki_chi, "ap/gains/kr-yaw": kr, "ap/gains/washout-c1": 1 / TAU_WO}
(OUTPUT_DIR / "10_lateral_autopilot").mkdir(parents=True, exist_ok=True)
(OUTPUT_DIR / "10_lateral_autopilot" / "gains.json").write_text(json.dumps(gains, indent=2))
print("\nXML-ready gains:")
for k, v in gains.items():
    print(f'    <property value="{v:.6g}"> {k} </property>')
