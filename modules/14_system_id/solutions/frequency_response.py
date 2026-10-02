"""Module 14 - frequency-domain identification from the elevator sweep.

H(f) = S_uy / S_uu (cross-/auto-spectra, Welch averaging) for
u = measured elevator [rad], y = measured pitch rate [rad/s], with the
coherence gamma^2 telling you where to trust it (> ~0.6).  Compared with the
q/delta_e Bode plot of the JSBSim linear model (linearize_fd) - the
"truth" here.  This is how flight-test teams validate a model's dynamics and
measure a closed-loop system's bandwidth and margins.

    python modules/14_system_id/solutions/frequency_response.py [--show]
"""

import sys
from pathlib import Path

from gnclab.cli import parse_args

args = parse_args(__doc__)

import control  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from gnclab import OUTPUT_DIR, initialize, make_fdm  # noqa: E402
from gnclab.linear import linearize_fd  # noqa: E402
from gnclab.plotting import save  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
from flight_test import ensure_logs  # noqa: E402

ensure_logs()
from gnclab.sysid import frf  # noqa: E402
from gnclab.trim import trim  # noqa: E402

d = pd.read_csv(OUTPUT_DIR / "14_system_id" / "long_sweep.csv", index_col="t")
fs = 1 / (d.index[1] - d.index[0])
u = (d.de - d.de.iloc[:150].mean()).to_numpy()
y = (d.q - d.q.iloc[:150].mean()).to_numpy()
f, H, coh = frf(u, y, fs, nperseg=1024)

fdm = make_fdm("gnc_trainer")
initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 500, "vt-fps": 25 / 0.3048})
trim(fdm, "full")
lin = linearize_fd(fdm, ["fcs/elevator-cmd-norm"]).subsystem(["Vt", "Alpha", "Theta", "Q"], ["fcs/elevator-cmd-norm"])
G = control.tf(control.ss(lin.A, lin.B / np.radians(25), np.array([[0, 0, 0, 1.0]]), 0.0))  # per rad of surface
w = 2 * np.pi * f[1:]
Gm = np.array([complex(G(1j * wi)) for wi in w])

good = (coh[1:] > 0.6) & (f[1:] >= 0.15) & (f[1:] <= 4.0)
mag_err = 20 * np.log10(np.abs(H[1:][good]) / np.abs(Gm[good]))
ph_err = np.degrees(np.angle(H[1:][good] / Gm[good]))
print(f"{good.sum()} frequency points with coherence > 0.6 between 0.15 and 4 Hz")
print(f"identified vs model: magnitude error mean {mag_err.mean():+.2f} dB (max |{np.abs(mag_err).max():.2f}| dB), "
      f"phase error mean {ph_err.mean():+.1f} deg (max |{np.abs(ph_err).max():.1f}| deg)")
i_pk = np.argmax(np.abs(Gm))
print(f"model peak |q/de| at {w[i_pk] / 2 / np.pi:.2f} Hz (short period ~ {w[i_pk]:.1f} rad/s)")

fig, ax = plt.subplots(3, 1, sharex=True, figsize=(9, 9))
ax[0].semilogx(f[1:], 20 * np.log10(np.abs(H[1:])), ".", label="identified (sweep)")
ax[0].semilogx(f[1:], 20 * np.log10(np.abs(Gm)), label="JSBSim linear model")
ax[1].semilogx(f[1:], np.degrees(np.unwrap(np.angle(H[1:]))), ".")
ax[1].semilogx(f[1:], np.degrees(np.unwrap(np.angle(Gm))))
ax[2].semilogx(f[1:], coh[1:], ".")
ax[2].axhline(0.6, color="k", ls=":")
ax[0].set(ylabel="|q/de| [dB]", title="Pitch-rate frequency response from a flight-test sweep")
ax[1].set(ylabel="phase [deg]")
ax[2].set(ylabel="coherence", xlabel="frequency [Hz]", ylim=(0, 1.05))
ax[0].legend()
for a_ in ax:
    a_.grid(alpha=0.3, which="both")
    a_.set_xlim(0.1, 6)
fig.tight_layout()
save(fig, "14_system_id", "frequency_response", show=args.show)
