"""Module 10 exercises - solutions for B (sideslip feedback) and C (bank limit).

For B, the modified autopilot is generated on the fly into outputs/ from the
reference XML (so this solution can't drift from it)."""

from gnclab.cli import parse_args

args = parse_args(__doc__)

import numpy as np  # noqa: E402

from gnclab import AIRCRAFT_DIR, OUTPUT_DIR, initialize, make_fdm, run  # noqa: E402
from gnclab.autopilot import engage_lateral, engage_longitudinal  # noqa: E402
from gnclab.trim import trim  # noqa: E402

REF = AIRCRAFT_DIR / "gnc_trainer" / "fcs" / "gnc_autopilot.xml"
MOD_DIR = OUTPUT_DIR / "10_lateral_autopilot" / "fcs_beta"
MOD_DIR.mkdir(parents=True, exist_ok=True)
xml = REF.read_text()
old = """    <pure_gain name="ap/dr-cmd">
      <input> ap/r-washed </input>
      <gain> ap/gains/kr-yaw </gain>
      <clipto> <min> -0.5 </min> <max> 0.5 </max> </clipto>
    </pure_gain>"""
new = """    <pure_gain name="ap/dr-r">    <input> ap/r-washed </input>   <gain> ap/gains/kr-yaw </gain> </pure_gain>
    <pure_gain name="ap/dr-beta"> <input> aero/beta-rad </input> <gain> ap/gains/kbeta </gain> </pure_gain>
    <summer name="ap/dr-cmd">                <!-- dr = kr washout(r) - kbeta beta -->
      <input> ap/dr-r </input>
      <input> -ap/dr-beta </input>
      <clipto> <min> -0.5 </min> <max> 0.5 </max> </clipto>
    </summer>"""
assert old in xml
xml = xml.replace(old, new)
# NB: <property> declarations belong at the <system> level, not inside a <channel>
decl = '<property value="0.23">     ap/gains/kr-yaw </property>'
assert decl in xml
xml = xml.replace(decl, decl + '\n  <property value="0">        ap/gains/kbeta </property>')
(MOD_DIR / "gnc_autopilot.xml").write_text(xml)


def course_change(systems_dir=None, **props):
    fdm = make_fdm("gnc_trainer", systems_dir=systems_dir)
    initialize(fdm, {"lat-geod-deg": 33.7, "h-sl-ft": 330, "vt-fps": 25 / 0.3048, "psi-true-deg": 0})
    trim(fdm, "full")
    for k, v in props.items():
        fdm[k] = v
    engage_longitudinal(fdm)
    engage_lateral(fdm)
    df = run(fdm, 30.0, {"chi": "flight-path/psi-gt-rad", "beta": "aero/beta-deg", "h": "position/h-sl-ft",
                         "phi": "attitude/phi-deg"},
             callback=lambda f, t: f.__setitem__("ap/chi-cmd-rad", np.radians(90)) if t >= 2 else None,
             record_every=6)
    chi = np.degrees(np.unwrap(df.chi.to_numpy()))
    t90 = df.index[np.argmax(chi >= 88)]
    return df, t90


print("B. sideslip feedback")
for kb in (0.0, 1.0, 2.0):
    df, t90 = course_change(MOD_DIR, **{"ap/gains/kbeta": kb})
    print(f"   kbeta = {kb:.1f}: peak |beta| {df.beta.abs().max():.2f} deg, 88 deg reached at {t90:.1f} s")
print("C. bank limit")
for phimax in (30, 45):
    df, t90 = course_change(**{"ap/phi-max-rad": np.radians(phimax)})
    print(f"   phi_max = {phimax} deg: 88 deg reached at {t90:.1f} s, altitude excursion "
          f"{(df.h - df.h.iloc[0]).abs().max():.1f} ft, peak bank {df.phi.abs().max():.1f} deg")
