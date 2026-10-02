"""Module 01 - exploring the property tree.

    python modules/01_first_simulation/solutions/property_explorer.py
"""

from gnclab.cli import parse_args

args = parse_args(__doc__)

from gnclab import initialize, make_fdm  # noqa: E402

fdm = make_fdm("c172x")

# --- How big is the tree? -------------------------------------------------
catalog = fdm.get_property_catalog()       # list of "name (R)" / "name (RW)"
print(f"c172x exposes {len(catalog)} properties")
top = {}
for entry in catalog:
    root = entry.split("/")[0]
    top[root] = top.get(root, 0) + 1
for root, n in sorted(top.items(), key=lambda kv: -kv[1]):
    print(f"  {root:<14s} {n:4d}")

# --- Search it (like grep) -----------------------------------------------
print("\nProperties matching 'alpha':")
print(fdm.query_property_catalog("alpha"))

# --- ic/ vs. state: ic/ only matters at run_ic() ------------------------
initialize(fdm, {"h-sl-ft": 3000, "vc-kts": 90})
print(f"ic/h-sl-ft = {fdm['ic/h-sl-ft']:.0f}, position/h-sl-ft = {fdm['position/h-sl-ft']:.0f}")
fdm["ic/h-sl-ft"] = 9000          # changes nothing until the next run_ic()
fdm.run()
print(f"after writing ic/h-sl-ft=9000 and one step: position/h-sl-ft = {fdm['position/h-sl-ft']:.0f}")

# --- Commands vs. positions: the FCS sits in between --------------------
fdm["fcs/elevator-cmd-norm"] = -0.5      # pilot pulls (nose up)
print(f"\nelevator cmd -0.5 -> pos before run(): {fdm['fcs/elevator-pos-rad']:+.4f} rad")
fdm.run()
print(f"                   -> pos after 1 step: {fdm['fcs/elevator-pos-rad']:+.4f} rad "
      f"({fdm['fcs/elevator-pos-deg']:+.2f} deg)")
print("The FCS (aircraft file <flight_control>) maps the normalized command to a"
      " surface angle in radians; you'll read that mapping in Module 04.")

# --- Units live in the name ---------------------------------------------
for p in ["velocities/vc-kts", "velocities/vtrue-fps", "velocities/vt-fps",
          "velocities/mach", "aero/qbar-psf", "atmosphere/rho-slugs_ft3",
          "atmosphere/T-R", "inertia/weight-lbs", "inertia/mass-slugs"]:
    print(f"  {p:<28s} = {fdm[p]:.5g}")
