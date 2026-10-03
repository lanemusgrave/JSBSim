"""Fly the AGM-158B. Default is an air release at 25,000 ft.

    python3 run_release.py
    python3 run_release.py cruise
"""
import os
import sys
import jsbsim

ROOT = os.path.dirname(os.path.abspath(__file__))


def make_fdm():
    fdm = jsbsim.FGFDMExec(os.path.dirname(ROOT))
    fdm.set_aircraft_path(os.path.dirname(ROOT))
    fdm.set_engine_path(os.path.join(ROOT, "Engines"))
    fdm.load_model("AGM158")
    fdm.set_dt(1.0 / 120.0)
    return fdm


def release(seconds=420.0, altitude=500.0, mach=0.80, heading=90.0):
    fdm = make_fdm()
    fdm.load_ic(os.path.join(ROOT, "reset_release.xml"), False)
    fdm["fcs/release-mode"] = 1.0
    fdm["guidance/altitude-setpoint-ft"] = altitude
    fdm["guidance/mach-setpoint"] = mach
    fdm["guidance/heading-setpoint-deg"] = heading
    fdm.run_ic()
    fdm["propulsion/engine[0]/set-running"] = 1
    fdm["propulsion/cutoff_cmd"] = 0
    fdm["fcs/release-mode"] = 1.0
    print("t_s    h_ft   Mach  alpha  gamma  phi   wings  fuel")
    n = int(seconds / fdm.get_delta_t())
    for i in range(n):
        fdm.run()
        if i % 1200 == 0:
            print(
                f"{fdm['simulation/sim-time-sec']:6.1f} "
                f"{fdm['position/h-sl-ft']:8.0f} "
                f"{fdm['velocities/mach']:6.3f} "
                f"{fdm['aero/alpha-deg']:6.2f} "
                f"{fdm['flight-path/gamma-deg']:6.2f} "
                f"{fdm['attitude/phi-deg']:6.1f} "
                f"{fdm['fcs/wings-deployed']:5.0f} "
                f"{fdm['propulsion/total-fuel-lbs']:6.1f}"
            )
    return fdm


def cruise(seconds=90.0, altitude=500.0, mach=0.80, heading=90.0):
    fdm = make_fdm()
    fdm.load_ic(os.path.join(ROOT, "reset_cruise.xml"), False)
    fdm["fcs/release-mode"] = 0.0
    fdm["guidance/altitude-setpoint-ft"] = altitude
    fdm["guidance/mach-setpoint"] = mach
    fdm["guidance/heading-setpoint-deg"] = heading
    fdm.run_ic()
    fdm["propulsion/engine[0]/set-running"] = 1
    print("t_s   h_ft   Mach  alpha  phi   thrust  fuel")
    n = int(seconds / fdm.get_delta_t())
    for i in range(n):
        fdm.run()
        if i % 600 == 0:
            print(
                f"{fdm['simulation/sim-time-sec']:6.1f} "
                f"{fdm['position/h-sl-ft']:7.1f} "
                f"{fdm['velocities/mach']:6.3f} "
                f"{fdm['aero/alpha-deg']:6.2f} "
                f"{fdm['attitude/phi-deg']:6.1f} "
                f"{fdm['propulsion/engine[0]/thrust-lbs']:7.1f} "
                f"{fdm['propulsion/total-fuel-lbs']:6.1f}"
            )
    return fdm


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "cruise":
        cruise()
    else:
        release()
