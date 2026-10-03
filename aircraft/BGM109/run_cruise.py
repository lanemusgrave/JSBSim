"""Fly the BGM-109 model. Cruise is the validated case.

    python3 run_cruise.py
    python3 run_cruise.py launch
"""
import os
import sys
import jsbsim

ROOT = os.path.dirname(os.path.abspath(__file__))


def make_fdm():
    fdm = jsbsim.FGFDMExec(os.path.dirname(ROOT))
    fdm.set_aircraft_path(os.path.dirname(ROOT))
    fdm.set_engine_path(os.path.join(ROOT, "Engines"))
    fdm.load_model("BGM109")
    fdm.set_dt(1.0 / 120.0)
    return fdm


def cruise(seconds=120.0, altitude=400.0, mach=0.72, heading=90.0):
    fdm = make_fdm()
    fdm.load_ic(os.path.join(ROOT, "reset_cruise.xml"), False)
    fdm["guidance/altitude-setpoint-ft"] = altitude
    fdm["guidance/mach-setpoint"] = mach
    fdm["guidance/heading-setpoint-deg"] = heading
    fdm["fcs/launch-mode"] = 0.0
    fdm.run_ic()
    fdm["propulsion/engine[0]/set-running"] = 1
    fdm["propulsion/cutoff_cmd"] = 0
    print("t_s   h_ft   Mach  alpha  phi   psi   thrust  fuel")
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
                f"{fdm['attitude/psi-deg']:6.1f} "
                f"{fdm['propulsion/engine[0]/thrust-lbs']:7.1f} "
                f"{fdm['propulsion/total-fuel-lbs']:7.1f}"
            )
    return fdm


def launch(seconds=40.0):
    fdm = make_fdm()
    fdm.load_ic(os.path.join(ROOT, "reset_launch.xml"), False)
    fdm["fcs/launch-mode"] = 1.0
    fdm["guidance/altitude-setpoint-ft"] = 600.0
    fdm["guidance/mach-setpoint"] = 0.72
    fdm["guidance/heading-setpoint-deg"] = 90.0
    fdm["inertia/pointmass-weight-lbs"] = 550.0
    fdm.run_ic()
    fdm["propulsion/engine[0]/set-running"] = 1
    fdm["fcs/launch-mode"] = 1.0
    print("t_s   h_ft   Mach  theta  phi   vt    wings  boost  weight")
    n = int(seconds / fdm.get_delta_t())
    for i in range(n):
        fdm.run()
        if i % 240 == 0:
            print(
                f"{fdm['simulation/sim-time-sec']:6.1f} "
                f"{fdm['position/h-sl-ft']:7.1f} "
                f"{fdm['velocities/mach']:6.3f} "
                f"{fdm['attitude/theta-deg']:6.1f} "
                f"{fdm['attitude/phi-deg']:6.1f} "
                f"{fdm['velocities/vt-fps']:6.0f} "
                f"{fdm['fcs/wings-deployed']:5.0f} "
                f"{fdm['fcs/booster-command']:5.0f} "
                f"{fdm['inertia/weight-lbs']:7.0f}"
            )
    return fdm


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "launch":
        launch()
    else:
        cruise()
