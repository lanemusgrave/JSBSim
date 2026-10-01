"""Module 07 - generate the gnc_trainer JSBSim model from a parameter set.

Real aero databases are *generated*: wind-tunnel / CFD / flight-test data go
through scripts that write the simulation's tables.  This file does the same
for a small fixed-wing UAS using the published Aerosonde-class parameters in
Beard & McLain, "Small Unmanned Aircraft: Theory and Practice" (2012) and the
companion code (github.com/randybeard/mavsim_public, parameters/aerosonde_parameters.py).

Outputs (overwritten every run):
  aircraft/gnc_trainer/gnc_trainer.xml           airframe, mass, aero, propulsion, FCS
  aircraft/gnc_trainer/Engines/gnc_dummy_motor.xml zero-power placeholder engine (see below)
  aircraft/gnc_trainer/Engines/gnc_direct.xml      its "direct" thruster

Deliberate differences from the textbook model (documented in the XML):
  * drag due to elevator uses |delta_e| (the book's signed term would make
    up-elevator *reduce* drag);
  * none for propulsion: the book's DC-motor + propeller model is implemented
    exactly, as an algebraic <system> feeding an external thrust force and
    torque moment.  Why not JSBSim's <electric_engine> + <propeller>?  JSBSim's
    trim spins propellers up by time-marching with a fixed 0.5 s step, which is
    only stable for rotor spin-up time constants > ~0.25 s.  A small electric
    motor's is ~0.05 s, so the built-in model either blows up (NaN) or needs a
    rotor inertia ~25x too large - giving a 10-20 s throttle response and
    gyroscopic moments bigger than the aero moments.  A zero-power dummy engine
    is kept only so JSBSim's trim routine has a throttle to adjust.

    python modules/07_build_uas_model/solutions/build_gnc_trainer.py
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
OUT = REPO / "aircraft" / "gnc_trainer"

# --------------------------------------------------------------------------
# Parameters (SI) - Beard & McLain Aerosonde-class UAS
# --------------------------------------------------------------------------
P = dict(
    mass=11.0, Jx=0.8244, Jy=1.135, Jz=1.759, Jxz=0.1204,
    S=0.55, b=2.8956, c=0.18994, e=0.9,
    # longitudinal
    CL0=0.23, CLa=5.61, CLq=7.95, CLde=0.13,
    CDp=0.043, CDq=0.0, CDde=0.0135,
    Cm0=0.0135, Cma=-2.74, Cmq=-38.21, Cmde=-0.99,
    M=50.0, alpha0=0.47,                       # stall blending sharpness, stall alpha [rad]
    # lateral-directional
    CY0=0.0, CYb=-0.98, CYp=0.0, CYr=0.0, CYda=0.075, CYdr=0.19,
    Cl0=0.0, Clb=-0.13, Clp=-0.51, Clr=0.25, Clda=0.17, Cldr=0.0024,
    Cn0=0.0, Cnb=0.073, Cnp=0.069, Cnr=-0.095, Cnda=-0.011, Cndr=-0.069,
    # propeller (CT, CQ quadratic in advance ratio J = V/(n D))
    D_prop=0.508, CT2=-0.1079, CT1=-0.06044, CT0=0.09357,
    CQ2=-0.01664, CQ1=0.004970, CQ0=0.005230,
    KV_rpm_per_volt=145.0, R_motor=0.042, i0=1.5, V_max=44.4,   # 12-cell battery, B&M addendum
    # control surface limits [rad]
    de_max=math.radians(25), da_max=math.radians(25), dr_max=math.radians(25),
    # actuator model (used when fcs/actuators-on = 1)
    act_bw=40.0, act_rate=math.radians(250),
)

# Structural frame layout [m] (x aft from the nose, z up)
X_CG = 0.40
X_PROP = 0.0


def cl_static(alpha: np.ndarray) -> np.ndarray:
    """B&M eq. 4.9: linear lift blended into flat-plate lift past stall."""
    M, a0 = P["M"], P["alpha0"]
    e1, e2 = np.exp(-M * (alpha - a0)), np.exp(M * (alpha + a0))
    sigma = (1 + e1 + e2) / ((1 + e1) * (1 + e2))
    lin = P["CL0"] + P["CLa"] * alpha
    flat = 2 * np.sign(alpha) * np.sin(alpha) ** 2 * np.cos(alpha)
    return (1 - sigma) * lin + sigma * flat


def cd_static(alpha: np.ndarray) -> np.ndarray:
    """B&M eq. 4.11: parasitic + induced drag (using the *linear* CL)."""
    AR = P["b"] ** 2 / P["S"]
    return P["CDp"] + (P["CL0"] + P["CLa"] * alpha) ** 2 / (math.pi * P["e"] * AR)


def table_1d(var: str, xs, ys, indent: str) -> str:
    rows = "\n".join(f"{indent}    {x:9.4f} {y:10.5f}" for x, y in zip(xs, ys))
    return (f"{indent}<table>\n{indent}  <independentVar lookup=\"row\"> {var} </independentVar>\n"
            f"{indent}  <tableData>\n{rows}\n{indent}  </tableData>\n{indent}</table>")


def coeff_fn(name: str, desc: str, factors: list[str], value: float, ref: str = "") -> str:
    """A <function> = qbar * S * [ref length] * factors... * value."""
    props = ["aero/qbar-psf", "metrics/Sw-sqft"] + ([ref] if ref else []) + factors
    inner = "\n".join(f"          <property> {p} </property>" for p in props)
    return (f"      <function name=\"{name}\">\n        <description> {desc} </description>\n"
            f"        <product>\n{inner}\n          <value> {value:.6g} </value>\n        </product>\n"
            f"      </function>")


def build_aero() -> str:
    alphas = np.radians(np.r_[-90, -60, -40, -30, np.arange(-24, 25, 2), 30, 40, 60, 90])
    cl_tab = table_1d("aero/alpha-rad", alphas, cl_static(alphas), "          ")
    cd_tab = table_1d("aero/alpha-rad", alphas, cd_static(alphas), "          ")
    q_nd = ["aero/ci2vel", "velocities/q-aero-rad_sec"]
    p_nd = ["aero/bi2vel", "velocities/p-aero-rad_sec"]
    r_nd = ["aero/bi2vel", "velocities/r-aero-rad_sec"]
    beta = ["aero/beta-rad"]
    de, da, dr = ["fcs/elevator-pos-rad"], ["fcs/aileron-pos-rad"], ["fcs/rudder-pos-rad"]
    cb, bw = "metrics/cbarw-ft", "metrics/bw-ft"
    return f"""  <aerodynamics>
    <!-- All coefficients are referenced to the CG (AERORP = CG).  Forces in the
         wind frame (DRAG, SIDE, LIFT), moments in the body frame. -->

    <axis name="LIFT">
      <function name="aero/force/L_alpha">
        <description> CL(alpha) with stall: linear CL0 + CLa*alpha blended to flat plate (B&amp;M eq. 4.9) </description>
        <product>
          <property> aero/qbar-psf </property>
          <property> metrics/Sw-sqft </property>
{cl_tab}
        </product>
      </function>
{coeff_fn("aero/force/L_q", f"CLq = {P['CLq']}", q_nd, P["CLq"])}
{coeff_fn("aero/force/L_de", f"CLde = {P['CLde']}", de, P["CLde"])}
    </axis>

    <axis name="DRAG">
      <function name="aero/force/D_alpha">
        <description> CD(alpha) = CDp + (CL0 + CLa*alpha)^2/(pi e AR)  (B&amp;M eq. 4.11) </description>
        <product>
          <property> aero/qbar-psf </property>
          <property> metrics/Sw-sqft </property>
{cd_tab}
        </product>
      </function>
      <function name="aero/force/D_de">
        <description> CDde = {P['CDde']} * |delta_e| (the book's term is signed; ours is not) </description>
        <product>
          <property> aero/qbar-psf </property> <property> metrics/Sw-sqft </property>
          <abs> <property> fcs/elevator-pos-rad </property> </abs>
          <value> {P['CDde']} </value>
        </product>
      </function>
    </axis>

    <axis name="SIDE">
{coeff_fn("aero/force/Y_beta", f"CYb = {P['CYb']}", beta, P["CYb"])}
{coeff_fn("aero/force/Y_da", f"CYda = {P['CYda']}", da, P["CYda"])}
{coeff_fn("aero/force/Y_dr", f"CYdr = {P['CYdr']}", dr, P["CYdr"])}
    </axis>

    <axis name="PITCH">
      <function name="aero/moment/Cm0">
        <description> Cm0 = {P['Cm0']} </description>
        <product> <property> aero/qbar-psf </property> <property> metrics/Sw-sqft </property>
                  <property> {cb} </property> <value> {P['Cm0']} </value> </product>
      </function>
{coeff_fn("aero/moment/Cm_alpha", f"Cma = {P['Cma']}", ["aero/alpha-rad"], P["Cma"], cb)}
{coeff_fn("aero/moment/Cm_q", f"Cmq = {P['Cmq']}", q_nd, P["Cmq"], cb)}
{coeff_fn("aero/moment/Cm_de", f"Cmde = {P['Cmde']}", de, P["Cmde"], cb)}
    </axis>

    <axis name="ROLL">
{coeff_fn("aero/moment/Cl_beta", f"Clb = {P['Clb']}", beta, P["Clb"], bw)}
{coeff_fn("aero/moment/Cl_p", f"Clp = {P['Clp']}", p_nd, P["Clp"], bw)}
{coeff_fn("aero/moment/Cl_r", f"Clr = {P['Clr']}", r_nd, P["Clr"], bw)}
{coeff_fn("aero/moment/Cl_da", f"Clda = {P['Clda']}", da, P["Clda"], bw)}
{coeff_fn("aero/moment/Cl_dr", f"Cldr = {P['Cldr']}", dr, P["Cldr"], bw)}
    </axis>

    <axis name="YAW">
{coeff_fn("aero/moment/Cn_beta", f"Cnb = {P['Cnb']}", beta, P["Cnb"], bw)}
{coeff_fn("aero/moment/Cn_p", f"Cnp = {P['Cnp']}", p_nd, P["Cnp"], bw)}
{coeff_fn("aero/moment/Cn_r", f"Cnr = {P['Cnr']}", r_nd, P["Cnr"], bw)}
{coeff_fn("aero/moment/Cn_da", f"Cnda = {P['Cnda']}", da, P["Cnda"], bw)}
{coeff_fn("aero/moment/Cn_dr", f"Cndr = {P['Cndr']}", dr, P["Cndr"], bw)}
    </axis>
  </aerodynamics>"""


def surface_channel(name: str, cmd: str, trim: str, out: str, limit: float) -> str:
    return f"""    <channel name="{name}">
      <summer name="fcs/{out}-sum">
        <input> {cmd} </input>
        <input> {trim} </input>
        <input> ap/{out}-cmd-norm </input>
        <clipto> <min> -1 </min> <max> 1 </max> </clipto>
      </summer>
      <aerosurface_scale name="fcs/{out}-cmd-rad">
        <input> fcs/{out}-sum </input>
        <range> <min> {-limit:.5f} </min> <max> {limit:.5f} </max> </range>
      </aerosurface_scale>
      <!-- Actuator model: first-order lag + rate limit + travel limit. -->
      <actuator name="fcs/{out}-actuator">
        <input> fcs/{out}-cmd-rad </input>
        <lag> {P['act_bw']} </lag>
        <rate_limit> {P['act_rate']:.4f} </rate_limit>
        <clipto> <min> {-limit:.5f} </min> <max> {limit:.5f} </max> </clipto>
      </actuator>
      <!-- fcs/actuators-on = 0 (default): ideal surfaces (needed for
           finite-difference linearization).  1: realistic actuators. -->
      <switch name="fcs/{out}-pos-select">
        <default value="fcs/{out}-cmd-rad"/>
        <test value="fcs/{out}-actuator"> fcs/actuators-on == 1 </test>
        <output> fcs/{out}-pos-rad </output>
      </switch>
    </channel>"""


def build_aircraft() -> str:
    m2in = 39.3701
    return f"""<?xml version="1.0"?>
<!--
  gnc_trainer: a ~11 kg, 2.9 m span fixed-wing UAS for the JSBSim GNC curriculum.
  GENERATED by modules/07_build_uas_model/solutions/build_gnc_trainer.py - edit the
  generator, not this file.

  Data: Aerosonde-class parameters from Beard & McLain, "Small Unmanned Aircraft:
  Theory and Practice" (Princeton, 2012), as published in mavsim_public.
  Structural frame: origin at the propeller hub, x aft, y right, z up.
-->
<fdm_config name="gnc_trainer" version="2.0" release="BETA">

  <fileheader>
    <author> JSBSim GNC Lab (generated) </author>
    <description> Small fixed-wing UAS, Aerosonde-class (Beard &amp; McLain) </description>
    <reference refID="BM2012" author="R. Beard, T. McLain" title="Small Unmanned Aircraft: Theory and Practice" date="2012"/>
  </fileheader>

  <metrics>
    <wingarea unit="M2"> {P['S']} </wingarea>
    <wingspan unit="M"> {P['b']} </wingspan>
    <chord unit="M"> {P['c']} </chord>
    <location name="AERORP" unit="M"> <x> {X_CG} </x> <y> 0 </y> <z> 0 </z> </location>
    <location name="VRP" unit="M"> <x> {X_CG} </x> <y> 0 </y> <z> 0 </z> </location>
  </metrics>

  <!-- negated_crossproduct_inertia="false": ixz = +integral(x z dm), the
       textbook (Beard & McLain) definition.  JSBSim's default ("true") expects
       the opposite sign - a classic source of wrong roll/yaw coupling. -->
  <mass_balance negated_crossproduct_inertia="false">
    <ixx unit="KG*M2"> {P['Jx']} </ixx>
    <iyy unit="KG*M2"> {P['Jy']} </iyy>
    <izz unit="KG*M2"> {P['Jz']} </izz>
    <ixz unit="KG*M2"> {P['Jxz']} </ixz>
    <emptywt unit="KG"> {P['mass']} </emptywt>
    <location name="CG" unit="M"> <x> {X_CG} </x> <y> 0 </y> <z> 0 </z> </location>
    <pointmass name="PAYLOAD">
      <description> Movable payload/ballast for CG and weight dispersions (default 0) </description>
      <weight unit="KG"> 0.0 </weight>
      <location unit="M"> <x> {X_CG} </x> <y> 0 </y> <z> 0 </z> </location>
    </pointmass>
  </mass_balance>

  <ground_reactions>
    <!-- Belly-landing skids; the vehicle is air-launched in all lessons. -->
    <contact type="STRUCTURE" name="NOSE_SKID">
      <location unit="M"> <x> 0.10 </x> <y> 0 </y> <z> -0.12 </z> </location>
      <static_friction> 0.6 </static_friction> <dynamic_friction> 0.5 </dynamic_friction>
      <spring_coeff unit="N/M"> 8000 </spring_coeff> <damping_coeff unit="N/M/SEC"> 300 </damping_coeff>
    </contact>
    <contact type="STRUCTURE" name="TAIL_SKID">
      <location unit="M"> <x> 1.60 </x> <y> 0 </y> <z> -0.05 </z> </location>
      <static_friction> 0.6 </static_friction> <dynamic_friction> 0.5 </dynamic_friction>
      <spring_coeff unit="N/M"> 4000 </spring_coeff> <damping_coeff unit="N/M/SEC"> 150 </damping_coeff>
    </contact>
    <contact type="STRUCTURE" name="LEFT_TIP">
      <location unit="M"> <x> 0.45 </x> <y> {-P['b'] / 2:.3f} </y> <z> 0.05 </z> </location>
      <static_friction> 0.6 </static_friction> <dynamic_friction> 0.5 </dynamic_friction>
      <spring_coeff unit="N/M"> 4000 </spring_coeff> <damping_coeff unit="N/M/SEC"> 150 </damping_coeff>
    </contact>
    <contact type="STRUCTURE" name="RIGHT_TIP">
      <location unit="M"> <x> 0.45 </x> <y> {P['b'] / 2:.3f} </y> <z> 0.05 </z> </location>
      <static_friction> 0.6 </static_friction> <dynamic_friction> 0.5 </dynamic_friction>
      <spring_coeff unit="N/M"> 4000 </spring_coeff> <damping_coeff unit="N/M/SEC"> 150 </damping_coeff>
    </contact>
  </ground_reactions>

  <!-- Placeholder engine: produces nothing, but gives JSBSim's trim routine a
       throttle (fcs/throttle-cmd-norm) to adjust.  Real thrust is below. -->
  <propulsion>
    <engine file="gnc_dummy_motor">
      <feed> 0 </feed>
      <thruster file="gnc_direct">
        <location unit="M"> <x> {X_PROP} </x> <y> 0 </y> <z> 0 </z> </location>
        <orient unit="DEG"> <roll> 0 </roll> <pitch> 0 </pitch> <yaw> 0 </yaw> </orient>
      </thruster>
    </engine>
    <tank type="FUEL">
      <location unit="M"> <x> {X_CG} </x> <y> 0 </y> <z> 0 </z> </location>
      <capacity unit="KG"> 0.001 </capacity>
      <contents unit="KG"> 0.001 </contents>
    </tank>
  </propulsion>

  <!-- Thrust along the shaft and the motor reaction torque on the airframe. -->
  <external_reactions>
    <force name="propeller" frame="BODY">
      <function> <product> <property> propulsion/thrust-N </property> <value> 0.224809 </value> </product> </function>
      <location unit="M"> <x> {X_PROP} </x> <y> 0 </y> <z> 0 </z> </location>
      <direction> <x> 1 </x> <y> 0 </y> <z> 0 </z> </direction>
    </force>
    <moment name="prop_torque" frame="BODY">
      <function> <product> <property> propulsion/torque-Nm </property> <value> 0.737562 </value> </product> </function>
      <direction> <x> -1 </x> <y> 0 </y> <z> 0 </z> </direction>
    </moment>
  </external_reactions>

  <!-- ===== Avionics, executed in this order every frame =====
       (JSBSim runs ALL <system>s, in file order, before <flight_control>.)
       1. Systems/gnc_sensors.xml : sensor models; publishes fb/... feedback signals
       2. fcs/gnc_autopilot.xml   : autopilot + guidance; publishes ap/...-cmd-norm
       3. motor-prop system        : throttle sum + propulsion
       4. <flight_control>         : pilot + trim + autopilot commands -> actuators -> surfaces
       gnclab points JSBSim's systems path at aircraft/gnc_trainer/fcs/; pass
       make_fdm(..., systems_dir=...) to fly your own gnc_autopilot.xml instead. -->
  <system name="interfaces">
    <!-- Interface properties shared by the files below, declared once here. -->
    <property value="0"> fcs/actuators-on </property>
    <property value="0"> ap/elevator-cmd-norm </property>
    <property value="0"> ap/aileron-cmd-norm </property>
    <property value="0"> ap/rudder-cmd-norm </property>
    <property value="0"> ap/throttle-cmd-norm </property>
  </system>
  <system file="gnc_sensors"/>
  <system file="gnc_autopilot"/>

{build_motor_prop_system()}

  <flight_control name="gnc_trainer FCS">
    <!-- Normalized commands (-1..1, throttle 0..1) -> surface angles [rad].
         Pilot, trim and autopilot (ap/...) commands are summed.  Trim inputs
         must be included so JSBSim's trim routine can use them. -->
{surface_channel("Pitch", "fcs/elevator-cmd-norm", "fcs/pitch-trim-cmd-norm", "elevator", P["de_max"])}
{surface_channel("Roll", "fcs/aileron-cmd-norm", "fcs/roll-trim-cmd-norm", "aileron", P["da_max"])}
{surface_channel("Yaw", "fcs/rudder-cmd-norm", "fcs/yaw-trim-cmd-norm", "rudder", P["dr_max"])}
  </flight_control>

{build_aero()}

</fdm_config>
"""


def build_motor_prop_system() -> str:
    """B&M propulsion addendum: DC motor + fixed-pitch prop, solved algebraically.

    Motor:  V_in = V_max * throttle,  K_V = K_Q [V s/rad],  armature R, no-load current i0
    Prop :  T = rho D^4 CT(J) n^2,  Q = rho D^5 CQ(J) n^2,  CT, CQ quadratic in J
    Torque balance  K_Q (V_in - K_V Omega)/R - K_Q i0 = Q(Omega)  is a quadratic in
    Omega:  a Omega^2 + b Omega + c = 0  (B&M addendum eq. 4.x).
    """
    KV = 60.0 / (2 * math.pi * P["KV_rpm_per_volt"])
    D = P["D_prop"]
    tp = 2 * math.pi
    # coefficients that do not depend on the flight state (rho and Va are properties)
    a_k = D ** 5 * P["CQ0"] / tp ** 2               # * rho
    b1_k = D ** 4 * P["CQ1"] / tp                   # * rho * Va
    b0 = KV * KV / P["R_motor"]
    c2_k = D ** 3 * P["CQ2"]                        # * rho * Va^2
    cV_k = KV * P["V_max"] / P["R_motor"]           # * throttle
    c0 = KV * P["i0"]
    t0_k, t1_k, t2_k = D ** 4 * P["CT0"] / tp ** 2, D ** 3 * P["CT1"] / tp, D ** 2 * P["CT2"]
    q0_k, q1_k, q2_k = D ** 5 * P["CQ0"] / tp ** 2, D ** 4 * P["CQ1"] / tp, D ** 3 * P["CQ2"]

    def lin(prop_a, k, prop_b=None):
        extra = f"<property> {prop_b} </property>" if prop_b else ""
        return f"<product> <property> {prop_a} </property> {extra} <value> {k:.8g} </value> </product>"

    return f"""  <!-- ===== Motor + propeller (Beard & McLain propulsion addendum) ===== -->
  <system name="motor-prop">
    <channel name="Propulsion">
      <!-- pilot/trim throttle (fcs/throttle-pos-norm) + autopilot throttle -->
      <summer name="fcs/throttle-total-norm">
        <input> fcs/throttle-pos-norm </input>
        <input> ap/throttle-cmd-norm </input>
        <clipto> <min> 0 </min> <max> 1 </max> </clipto>
      </summer>
      <fcs_function name="propulsion/rho-kgm3">
        <function> <product> <property> atmosphere/rho-slugs_ft3 </property> <value> 515.379 </value> </product> </function>
      </fcs_function>
      <fcs_function name="propulsion/Va-mps">
        <function> <product> <property> velocities/u-aero-fps </property> <value> 0.3048 </value> </product> </function>
      </fcs_function>
      <!-- quadratic in Omega: a, b, c -->
      <fcs_function name="propulsion/a">
        <function> {lin("propulsion/rho-kgm3", a_k)} </function>
      </fcs_function>
      <fcs_function name="propulsion/b">
        <function> <sum> {lin("propulsion/rho-kgm3", b1_k, "propulsion/Va-mps")} <value> {b0:.8g} </value> </sum> </function>
      </fcs_function>
      <fcs_function name="propulsion/c">
        <function>
          <sum>
            <product> <property> propulsion/rho-kgm3 </property> <property> propulsion/Va-mps </property>
                      <property> propulsion/Va-mps </property> <value> {c2_k:.8g} </value> </product>
            <product> <property> fcs/throttle-total-norm </property> <value> {-cV_k:.8g} </value> </product>
            <value> {c0:.8g} </value>
          </sum>
        </function>
      </fcs_function>
      <fcs_function name="propulsion/omega-rad_sec">
        <description> Omega = (-b + sqrt(b^2 - 4ac)) / 2a, floored at 0 </description>
        <function>
          <max>
            <value> 0 </value>
            <quotient>
              <difference>
                <pow>
                  <max> <value> 0 </value>
                    <difference>
                      <product> <property> propulsion/b </property> <property> propulsion/b </property> </product>
                      <product> <value> 4 </value> <property> propulsion/a </property> <property> propulsion/c </property> </product>
                    </difference>
                  </max>
                  <value> 0.5 </value>
                </pow>
                <property> propulsion/b </property>
              </difference>
              <product> <value> 2 </value> <property> propulsion/a </property> </product>
            </quotient>
          </max>
        </function>
      </fcs_function>
      <fcs_function name="propulsion/thrust-N">
        <function>
          <product> <property> propulsion/rho-kgm3 </property>
            <sum>
              <product> <property> propulsion/omega-rad_sec </property> <property> propulsion/omega-rad_sec </property> <value> {t0_k:.8g} </value> </product>
              <product> <property> propulsion/omega-rad_sec </property> <property> propulsion/Va-mps </property> <value> {t1_k:.8g} </value> </product>
              <product> <property> propulsion/Va-mps </property> <property> propulsion/Va-mps </property> <value> {t2_k:.8g} </value> </product>
            </sum>
          </product>
        </function>
      </fcs_function>
      <fcs_function name="propulsion/torque-Nm">
        <function>
          <product> <property> propulsion/rho-kgm3 </property>
            <sum>
              <product> <property> propulsion/omega-rad_sec </property> <property> propulsion/omega-rad_sec </property> <value> {q0_k:.8g} </value> </product>
              <product> <property> propulsion/omega-rad_sec </property> <property> propulsion/Va-mps </property> <value> {q1_k:.8g} </value> </product>
              <product> <property> propulsion/Va-mps </property> <property> propulsion/Va-mps </property> <value> {q2_k:.8g} </value> </product>
            </sum>
          </product>
        </function>
      </fcs_function>
      <fcs_function name="propulsion/rpm">
        <function> <product> <property> propulsion/omega-rad_sec </property> <value> {60 / tp:.8g} </value> </product> </function>
      </fcs_function>
      <fcs_function name="propulsion/current-A">
        <function>
          <quotient>
            <difference>
              <product> <property> fcs/throttle-total-norm </property> <value> {P['V_max']} </value> </product>
              <product> <property> propulsion/omega-rad_sec </property> <value> {KV:.8g} </value> </product>
            </difference>
            <value> {P['R_motor']} </value>
          </quotient>
        </function>
      </fcs_function>
    </channel>
  </system>"""


def build_motor() -> str:
    return """<?xml version="1.0"?>
<!-- gnc_trainer placeholder engine (generated): zero power, so it produces no
     thrust.  It exists only so JSBSim's trim has a throttle to adjust.  The
     real motor+prop model is the "motor-prop" <system> in gnc_trainer.xml. -->
<electric_engine name="gnc_dummy_motor">
  <power unit="WATTS"> 0.0 </power>
</electric_engine>
"""


def build_prop() -> str:
    return """<?xml version="1.0"?>
<!-- gnc_trainer placeholder thruster (generated) - see gnc_dummy_motor.xml -->
<direct name="gnc_direct">
</direct>
"""


def build_autopilot_stub() -> str:
    return """<?xml version="1.0"?>
<!-- gnc_trainer autopilot (filled in from Module 09 on). -->
<system name="gnc_autopilot">
</system>
"""


if __name__ == "__main__":
    (OUT / "Engines").mkdir(parents=True, exist_ok=True)
    (OUT / "fcs").mkdir(exist_ok=True)
    (OUT / "gnc_trainer.xml").write_text(build_aircraft())
    (OUT / "Engines" / "gnc_dummy_motor.xml").write_text(build_motor())
    (OUT / "Engines" / "gnc_direct.xml").write_text(build_prop())
    ap = OUT / "fcs" / "gnc_autopilot.xml"
    if not ap.exists():
        ap.write_text(build_autopilot_stub())
    for p in sorted(OUT.rglob("*.xml")):
        print("wrote", p.relative_to(REPO))
