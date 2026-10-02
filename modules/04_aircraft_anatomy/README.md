# Module 04: Anatomy of an aircraft file

**Time:** about 5 h · **Reading:** RM §2.5 *Math* (functions, tables), §2.6 *Forces and Moments*, §3.1 *Aircraft* (pp. 33–63, the big one), §3.2–3.3 skim; RM Case Studies *Simple Ball* and *Ball with Parachute* (pp. 109–116)

## Objectives

1. Know every top-level section of an aircraft XML file and what JSBSim does
   with it.
2. Write aerodynamic functions and tables and know how JSBSim turns them into
   forces and moments.
3. Build three vehicles from scratch: a sphere, a sphere with a parachute, and
   a glider. Check each against hand calculations.
4. Read someone else's model (the C172) and produce a contribution table, which is
   what you'll do when you review aero changes in the team's fork.

---

## The file, section by section

```xml
<fdm_config name="m04_glider">
  <fileheader>        author, description, references - REVIEWERS READ THIS
  <metrics>           wing area S, span b, chord cbar; AERORP (aero reference point), VRP, EYEPOINT
  <mass_balance>      Ixx Iyy Izz Ixz, empty weight, CG location, <pointmass> payloads
  <ground_reactions>  <contact type="BOGEY|STRUCTURE">: gear and skids as springs + dampers + friction
  <external_reactions> extra forces (parachutes, tow lines, thrust you compute yourself)
  <propulsion>        <engine file="..."> + <thruster file="..."> + <tank>s   (Module 07)
  <system> / <flight_control> / <autopilot>   channels of FCS components   (Module 08)
  <aerodynamics>      <axis name="LIFT|DRAG|SIDE|ROLL|PITCH|YAW"> <function>s
  <output>            optional logging (Module 03)
</fdm_config>
```

**Locations** are in the *structural* frame (x aft, y right, z up; Module 02).
JSBSim computes the arm from each force's location to the **current CG**, so
moving the CG (fuel burn, payload) changes the moments automatically.

### Aerodynamics: how a coefficient becomes a force

JSBSim **sums every `<function>` in an axis**. Each function must return the
*dimensional* force [lbf] or moment [lbf·ft], so the standard pattern is

```xml
<axis name="PITCH">
  <function name="aero/moment/Cm_alpha">
    <description> Cmalpha = -0.80 </description>
    <product>
      <property> aero/qbar-psf </property>      <!-- q̄ -->
      <property> metrics/Sw-sqft </property>    <!-- S -->
      <property> metrics/cbarw-ft </property>   <!-- c̄ (b for roll/yaw) -->
      <property> aero/alpha-rad </property>
      <value> -0.80 </value>                    <!-- the derivative -->
    </product>
  </function>
```

| Default axes | Frame | Units |
|---|---|---|
| `DRAG`, `SIDE`, `LIFT` | wind (drag positive aft along the relative wind, lift positive up) | lbf |
| `ROLL`, `PITCH`, `YAW` | body, **about the AERORP** | lbf·ft |
| (alternatives) `X Y Z`, `AXIAL NORMAL` | body | lbf |

Rate derivatives use the non-dimensional rates: `aero/bi2vel` = b/(2V) and
`aero/ci2vel` = c̄/(2V), so C<sub>mq</sub> q̄ S c̄ (c̄/2V) q.

**Math elements** (RM §2.5): `<sum> <difference> <product> <quotient> <pow>
<abs> <sin> <cos> <min> <max> <lt> <ifthen> <table>` ... A `<table>` interpolates
linearly. It can have 1, 2 or 3 independent variables (`lookup="row|column|table"`),
which is how real aero databases built from wind-tunnel and CFD data look:
C<sub>L</sub>(α, Mach), C<sub>m</sub>(α, δ<sub>e</sub>)...

Every named function is also a **property**, so you can log it, plot it, or use
it in another function. The glider's drag polar reuses `aero/force/L_alpha`.

---

## Walkthrough

### 1. A sphere: [`aircraft/m04_ball/m04_ball.xml`](../../aircraft/m04_ball/m04_ball.xml)

The smallest useful file: metrics, mass, ground contacts, one drag function,
and an external "parachute" force whose deployment is a `lag_filter` in a
`<system>`. Run [`solutions/ball_terminal_velocity.py`](solutions/ball_terminal_velocity.py):

```text
free fall: JSBSim 221.1 ft/s vs analytic 218.7 ft/s at 3000 ft AGL
under canopy: JSBSim 24.27 ft/s vs analytic 24.28 ft/s (7.4 m/s)
```

V<sub>t</sub> = √(2mg / ρ C<sub>D</sub>S) depends on ρ, so the falling ball is
always slightly *behind* the local terminal velocity. Two modeling lessons are
in the comments of the XML:

- **Contact points are fixed to the body.** One point at the "bottom" of a
  sphere rotates away as soon as the ball rolls, and the ball then sinks through
  the ground. The model uses six points.
- **Where a force acts matters.** The parachute attaches *behind* the CG, so its
  drag weathervanes the body. Attach it in front and the ball tumbles all the
  way down.

### 2. A glider: [`aircraft/m04_glider/m04_glider.xml`](../../aircraft/m04_glider/m04_glider.xml)

A 5 kg, 3 m span UAS sailplane with a classic derivative build-up: a
C<sub>L</sub>(α) table with stall, a parabolic polar C<sub>D</sub> = C<sub>D0</sub> + K C<sub>L</sub>²,
a full set of moment derivatives, and an FCS that turns normalized commands
into surface angles. **Read the whole file.** It is the template for your UAS
model in Module 07.

Run [`solutions/glider_polar.py`](solutions/glider_polar.py). It glides the
model at fixed elevator settings, waits for the phugoid to die out, and turns
steady-glide data back into C<sub>L</sub>, C<sub>D</sub> and L/D:

```text
best measured L/D = 19.0 at CL = 0.74, V = 12.1 m/s; theory: L/D_max = 19.4 at CL* = 0.77
```

That is a *simulated flight test* that verifies the model does what the XML
says. You'll do the same against real data in Module 14.

> 💡 While building this model, the first version had C<sub>lβ</sub> = −0.08. The
> glider slowly wound itself into a steady 40° spiral. The classic
> spiral-stability criterion explains it: C<sub>lβ</sub>C<sub>nr</sub> = 0.0064 <
> C<sub>nβ</sub>C<sub>lr</sub> = 0.0072, so the spiral was unstable. More dihedral
> effect (C<sub>lβ</sub> = −0.10) fixed it. Exercise 1 has you reproduce this.

### 3. Reading someone else's model

Run [`solutions/read_c172_aero.py`](solutions/read_c172_aero.py). It lists
every aero function in `c172x.xml` and its coefficient **contribution** at trim:

```text
== PITCH axis (coefficient contributions at trim)
     -0.0251  aero/coefficient/Cmalpha    Pitch moment due to alpha
     +0.1000  aero/coefficient/Cmo        Pitching moment at zero alpha
     -0.1150  aero/coefficient/Cmde       Pitch moment due to elevator deflection
     -0.0401  = total Cpitch
```

Why isn't the trimmed C<sub>m</sub> zero? Because the aero moments are
referenced to the **AERORP**, and the lift and drag acting at the AERORP also
produce a moment about the CG (the C172 ARP is 2.3 in ahead of the loaded CG).
The *total* moment about the CG is zero. Point the script at any other model
(`python .../read_c172_aero.py m04_glider`) to review it the same way.

---

## Exercises

1. **[`exercises/glider_ex.py`](exercises/glider_ex.py):** make your own copy
   `aircraft/my_glider` and add **flaps**: an actuator channel plus ΔC<sub>L</sub>,
   ΔC<sub>D</sub> and ΔC<sub>m</sub> terms. Compare the glide with flaps up and down. Then
   reduce C<sub>lβ</sub> to −0.06 and watch the spiral. Solution model:
   [`aircraft/m04_glider_flaps`](../../aircraft/m04_glider_flaps/m04_glider_flaps.xml).
2. Add a second independent variable to the glider's C<sub>L</sub> table: make
   C<sub>L</sub> a function of α **and** `fcs/flap-pos-norm` (2-D table,
   `lookup="column"`). Delete the separate ΔC<sub>L</sub> flap function and check the
   glide is unchanged.
3. Move the glider's CG 5 cm forward *without* moving the AERORP. Predict
   the change in trimmed elevator. (Hint: C<sub>m,CG</sub> = C<sub>m,ARP</sub> +
   C<sub>L</sub>(x<sub>ARP</sub> − x<sub>CG</sub>)/c̄, with x positive aft.) Check it by gliding.
4. Run the bundled `scripts/ball_chute.xml` and compare its reefing schedule
   with RM Case Study 2.

## Self-check

- Which axes are in the wind frame, and which in the body frame?
- You add `<function>` for C<sub>lp</sub> but forget `aero/bi2vel`. By what factor is
  your roll damping wrong at 15 m/s with b = 3 m?
- Why do aerodynamic moments depend on where the AERORP and the CG are?
- What's the difference between a contact of type `BOGEY` and `STRUCTURE`?
- A colleague's new aero table makes the trimmed elevator jump 5°. How would
  you find which term did it?

## Done when

- [ ] `my_glider` flies with flaps and you can explain the speed change
- [ ] You can write a 1-D table and a derivative function from memory
- [ ] You've produced a contribution table for a model you didn't write
