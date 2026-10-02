# Module 12: Guidance

**Time:** about 6 h · **Reading:** B&M ch. 9 (design models for guidance), ch. 10 (straight-line and orbit following), ch. 11 (path manager, fillets); RM Case Study 4 *Modeling a waypoint navigation system* (pp. 128–135) and the `<waypoint_heading>`/`<waypoint_distance>` components

## Objectives

1. Implement vector-field **straight-line** and **orbit** following, and know
   what k<sub>path</sub>, χ<sub>∞</sub> and k<sub>orbit</sub> do.
2. Fly a **waypoint mission** with a path manager, in wind, on top of the
   Module 09–10 autopilot, with every non-ideality on.
3. Compare JSBSim's built-in **direct-to** guidance (RM case study) with path
   following, and understand course vs heading in wind.
4. Improve corners with **fillets** (exercise).

---

## Refresher

```text
 guidance (10 Hz, Python)        autopilot (120 Hz, XML)
 waypoints ─► path manager ─► χ_c, h_c ─► course & altitude hold ─► roll/pitch ─► surfaces
                   ▲                                                    │
                   └──────────── position (N,E), course χ ◄──────────────┘
```

**Straight line** through r with direction q (course χ<sub>q</sub>). Cross-track error
e<sub>py</sub> = −sin χ<sub>q</sub>(p<sub>n</sub>−r<sub>n</sub>) + cos χ<sub>q</sub>(p<sub>e</sub>−r<sub>e</sub>) (positive = right of the path):

$$
\chi_c = \chi_q - \chi_\infty\frac{2}{\pi}\tan^{-1}(k_{path}\,e_{py})
$$

**Orbit** with centre c, radius ρ, direction λ (+1 CW), at distance d and bearing φ from the centre:

$$
\chi_c = \varphi + \lambda\left[\frac{\pi}{2} + \tan^{-1}\!\left(k_{orbit}\frac{d-\rho}{\rho}\right)\right]
$$

**Wind:** guidance commands *course* (where you go over the ground). The course
loop flies a crab angle asin(V<sub>w,⊥</sub>/V<sub>a</sub>) automatically. In an orbit the needed
bank changes around the circle with ground speed: φ = atan(V<sub>g</sub>²/(gR)).

## The code

- [`src/gnclab/guidance.py`](../../src/gnclab/guidance.py): `line_course`,
  `orbit_course`, `WaypointManager` (half-plane switching, B&M Algorithm 5,
  loiter at the end), `ne_position`.
- The XML autopilot gained a **Direct-to guidance** channel using JSBSim's
  `<waypoint_heading>` and `<waypoint_distance>` (set `guidance/target-lat-rad`,
  `-lon-rad`, `ap/guidance-on = 1`).

> 🐛 **Gotcha found while building this module:**
> `position/distance-from-start-lat-mt` and `-lon-mt` look like local north/east,
> but **they are unsigned distances**. Fly west and "east" still increases.
> The first mission run drifted hundreds of metres off its legs because the
> vector field was steering correctly toward a position it couldn't see. The
> fix is `gnclab.guidance.ne_from_latlon` (signed, from lat/lon relative to the
> IC point). Module 10's ground-track plot had the same bug. Lesson: **check
> the sign of every property you didn't create**, with a test that flies west
> and south.

---

## Walkthrough

### 1. Line and orbit: [`solutions/line_and_orbit.py`](solutions/line_and_orbit.py)

```text
wind 0.0 m/s: line  |cross-track| last 20 s: mean 0.11 m; crab angle  0.0 deg
              orbit |radial error|  last 40 s: mean 1.80 m
wind 6.0 m/s: line  |cross-track| last 20 s: mean 0.11 m; crab angle 13.9 deg  (= asin(6/25))
              orbit |radial error|  last 40 s: mean 6.45 m, max 24.8 m
```

The orbit in wind bulges on the downwind side. At 31 m/s ground speed and a
150 m radius the airplane needs 33° of bank, and the autopilot limits it to 30°.
That's a guidance–autopilot **interface requirement**: the loiter radius must
account for worst-case ground speed (B&M add a feed-forward term for this).

### 2. Mission: [`solutions/waypoint_mission.py`](solutions/waypoint_mission.py)

Four legs with climbs and descents in a 6 m/s wind, then a loiter:

```text
leg 1 ((0, 0) -> (800, 0)):     steady |cross-track| mean  0.3 m
leg 3 ((800, 800) -> (0, 800)): steady |cross-track| mean 25.8 m, max 122 m   <- 90 deg corners overshoot
direct-to      : |cross-track from the leg| for north 300..750 m: mean 58.2 m
path following : |cross-track from the leg| for north 300..750 m: mean  3.0 m
```

- Switching legs *at* the waypoint overshoots each 90° corner by about one
  turn radius. Exercise A fixes it.
- **Direct-to vs path following:** started 150 m off the leg, direct-to flies a
  new straight line to the waypoint and never returns to the leg. Because our
  inner loop holds GPS *course*, direct-to does **not** drift downwind. The
  textbook curved "homing" track appears when a *heading* hold chases the
  bearing (like the C172 autopilot in Module 10).

---

## Exercises ([`exercises/guidance_ex.py`](exercises/guidance_ex.py))

- **A. Fillets** (B&M Algorithm 6): turn onto the next leg on a circle tangent to
  both legs. Solution: [`solutions/guidance_ex.py`](solutions/guidance_ex.py)
  (max path deviation 124 m → 26 m).
- **B. Tuning:** sweep k<sub>path</sub> and χ<sub>∞</sub>. What breaks when k<sub>path</sub> is too large,
  and how does that relate to Module 10's course-loop bandwidth?
- **C. (Advanced) TECS:** total-energy control for pitch and throttle. No
  reference solution; a sketch is in `answers.md`.

## Self-check

- Guidance commands course, not heading. Why does that matter in a 6 m/s crosswind?
- What sets the smallest orbit radius you can fly in wind?
- Why does a large k<sub>path</sub> cause weaving rather than faster convergence?
- What's the difference between half-plane switching and "within X m of the waypoint"?
  When does the latter fail?

## Done when

- [ ] You can fly the mission in wind and explain every deviation on the plot
- [ ] Your fillet manager cuts the corner overshoot by at least 3×
- [ ] You can explain the unsigned-distance bug and how you'd test for it
