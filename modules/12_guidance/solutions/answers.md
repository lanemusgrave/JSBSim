# Module 12: answers

**A. Fillets.** Maximum deviation from the waypoint path: switching at the
waypoints 124 m, fillets (R = 120 m) 26 m. The plain manager starts turning only
after it passes the waypoint, so it overshoots by about one turn radius
(R = V²/(g tan φ<sub>max</sub>) ≈ 110 m) and then has to come back. The fillet starts
the turn R/tan(ρ/2) before the corner (120 m for 90°) on a circle tangent to both
legs. The remaining 26 m is the orbit's capture transient, plus the fact that the
fillet radius only just exceeds the achievable turn radius. Make R ≥ 1.2 × the
minimum turn radius, including the downwind ground speed.

**B. k_path sweep** (150 m offset, calm air):

| k_path | within 5 m at | overshoot | late \|e\| max | bank activity |
|---|---|---|---|---|
| 0.005 | 45 s | 0 m | 7.4 m | quiet |
| **0.02** | **16 s** | **0 m** | **0.3 m** | quiet |
| 0.1 | 10 s | 24 m | 0.1 m | 0.8° std |
| 0.3 | 9 s | 44 m | 35 m | **±30° weave** |
| 1.0 | 9 s | 57 m | 56 m | **±30° weave** |

k<sub>path</sub> sets the "spatial gain": near the line the course command changes by
about χ<sub>∞</sub>(2/π)k<sub>path</sub> rad per metre of error. Too large, and the outer loop
asks for course changes faster than the course loop (≈1 rad/s, Module 10) and
roll loop can deliver. The bank saturates at ±30° and the airplane weaves across
the line, a guidance-induced limit cycle. Rule of thumb: the effective guidance
bandwidth V·χ<sub>∞</sub>(2/π)k<sub>path</sub> ≈ 25·1.05·0.64·k should stay well below the
course-loop bandwidth. That gives about 0.4 rad/s at k = 0.02 and 6 rad/s at k = 0.3.
χ<sub>∞</sub> sets the approach angle far from the line (60° here). 90° means
"aim straight at the line", which overshoots more.

**C. TECS (no reference solution).** Sketch: with γ = ḣ/V and normalized energy
rates Ė<sub>T</sub> = γ + V̇/g and Ḃ = γ − V̇/g, use commands γ<sub>c</sub> = k<sub>h</sub>(h<sub>c</sub>−h)/V and
V̇<sub>c</sub> = k<sub>V</sub>(V<sub>c</sub>−V). Then throttle = PI(Ė<sub>T,c</sub> − Ė<sub>T</sub>) (scaled by
W/T<sub>max</sub>) and θ<sub>c</sub> = PI(Ḃ<sub>c</sub> − Ḃ), with the pitch loop from Module 09 inside.
The payoff: a climb *and* a speed change no longer fight each other through the
phugoid, and when thrust saturates, TECS gives priority (usually to speed) by
design rather than by accident.

**Self-check.**
- *Course or heading for path following?* Course (ground track); see the 13.9°
  crab angle in wind.
- *Why can an orbit drift in wind even with perfect guidance?* The downwind side
  needs φ = atan(V<sub>g</sub>²/(gR)): 33° at 31 m/s and R = 150 m, more than the 30° limit.
- *Direct-to vs path following?* Direct-to has no memory of the leg. Once
  displaced, it flies a new line to the waypoint (a 58 m mean offset in the demo).
