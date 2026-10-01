# Module 02: answers

**2. Gimbal lock.** At θ = 89.9°, tan θ ≈ 573 and 1/cos θ ≈ 573, so
ψ̇ = (q sin φ + r cos φ)/cos θ ≈ 57 rad/s for 0.1 rad/s body rates. At exactly
90° it is undefined: roll and yaw become the same rotation and the Euler angles
lose a degree of freedom. JSBSim integrates the quaternion, q̇ = ½ Ω(ω) q,
which has no singularity. Euler angles are only *computed* from it for output.
Your guidance/estimation code should avoid integrating Euler angles for the
same reason (vertical climbs, loops, tail-sitters).

**3. Latitude.** The Coriolis drift scales with cos(lat): 0.94 ft/s at 0°,
about 0.66 ft/s at 45°, about 0.16 ft/s at 80° after 20 s. The centrifugal
reduction ω²R cos²(lat) is 0.111 ft/s² at the equator, about half that at
45°, and almost nothing at 80°. Gravitation itself also increases toward the
poles (oblateness + J2), so effective g goes from about 32.06 to about 32.23 ft/s².

**4. Structural frame.** x is positive *aft*. The empty CG at 41.0 in is 2.2 in
*ahead* of the ARP (43.2 in). Lift acting at the ARP behind the CG gives a
nose-*down* moment, which is the statically stable arrangement the tail trims
against. The loaded CG at about 45.5 in is 2.3 in *behind* the ARP, so lift at
the ARP is now nose-*up* and the horizontal tail does more of the stabilizing
work. Static stability depends on the CG relative to the *neutral point*
(Module 05), not to the ARP; the ARP is only where the aero force data are
referenced.

**Self-check.**
- `velocities/v-fps` is body-axis side velocity; `v-east-fps` is NED.
- Pure roll: C = [[1,0,0],[0,cφ,sφ],[0,−sφ,cφ]]. Positive φ is right wing
  down (right-hand rule about +x, nose).
- No singularity, cheaper, and the constraint is easy to renormalize.
- γ = θ − α = 2° wings-level. In a bank the simple relation fails; use
  sin γ = cos α cos β sin θ − (sin β sin φ + cos β sin α cos φ) cos θ.
- `fbx-total` is the sum of aero + propulsion + gear (and external) forces
  *without* gravity, and m·u̇ also includes the rotating-frame terms
  m(qw − rv).
