# d_arc compensation — correcting convex hull underestimation under rotation

## Overview

When CastHullShape checks continuous collision, it computes the signed distance between an obstacle and the convex hull of a shape at two consecutive poses. This is exact for pure translation, but when the shape also rotates, points trace **circular arcs** while the convex hull connects them with **straight chords**. The gap — the arc-chord sagitta, d_arc — can exceed the collision safety margin, allowing the true swept arc to penetrate obstacles that the convex hull reports as safe.

The Coal cast manager implements **d_arc compensation**: on every transform update, it computes d_arc per shape and applies it as the CastHullShape's swept sphere radius. Coal's GJK automatically subtracts this from reported distances, and the broadphase AABB inflates accordingly. This is transparent to all callers. Disabled by default; enable via:

```yaml
plugins:
  CoalCastBVHManager:
    class: CoalCastBVHManagerFactory
    config:
      d_arc_compensation: true
```

## Background

Schulman et al. (IJRR 2014, Section IV-B) quantify the gap as `d_arc = r·φ²/8`, where r is the maximum distance from any point on the shape to the rotation axis and φ is the rotation angle. The paper dismisses this as "well under 1 cm" for their applications. However, with a 1 mm collision margin, d_arc can easily exceed the safety margin (see Practical impact section below).

## Mathematical proof

### Setup

Consider shape A undergoing a rigid-body motion from time t to t+1. By Chasles' theorem, any rigid motion can be decomposed into a screw motion: rotation by angle φ around a unique axis k̂ (the screw axis) plus translation along k̂. Each point on A traces a **helix** around this screw axis.

Focus on a single point **p** on A at perpendicular distance r from the screw axis. The translation along k̂ is captured exactly by the convex hull (linear interpolation between start and end), so only the rotational component contributes to d_arc. Project onto the plane perpendicular to k̂. In this plane, p traces a circular arc of radius r from angle 0 to angle φ.

### Arc vs chord gap (the sagitta)

The chord connects the start position p₀ = (r, 0) to the end position p₁ = (r cos φ, r sin φ).

The midpoint of the arc is at angle φ/2: p_mid = (r cos(φ/2), r sin(φ/2)), at distance r from the origin.

The midpoint of the chord is:
```
M = (p₀ + p₁)/2 = (r(1 + cos φ)/2, r sin φ/2)
```

Distance of M from the origin:
```
|M|² = r²(1 + cos φ)²/4 + r² sin²φ/4
     = r²/4 · ((1 + cos φ)² + sin²φ)
     = r²/4 · (2 + 2 cos φ)
     = r²/2 · (1 + cos φ)
```

Using the half-angle identity `1 + cos φ = 2 cos²(φ/2)`:
```
|M| = r · cos(φ/2)
```

The **sagitta** (maximum perpendicular distance from chord to arc) is:
```
s = r - r·cos(φ/2) = r·(1 - cos(φ/2))
```

This is the maximum because the circular arc is symmetric about the perpendicular bisector of the chord. The gap is zero at both endpoints and increases monotonically toward the midpoint, where the arc point (r cos(φ/2), r sin(φ/2)) lies on the perpendicular bisector at distance r from the origin, while the chord midpoint M lies at distance r·cos(φ/2).

Taylor-expanding `cos(φ/2) = 1 - φ²/8 + φ⁴/384 - ...`:
```
s = r·φ²/8 - r·φ⁴/384 + ...
```

Therefore **d_arc = r·φ²/8** is a second-order approximation of the exact gap r·(1 - cos(φ/2)).

### Why this is a tight upper bound

For any point p on shape A at perpendicular distance r_p ≤ r from the screw axis, the gap for that point is r_p·(1 - cos(φ/2)) ≤ r·(1 - cos(φ/2)). So expanding the convex hull by d_arc = r·(1 - cos(φ/2)) via Minkowski sum with a ball of that radius guarantees the swept volume is contained: every point p(t) on the swept arc is within d_arc of its chord, and every chord is a subset of convhull(A(0), A(1)), so p(t) is within d_arc of convhull(A(0), A(1)).

The bound is tight for a single point at distance exactly r from the screw axis (achieved at the arc midpoint). For the full shape, the Minkowski ball expansion inflates uniformly in all directions while the arc bulge is directional. This non-directionality is inherent in the signed-distance formulation: signed distance is a scalar, so d_arc can only be applied uniformly. The paper uses the same approach (Eq. 25e: `sd ≥ d_safe + d_arc`).

In a multi-timestep trajectory, the over-expansion at sweep endpoints is typically covered by the adjacent interval's convex hull. However, at the trajectory start and end, the over-expansion is not covered — if the task requires approaching an obstacle up to exactly the safety margin at the final pose, d_arc inflation at the endpoint might prevent this.

### Exact vs approximate formula

| φ (deg) | φ (rad) | Exact: r·(1 - cos(φ/2)) | Approx: r·φ²/8 | Relative error |
|---------|---------|--------------------------|-----------------|----------------|
| 5       | 0.0873  | 0.000952r                | 0.000952r       | 0.02%          |
| 10      | 0.1745  | 0.00381r                 | 0.00381r        | 0.06%          |
| 20      | 0.3491  | 0.01519r                 | 0.01524r        | 0.3%           |
| 45      | 0.7854  | 0.07612r                 | 0.07711r        | 1.3%           |
| 90      | 1.5708  | 0.2929r                  | 0.3084r         | 5.3%           |

The paper uses the r·φ²/8 approximation for analytical clarity (to show d_arc is O(φ²)). For implementation, **use the exact formula r·(1 - cos(φ/2))**:
- Always correct, valid for any φ
- Tight (equals the maximum gap exactly, no overestimation)
- Negligible cost per shape per timestep (see trig-free evaluation below)

The approximation is only useful for symbolic/gradient analysis where φ² is easier to differentiate than 1 - cos(φ/2).

### Trig-free evaluation via half-angle identities

The exact formula can be evaluated without any trigonometric function calls by exploiting the rotation matrix trace and half-angle identities:

1. **Rotation angle from matrix trace** (no `acos`):
   ```
   cos(φ) = (tr(R) - 1) / 2
   ```
   This is just three additions on the matrix diagonal.

2. **Half-angle values from cos(φ)** (no `cos`/`sin`, just `sqrt`):
   ```
   cos(φ/2) = √((1 + cos(φ)) / 2)
   sin(φ/2) = √((1 - cos(φ)) / 2)
   ```
   These follow directly from the half-angle identities cos²(x) = (1 + cos(2x))/2.

3. **Sagitta factor** is then simply `1 - cos(φ/2)`, already computed.

4. **Rotation axis from skew-symmetric part of R** (no `atan2`):
   ```
   k = [R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1)] / (2·sin(φ))
   ```
   where `sin(φ) = 2·sin(φ/2)·cos(φ/2)`, both already computed.

5. **Screw axis position** (closest point to origin in shape-local frame):
   ```
   t_perp = t - (t·k)k
   c = t_perp/2 + (k × t_perp)·cos(φ/2) / (2·sin(φ/2))
   ```
   The `cos(φ/2)/sin(φ/2)` ratio reuses the values from step 2.

6. **r_max**: distance from shape bounding sphere center to screw axis, plus bounding radius.

Total cost: 2 `sqrt` calls, 1 division, a few dot/cross products, and 1 `norm`. No `acos`, `cos`, `sin`, or `atan2`. This is ~60 flops, comparable to the `inverseTimes` matrix operation already performed per shape in the transform update path.

**Numerical stability:** When φ < ~10⁻⁷ rad (cos(φ) ≈ 1), d_arc is below double-precision noise and the function returns 0 immediately. When φ ≈ π, the axis extraction from the skew-symmetric part becomes noisy (sin(φ) → 0), but d_arc is dominated by the sagitta factor ≈ 1.0 and r_max, both stable. Per-segment rotations near π should not occur in practice.

## Practical impact with 1 mm collision margin

Typical industrial robot parameters:

| Scenario | r (m) | φ (deg) | d_arc (mm) | vs 1mm margin |
|----------|-------|---------|------------|---------------|
| End-effector, slow motion | 0.5 | 5° | 0.48 | 48% of margin |
| End-effector, moderate | 0.5 | 10° | 1.90 | **190%** |
| End-effector, fast | 0.5 | 20° | 7.60 | **760%** |
| Long reach, moderate | 1.0 | 10° | 3.81 | **381%** |
| Wrist link, fast | 0.1 | 30° | 3.41 | **341%** |

All values computed with exact formula: d_arc = r·(1 - cos(φ/2)).

**At 10° rotation per timestep with r = 0.5 m, d_arc is already nearly 2 mm — twice a 1 mm safety margin.** The LVS thresholds section below shows that staying under 1 mm d_arc at this reach requires ≤ 7.2° per segment. Since `collision_margin_buffer` is d_check (broadphase query range), not d_safe, there is **zero implicit buffer** covering d_arc. The convex hull check can report `distance ≥ margin` (constraint satisfied) while the true swept arc penetrates the obstacle.

## Implementation

### How d_arc compensation works

`collectCastTransformUpdate()` in `coal_cast_managers.cpp` computes d_arc per shape using the trig-free method (see below), then calls `CastHullShape::setSweptSphereRadius(d_arc)` before `updateCastTransform()`. This has two effects:

1. **Broadphase**: `computeLocalAABB()` inflates the swept-volume AABB by d_arc in all directions, ensuring the broadphase tree covers the arc, not just the chord.
2. **Narrowphase**: Coal's GJK stores the swept sphere radius in `MinkowskiDiff::swept_sphere_radius` and subtracts it from the reported distance post-convergence. The collision constraint effectively becomes `distance ≥ margin + d_arc` without any change to callers.

**Swept sphere interaction**: CastHullShape's `computeShapeSupport()` uses `WithSweptSphere` mode for the underlying shape's intrinsic radius (e.g., sphere, capsule). CastHullShape's own swept sphere radius (d_arc) is a second, independent inflation layer — no double-counting. In a self-collision pair (two CastHullShapes), Coal sums both radii: d_arc₁ + d_arc₂, which is correct since each link's arc gap is independent.

### CastHullShape support function (unchanged)

`coal_casthullshape.cpp` — `computeShapeSupport()` implements the Schulman convex-hull support function: it evaluates the underlying shape's support at both pose 0 and pose 1, and returns whichever has a larger dot product with the query direction. This computes `sd(convhull(A(t), A(t+1)), B)`. The d_arc compensation is applied externally via the swept sphere radius, not by modifying the support function.

### Bullet's btCastHullShape

Same Schulman support function. d_arc is not compensated in Bullet.

### TrajOpt's collision_margin_buffer — d_check, NOT d_safe

The code comment says "Additional collision margin that is added for the collision check but is not used when calculating the error" (`collision_types.h:153-154`). Tracing the code confirms this:

1. **d_safe = margin** — the constraint enforces `distance ≥ margin` (`continuous_collision_constraint.cpp:120`, `collision_utils.cpp:182`)
2. **d_check = margin + margin_buffer** — controls broadphase AABB inflation and GJK `distance_upper_bound` (`continuous_collision_evaluators.cpp:96`)
3. **Gradient weighting**: `error_with_buffer = margin + margin_buffer - distance` is used only for weighting which contacts contribute most to the gradient (`weighted_average_methods.cpp:50`)

**The buffer does NOT increase the safety margin.** Without d_arc compensation enabled, the collision constraint has no rotational correction term.

## LVS thresholds required for d_arc ≤ 1 mm

Translation does not contribute to d_arc — the convex hull captures translational motion exactly (see Setup above). Only the per-segment rotation φ matters. From the exact formula, the maximum rotation per segment that keeps d_arc ≤ 1 mm is:

```
φ_max = 2 · arccos(1 - 0.001 / r)
```

| r (m) | Typical scenario     | φ_max (deg) | φ_max (rad) |
|-------|----------------------|-------------|-------------|
| 0.1   | Wrist / small link   | 16.2°       | 0.283       |
| 0.2   | Forearm link         | 11.5°       | 0.200       |
| 0.3   | Mid-arm              | 9.3°        | 0.163       |
| 0.5   | End-effector reach   | 7.2°        | 0.126       |
| 0.8   | Long arm             | 5.7°        | 0.100       |
| 1.0   | Full reach           | 5.1°        | 0.089       |

For a typical 6-DOF industrial arm with ~1 m reach, **~5° per segment** keeps d_arc under 1 mm for all links. A 90° joint motion would require ~18 subdivisions.

Note that r is per-shape, not per-joint: it is the maximum distance from any point on a collision shape to the screw axis of the motion, which depends on the shape's bounding radius and its offset from the screw axis. For multi-DOF motion the screw axis of the composed rotation is configuration-dependent, so the worst case uses the robot's maximum reach.

### TrajOpt does not implement rotation-aware LVS

TrajOpt's collision evaluators (both SCO and IFOPT) subdivide segments using only the **joint-space L2 norm**, not Cartesian translation or rotation:

```cpp
// continuous_collision_evaluators.cpp:174, collision_terms.cpp:840
const double dist = (dof_vals1 - dof_vals0).norm();
long cnt = std::ceil(dist / longest_valid_segment_length) + 1;
```

This norm is blind to Cartesian geometry: a small joint-space step on a long link can produce large rotation (and large d_arc), while a large joint-space step on a short wrist link may produce negligible Cartesian motion.

Tesseract's **SimplePlanner LVS profiles** do evaluate translation, rotation, and joint-space distance separately, taking the maximum:

```cpp
// interpolation.cpp:781
int trans_steps = int(trans_dist / translation_lvs_length) + 1;  // default 0.1 m
int rot_steps   = int(rot_dist / rotation_lvs_length) + 1;       // default 5°
int joint_steps = int(joint_dist / state_lvs_length) + 1;        // default 5°
steps = std::max({trans_steps, rot_steps, joint_steps});
```

However, this only generates the **seed trajectory**. Once TrajOpt starts optimizing, it moves waypoints freely and uses its own joint-space LVS for collision sub-stepping. The seed's fine subdivision has no effect after the first optimization iteration reshapes the trajectory.

**Without d_arc compensation**, the gap is doubly unguarded: no correction in the collision check, and no rotation-aware subdivision in the optimizer. Enabling `d_arc_compensation: true` on the cast manager addresses the collision check side, which works regardless of the planner or optimizer's subdivision strategy.

## Alternative approach (not implemented)

- **More timesteps**: d_arc ∝ φ², so doubling timesteps (halving φ) reduces d_arc by 4×. However, TrajOpt's `longest_valid_segment_length` uses joint-space norm, not Cartesian rotation, so it cannot automatically enforce the rotation thresholds from the table above. A rotation-aware LVS in the optimizer would be needed. Also increases optimization cost linearly.

## References

- Schulman, J., Duan, Y., Ho, J., Lee, A., Awwal, I., Bradlow, H., Pan, J., Patil, S., Goldberg, K., & Abbeel, P. (2014). Motion Planning with Sequential Convex Optimization and Convex Collision Checking. *International Journal of Robotics Research*, 33(9), 1251-1270.
