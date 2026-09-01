# d_arc compensation — correcting convex hull underestimation under rotation

## Overview

When CastHullShape checks continuous collision, it computes the signed distance between an obstacle and the convex hull of a shape at two consecutive poses. This is exact for pure translation, but when the shape also rotates, points trace **circular arcs** while the convex hull connects them with **straight chords**. The gap — the arc-chord sagitta, d_arc — can exceed the collision safety margin, allowing the true swept arc to penetrate obstacles that the convex hull reports as safe.

The Coal cast manager can close that gap. **d_arc compensation** computes the sagitta per shape on every transform update and applies it as the CastHullShape's swept sphere radius, so Coal's GJK subtracts it from reported distances and the broadphase AABB inflates to match. It is transparent to every caller and **disabled by default** — see *Enabling it*.

## The Schulman proposal, and what we changed

Schulman et al. (IJRR 2014, Section IV-B) quantify the gap as `d_arc = r·φ²/8`, where φ is the rotation angle and r is "the maximum distance from a point on A to the **local** rotation axis", applied as an additive term on the safety distance (Eq. 21: `sd(convhull(A(t), A(t+1)), O) > d_safe + d_arc`). The paper asserts the bound without proof and then ignores it — "well under 1 cm in all of the problems we considered", and `d_arc = 0.001 cm` in the needle-steering application of Section VI. There is therefore no reference implementation to compare against, only the bound. With a 1 mm collision margin the correction is not ignorable; see *Does this affect you?* below.

That local axis passes through the body frame's origin — an arbitrary modelling convention, for a robot link wherever the URDF happens to put the joint frame — so the paper's r is not a property of the motion at all. Sliding that origin while the motion stays fixed moves the paper's d_arc by 14×, and at the natural placement it comes out roughly 10× too small.

This implementation uses the **screw axis** (Chasles) instead, which is intrinsic to the relative transform and therefore frame-invariant. For a single revolute joint the screw axis *is* the joint axis and screw interpolation *is* the true motion, so d_arc is exact rather than approximate. The paper's origin axis bounds nothing physical unless one also adopts its implicit "rotate about the body origin, translate linearly" interpolation, which is not how a robot moves.

The second departure is smaller: the exact sagitta replaces the paper's `φ²/8` Taylor term, which is the looser of the two upper bounds. What this implementation computes, per shape, is

```
d_arc = r · (1 - cos(φ/2))
```

with φ the rotation angle of the segment and r the furthest any point of the shape can be from the axis it turns about — the distance from the shape's bounding-sphere centre to the screw axis, plus that sphere's radius. The code calls it `r_max`. *Derivation* below carries the evidence for both departures.

## Does this affect you?

Typical industrial robot parameters, with a 1 mm collision margin:

| Scenario | r (m) | φ (deg) | d_arc (mm) | vs 1mm margin |
|----------|-------|---------|------------|---------------|
| End-effector, slow motion | 0.5 | 5° | 0.48 | 48% of margin |
| End-effector, moderate | 0.5 | 10° | 1.90 | **190%** |
| End-effector, fast | 0.5 | 20° | 7.60 | **760%** |
| Long reach, moderate | 1.0 | 10° | 3.81 | **381%** |
| Wrist link, fast | 0.1 | 30° | 3.41 | **341%** |

All values computed with exact formula: d_arc = r·(1 - cos(φ/2)).

**At 10° rotation per timestep with r = 0.5 m, d_arc is already nearly 2 mm — twice a 1 mm safety margin.** Staying under 1 mm at that reach means holding each segment to ≤ 7.2°, tabulated below.

Translation does not contribute to d_arc — the convex hull captures translational motion exactly (see *Derivation*). Only the per-segment rotation φ matters. From the exact formula, the maximum rotation per segment that keeps d_arc ≤ 1 mm is:

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

## Why nothing else covers the gap

Two mechanisms might be expected to absorb it. Neither does.

### TrajOpt's `collision_margin_buffer` is d_check, not d_safe

The code comment says "Additional collision margin that is added for the collision check but is not used when calculating the error" (`TrajOptCollisionConfig::collision_margin_buffer`, `collision_types.h`). Tracing the code confirms this:

1. **d_safe = margin** — the constraint enforces `distance ≥ margin` (`ContinuousCollisionConstraintD::update()`, and `trajopt_common::getGradient()`)
2. **d_check = margin + margin_buffer** — controls broadphase AABB inflation and GJK `distance_upper_bound` (the evaluator's constructor calls `contact_manager_->incrementCollisionMargin(margin_buffer_)`)
3. **Gradient weighting**: `error_with_buffer = margin + margin_buffer - distance` is used only for weighting which contacts contribute most to the gradient (`getWeightedAvgGradient*()`, weighted by `getMaxErrorWithBuffer()`)

**The buffer does not increase the safety margin** — it widens the query range, not the constraint. So the convex hull check can report `distance ≥ margin`, constraint satisfied, while the true swept arc penetrates the obstacle.

### Subdivision is not rotation-aware

d_arc ∝ φ², so doubling the number of timesteps quarters it — but nothing in the optimizer subdivides on rotation. TrajOpt's collision evaluators, both SCO and IFOPT, use only the **joint-space L2 norm**, not Cartesian translation or rotation:

```cpp
// LVSContinuousCollisionEvaluator::calcCollisionsHelper (ifopt) and
// DiscreteCollisionEvaluator::CalcCollisions (sco), among others
const double dist = (dof_vals1 - dof_vals0).norm();
long cnt = std::ceil(dist / longest_valid_segment_length) + 1;
```

This norm is blind to Cartesian geometry: a small joint-space step on a long link can produce large rotation (and large d_arc), while a large joint-space step on a short wrist link may produce negligible Cartesian motion.

Tesseract's **SimplePlanner LVS profiles** do subdivide on translation and rotation as well as joint distance, taking the maximum of the three (`interpolateJointJointWaypoint`, defaulting to 0.1 m and 5°). But that only shapes the **seed trajectory**: once TrajOpt starts optimizing it moves waypoints freely and re-subdivides with its own joint-space LVS, so the seed's fine subdivision is gone after the first iteration.

Reaching the thresholds tabulated above therefore means setting `longest_valid_segment_length` by hand against the tightest link and paying for it in optimization time linearly; doing it automatically would need a rotation-aware LVS in the optimizer.

**The gap is thus doubly unguarded**: no correction in the collision check, and no rotation-aware subdivision in the optimizer. Enabling d_arc compensation addresses the collision-check side, which works regardless of the planner's or optimizer's subdivision strategy.

## Enabling it

```yaml
plugins:
  CoalCastBVHManager:
    class: CoalCastBVHManagerFactory
    config:
      d_arc_compensation: true
```

Cost is a few arithmetic operations per shape per transform update, with no `acos`, `cos`, `sin` or `atan2` on any path, and nothing at all when a link's rotation is negligible; *How it works* breaks it down. Those are operation counts — the feature has not been benchmarked end to end.

**What changes when you enable it:** reported distances shrink by d_arc, and swept-volume AABBs inflate by it, so expect more broadphase candidate pairs. The effect is largest for segments turning near a half turn, where d_arc approaches r_max. That is the correction working, not a regression, but it is visible to anyone watching pair counts.

## What it does not bound

Two gaps remain, both inherent in the formulation rather than introduced by it:

- **The interpolation is assumed.** d_arc bounds the deviation of a *screw* motion from the chord. A single revolute or prismatic joint moves exactly that way, so the bound is exact. Multi-joint motion does not, and its true path can bulge past the bound by an amount nothing here measures. Only subdividing the segment controls that — see *Subdivision is not rotation-aware*.
- **φ is recovered from the trace, so it lands in [0, π].** A segment turning further reads as its complement and yields a small d_arc. The convex hull of the two poses is already meaningless for such a segment, so subdivision is the only guard either way.

## How it works

`updateCastShapeTransforms()` in `coal_cast_managers.cpp` computes d_arc per shape using the trig-free method (see below), then calls `CastHullShape::setSweptSphereRadius(d_arc)` before `updateCastTransform()`. This has two effects:

1. **Broadphase**: `computeLocalAABB()` inflates the swept-volume AABB by d_arc in all directions, ensuring the broadphase tree covers the arc, not just the chord.
2. **Narrowphase**: Coal's GJK stores the swept sphere radius in `MinkowskiDiff::swept_sphere_radius` and subtracts it from the reported distance post-convergence. The collision constraint effectively becomes `distance ≥ margin + d_arc` without any change to callers.

**Swept sphere interaction**: CastHullShape's `computeShapeSupport()` uses `WithSweptSphere` mode for the underlying shape's intrinsic radius (e.g., sphere, capsule). CastHullShape's own swept sphere radius (d_arc) is a second, independent inflation layer — no double-counting. In a self-collision pair (two CastHullShapes), Coal sums both radii: d_arc₁ + d_arc₂, which is correct since each link's arc gap is independent.

### CastHullShape support function (unchanged)

`coal_casthullshape.cpp` — `computeShapeSupport()` implements the Schulman convex-hull support function: it evaluates the underlying shape's support at both pose 0 and pose 1, and returns whichever has a larger dot product with the query direction. This computes `sd(convhull(A(t), A(t+1)), B)`. The d_arc compensation is applied externally via the swept sphere radius, not by modifying the support function.

Bullet's `btCastHullShape` uses the same Schulman support function, but does not compensate d_arc.

### Trig-free evaluation via half-angle identities

The exact formula can be evaluated without any trigonometric function calls by exploiting the rotation matrix trace and half-angle identities:

1. **Rotation angle from matrix trace** (no `acos`):
   ```
   cos(φ) = (tr(R) - 1) / 2
   ```
   Three additions on the matrix diagonal. A rotation small enough that `cos(φ) > 1 - 1e-14` — about
   1.4e-7 rad — returns d_arc = 0 here and goes no further; every step below assumes that case has
   already been excluded.

2. **Half-angle values from cos(φ)** (no `cos`/`sin`, just `sqrt`):
   ```
   cos(φ/2) = √((1 + cos(φ)) / 2)
   sin(φ/2) = √((1 - cos(φ)) / 2)
   ```
   These follow directly from the half-angle identities cos²(x) = (1 + cos(2x))/2.

3. **Sagitta factor** is then simply `1 - cos(φ/2)`, already computed.

4. **Rotation axis below the 120-degree handoff**, from the skew-symmetric part of R (no `atan2`):
   ```
   w = [R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1)]   (= 2·sin(φ)·k̂, unnormalised)
   ```
   `w` is used **unnormalised**. It appears only inside projections onto the plane perpendicular to
   the axis, which are scale-free once divided by `|w|²`, and a single factor
   `1/(2·(1 - cos φ)) == 1/(4·sin²(φ/2))` absorbs both the `2·sin(φ)` normalisation and the
   `cot(φ/2)` that step 6 needs. Nothing on this path ever divides by `sin(φ)` on its own, so it has
   no singularity beyond the φ ≈ 0 case step 1 already excludes.

5. **Rotation axis past the handoff**, from the symmetric part instead:
   ```
   R + Rᵀ = 2·cos(φ)·I + 2·(1 - cos φ)·k̂k̂ᵀ
   ```
   Its divisor (`1 - cos φ`) grows to 2 exactly where the skew part's (`sin φ`) shrinks to 0,
   reaching exactly 0 at φ = π. The best-determined column of `k̂k̂ᵀ` — the one with the largest
   diagonal entry — gives `k̂` up to sign; the sign is recovered by agreeing with the skew part where
   it still carries one, and where it does not, at φ = π exactly, `cot(φ/2)` has vanished with it so
   both signs give the same downstream result. The handoff sits at 120 degrees rather than at a
   tolerance close to π because both extractions are well conditioned across a wide overlap there —
   it is not a cliff either one needs to be timed against.

6. **Screw axis position** (closest point to the origin in the shape-local frame):
   ```
   t_perp = t - (t·k̂)k̂
   c = t_perp/2 + (k̂ × t_perp)·cos(φ/2) / (2·sin(φ/2))
   ```
   Past the handoff this is evaluated as written: `cos(φ/2)` is already available from step 3 as
   `1 - sagitta_factor`, and `sin(φ/2)` comes from step 2's identity. Below it, step 4's single
   factor stands in for the `cot(φ/2)` ratio, applied directly to the unnormalised `w` and
   `w × t_perp`, so `sin(φ/2)` is never isolated there.

7. **r_max**: distance from the shape's bounding-sphere centre to the screw axis, plus the bounding radius.

**Cost**, split by what `computeDArcScalars` does once per link and what `computeDArc` does once per
shape on that link:

- **Per link** (`computeDArcScalars`), only when the rotation exceeds the ~1.4e-7 rad early-return
  threshold: 1 `sqrt` (the half-angle cosine) and 1 division (the `1/(4·sin²(φ/2))` factor). Zero
  cost otherwise.
- **Per shape, below the 120-degree handoff** (`computeDArc`'s common case): 1 division (`1/|w|²`)
  and 1 `sqrt` (the final distance-to-axis norm).
- **Per shape, past the 120-degree handoff** (rare — the branch is taken for any rotation past 120
  degrees, a 60-degree range, but a single trajectory segment seldom turns that far): 3 `sqrt` calls
  and 2 divisions.

No `acos`, `cos`, `sin`, or `atan2` on either path.

**Numerical stability:** Step 1's early return does more than skip negligible work. It is also what keeps `1/|w|²` finite on the fast path, since nothing else bounds `|w| = 2·sin(φ)` away from zero there.

Near a half turn it is the skew part that degrades. `2·sin(φ)` shrinks to exactly zero at φ = π, and so does the vector it scales, so a `1/(2·sin φ)` normalisation is `0/0` there and merely ill-conditioned within about a degree of it. Neither failure announces itself: coal validates a swept-sphere radius with `radius < 0`, which NaN passes, and a finite-but-wrong axis passes any check at all. Because `r_max = dist_to_axis + aabb_radius` is computed *from* that axis, the error reaches the reported contact distance — r_max is not independent of the axis. Hence the handoff to the symmetric part, which guarantees a finite, correctly signed d_arc at every φ including exactly π.

Both guarantees are pinned against an `Eigen::AngleAxisd` oracle sharing no code with the implementation: `HalfTurnProducesFiniteDArc` over a set of axes and a bracket of angles around π, and `HandoffIsContinuousAcrossTheBranch` sweeping 110° to 130°, so the handoff is not a visible cliff in the reported d_arc.

## Derivation

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

The bound is tight for a single point at distance exactly r from the screw axis (achieved at the arc midpoint). For the full shape, the Minkowski ball expansion inflates uniformly in all directions while the arc bulge is directional. This non-directionality is inherent in the signed-distance formulation: signed distance is a scalar, so d_arc can only be applied uniformly. The paper applies it the same way (Eq. 21).

The bound has been checked against a brute-force sweep: sampling points on a shape's bounding sphere and times in [0, 1], the largest deviation of the true screw motion from each point's chord reaches 0.9943–0.9997 of the computed d_arc across angles from 10° to exactly 180° and both axis-extraction branches — never above it.

In a multi-timestep trajectory, the over-expansion at sweep endpoints is typically covered by the adjacent interval's convex hull. However, at the trajectory start and end, the over-expansion is not covered — if the task requires approaching an obstacle up to exactly the safety margin at the final pose, d_arc inflation at the endpoint might prevent this.

### Why the screw axis, not the paper's local axis

Holding one physical motion fixed and sliding the body frame's origin along its own x axis isolates the frame dependence claimed above:

| body-origin offset | d_arc, screw axis (this implementation) | d_arc, paper's `r·φ²/8` about the local origin |
|--------------------|----------------------------------------|-------------------------------------------------|
| 0.0 m | 2.19e-2 | 2.30e-3 |
| 0.5 m | 2.19e-2 | 9.95e-3 |
| 1.0 m | 2.19e-2 | 1.76e-2 |
| 2.0 m | 2.19e-2 | 3.29e-2 |

The screw-axis column is one motion measured four ways and does not move. The paper's recipe spans 14× across the same placements, and is smallest — by about 10× — exactly where a modeller would naturally put the frame, on the shape itself.

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
- Negligible cost per shape per timestep (see *How it works*)

The approximation is only useful for symbolic/gradient analysis where φ² is easier to differentiate than 1 - cos(φ/2).

## References

- Schulman, J., Duan, Y., Ho, J., Lee, A., Awwal, I., Bradlow, H., Pan, J., Patil, S., Goldberg, K., & Abbeel, P. (2014). Motion Planning with Sequential Convex Optimization and Convex Collision Checking. *International Journal of Robotics Research*, 33(9), 1251-1270.
