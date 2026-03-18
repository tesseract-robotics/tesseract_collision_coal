# Code Review: Coal Continuous Collision Detection Implementation

**Reviewer:** Claude Code
**Date:** 2026-03-11 (updated 2026-03-18, 2026-03-19)
**Scope:** Coal cast (continuous) collision checking — CastHullShape, cast managers, utilities, and tests

## Summary

This implementation ports Tesseract's Bullet-based continuous collision detection to the Coal library. The architecture closely mirrors Bullet's approach (dual COW maps, CastHullShape as swept convex hull, support-function-based cc_time classification) while adapting to Coal's API (separate static/dynamic broadphase managers, `ComputeCollision` functor caching, `GEOM_CUSTOM` virtual dispatch).

Overall quality is **good**. The code is well-structured, thoroughly tested, and handles edge cases (multi-shape links, octree expansion, disabled objects). Below are specific findings organized by severity.

---

## Issues Found

### 1. `#pragma GCC diagnostic` for non-virtual destructor

**File:** `coal_utils.h:537-547`

```cpp
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
struct CollisionCallback : coal::CollisionCallBackBase { ... };
#pragma GCC diagnostic pop
```

The `CollisionCallback` struct inherits from `coal::CollisionCallBackBase` which apparently lacks a virtual destructor. The pragma suppresses the warning, but the struct declares `virtual ~CollisionCallback() = default;` itself. This is correct for preventing UB when deleting through a base pointer, but the pragma suggests an upstream Coal issue. Consider filing this upstream.

**Severity:** Low — correctly handled, the pragma is just cosmetic since the derived class does declare the virtual destructor.

### 2. `CCType_Time0`/`CCType_Time1` not produced for pure-translational sweeps

**File:** `coal_utils.cpp:443-458`

`populateContinuousCollisionFields` classifies `cc_type` by comparing *local-frame* support values at t=0 and t=1:

```cpp
double sup_local0 = normal_local0 · pt_local0;   // shape frame at t=0
double sup_local1 = normal_local1 · pt_local1;   // shape frame at t=1
```

For a pure translation the cast transform has no rotation component, so `normal_local0 == normal_local1` and `sup_local0 == sup_local1`. The result is always `CCType_Between`, regardless of how strongly the contact normal aligns with the sweep direction.

Bullet uses *world-frame* supports:

```
sup_world = normal_world · (shape_center + R * pt_local)
           = normal_world · shape_center  +  normal_local · pt_local
```

The `normal_world · shape_center` term is non-zero for a translation, so Bullet produces `CCType_Time1` when `normal · sweep > 0` and `CCType_Time0` when `normal · sweep < 0`.

**Why Coal drops the translational term:** For a multi-shape link that *rotates*, each per-shape world center orbits the link origin. Including `normal · center` in the comparison would pick up that orbital displacement, giving different `cc_type` values for shapes in the same link based on their local offsets rather than any real change in swept geometry. Bullet avoids this because compound children use a link-level rotation cast transform (no per-shape local translation), so their `shape_center` is effectively the link origin. Coal's `CastHullShape` is built around the per-shape world transform, which does include the local translation offset — hence the deliberate decision to drop it.

**The gap:** For single-shape links (no local offset) executing a pure translation, there is no orbital-motion bias. Both Bullet and Coal could safely use world-frame supports there, and Bullet does. Coal does not, so it always produces `CCType_Between` in that case, losing the advancing/receding classification that `CCType_Time1`/`CCType_Time0` encode.

**Practical consequence:** Downstream code that checks `cc_type == CCType_Time1` to identify contacts that are worst at the end pose will behave differently for translational sweeps under Coal. The `cc_time` value remains accurate (center-trajectory projection), so timing information is preserved; only the directional severity classification is affected.

**Possible fix:** Compute the support comparison at the link-frame level rather than the per-shape frame level (i.e., use the link origin as `center`, not the per-shape world center). This matches Bullet's compound-child treatment and removes the orbital-motion bias while restoring the translational classification. For the common single-shape case this is equivalent to world-frame supports.

**Status: Fixed.** `populateContinuousCollisionFields` now uses `sup_local + normal · link_center` for both poses. For pure rotation `link_center0 == link_center1` so the result is identical to before; for pure translation `sup_local0 == sup_local1` so `link_sup1 − link_sup0 = normal · link_sweep`, correctly producing `CCType_Time1` or `CCType_Time0`. `nearest_points_local` is now also populated for the `CCType_Time0` and `CCType_Time1` branches (previously left at the zero default). `CCType_Between` projection was also updated to use the link-center trajectory for consistency.

---

### 3. `computeVolume()` returns AABB volume, not swept hull volume

**File:** `coal_casthullshape.cpp:90-123`

The comment says "conservative AABB via the support function" which is correct — but this overapproximates the actual swept volume. For thin sweeps (small rotation, large translation), the AABB can be much larger than the actual swept hull. This is fine for broadphase but could be misleading if used for mass/inertia calculations.

**Severity:** Low — volume is conservative, which is safe for collision checking purposes.

---

## Positive Observations

1. **Functor caching** (`CollisionCacheMap`): Pre-computing `coal::ComputeCollision` functors per object pair and caching GJK guesses is an excellent optimization not present in the Bullet implementation.

2. **Batch broadphase updates**: Using `static_update_`/`dynamic_update_` vectors for batch BVH rebalancing is a good performance optimization.

3. **Per-child cast transforms**: The implementation correctly computes per-shape relative transforms for multi-shape links (matching Bullet's compound shape handling), with a dedicated test (`collision_multi_shape_cast_unit`).

4. **Support function design**: Using Coal's `GEOM_CUSTOM` virtual dispatch through `computeShapeSupport` is cleaner than Bullet's approach of wrapping collision algorithms. The Schulman support function implementation is mathematically sound.

5. **Test coverage**: Good coverage of cast scenarios — box-box, sphere-sphere (both primitive and convex mesh), and multi-shape rotational tests with detailed assertions on cc_time, cc_type, cc_transform, and nearest_points.

6. **GJK configuration**: Correctly using `DefaultGJK` (no Polyak acceleration) for CastHullShape pairs and unbounded `distance_upper_bound` to handle large swept volumes. The `collision_distance_threshold = -gjk_tolerance` fix prevents false positives at security margin boundaries.

7. **cc_time interpolation**: Using center-to-center projection for `CCType_Between` cases avoids artifacts from off-axis mesh vertices, which is a good improvement over naive support-point-based interpolation.

---

## Recommendations

1. Consider adding a test for the `clone()` method's behavior with active cast transforms.

---

## Test coverage gaps (2026-03-19 audit, updated 2026-03-18)

Audited all cast unit tests against the full `ContactResult` struct fields. Findings below.

### `single_contact_point`

Never set by either Bullet or Coal in cast callbacks (both default to `false`). Only discrete tests branch on it. Not a Trajopt/cast concern.

### `type_id`

**Gap (now addressed):** Correctly propagated via `cd->getTypeID()` in Coal's callback, but not tested with non-zero values in any cast test. `collision_cast_cctype_unit` now registers `moving_sphere` with `type_id=7` and asserts `contact.type_id[ki] == 7` and `contact.type_id[si] == 0` for both Bullet and Coal backends.

### `subshape_id`

Already tested via `runStaticOctreeSubshapeIdReportsPrimitiveIdentity` (`subshape_id >= 0` for at least one octree contact). Not a Trajopt gradient concern.

### `nearest_points_local`

**Gap (now addressed):** Not validated in any octree cast test. A zero `nearest_points_local` would cause Trajopt to compute a zero-offset Jacobian for the contact point, producing incorrect gradients. The octree cast tests now assert `nearest_points_local[ki].norm() > 1e-6`. Note: `nearest_points_local` for `CCType_Time1` and `CCType_Time0` was previously left at the zero default and was fixed as part of issue 2 above. `collision_cast_cctype_unit` now asserts the exact round-trip values for both CCType_Time0 and CCType_Time1 (e.g., `transform[ki] * nearest_points_local[ki]` and `cc_transform[ki] * nearest_points_local[ki]`).

### `normal` is a unit vector

**Gap (now addressed):** Only one octree test checked `normal.norm() == 1.0`. Added `EXPECT_NEAR(cr.normal.norm(), 1.0, 1e-4)` to `collision_box_box_cast_unit`, `collision_sphere_sphere_cast_unit` (all four scenarios: primitive scenario 1 & 2, convex scenario 1 & 2), `collision_multi_shape_cast_unit`, and `collision_cast_cctype_unit`.

### `normal` direction for `CCType_Time1`

**Gap (now addressed):** Not validated in octree cast tests. The invariant is: if `cc_type == CCType_Time1`, then the outward normal from the kinematic shape must have a strictly positive component along the sweep direction (this is exactly the condition that selects `CCType_Time1` over `CCType_Between`). Formally: `((ki==0 ? 1 : -1) * cr.normal).dot(sweep_dir) > 0`. The octree cast tests now assert this. `collision_cast_cctype_unit` also asserts `ns * cr.normal[0] > 0.5` for the CCType_Time1 and CCType_Time0 sphere-sphere scenarios.

### `distance` sign

**Gap (now addressed):** Octree cast tests previously only checked `distance < 0.11` (within margin). Since the kinematic shape ends inside the octree, the swept hull must genuinely penetrate a voxel and `distance` must be negative. The octree cast tests now assert `distance < 0.0`.

### `CCType_Time1` and `CCType_Time0` (dedicated scenarios)

**Gap (now addressed):** No test explicitly constructed a scenario that was guaranteed to produce `CCType_Time1` or `CCType_Time0` and verified the exact `cc_time` value (0.0 or 1.0). `collision_cast_cctype_unit` now provides:
- `runTestCCTypeTime1`: moving sphere sweeps in +x into a static sphere. `normal · sweep > 0` → `CCType_Time1`, `cc_time = 1.0`. Runs on both Bullet and Coal.
- `runTestCCTypeTime0`: moving sphere starts overlapping a static sphere and retreats in −x. `normal · sweep < 0` → `CCType_Time0`, `cc_time = 0.0`. Runs on both Bullet and Coal.

### `shape_id` for multi-shape links

**Gap (now addressed):** All existing tests used single-shape links, so `shape_id` was always 0. `collision_multi_shape_cast_unit` now adds `EXPECT_EQ(cr.shape_id[arm_idx], 0)` to verify that the collision is correctly attributed to sub-shape A (index 0, the one that sweeps through the obstacle) and not sub-shape B (index 1).

### Trajopt octree regression analysis

The investigation did not identify a definitive code-level bug from static analysis alone. Probable candidates:

1. **`nearest_points_local` not set for `CCType_Time1`/`CCType_Time0`** (now fixed, issue 2 above): Trajopt uses `nearest_points_local` for Jacobian offset computation. A zero value produces zero-offset Jacobians, making the gradient independent of the contact point position and potentially reducing gradient magnitude to near zero.

2. **GJK functor cache stale guess**: `CollisionCacheMap` caches GJK support hints keyed by `(co1*, co2*)` pointer pair. Between Trajopt optimization iterations the robot configuration changes but the cache key stays the same. A stale warm-start guess from the previous configuration could cause GJK to converge to a different local minimum, producing an incorrect contact normal and thus an incorrect gradient direction. This is harder to detect via static analysis; a targeted test would be needed.
