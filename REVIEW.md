# Code Review: Coal Continuous Collision Detection Implementation

**Reviewer:** Claude Code
**Date:** 2026-03-11 (updated 2026-03-18)
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

**Severity:** Medium — correct behaviour for rotation-dominant motion; semantically incorrect (relative to Bullet) for pure-translational sweeps, with a downstream planner impact if `cc_type` is used for more than display.

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
