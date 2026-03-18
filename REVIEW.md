# Code Review: Coal Continuous Collision Detection Implementation

**Reviewer:** Claude Code
**Date:** 2026-03-11 (updated 2026-03-18)
**Scope:** Coal cast (continuous) collision checking — CastHullShape, cast managers, utilities, and tests

## Summary

This implementation ports Tesseract's Bullet-based continuous collision detection to the Coal library. The architecture closely mirrors Bullet's approach (dual COW maps, CastHullShape as swept convex hull, support-function-based cc_time classification) while adapting to Coal's API (separate static/dynamic broadphase managers, `ComputeCollision` functor caching, `GEOM_CUSTOM` virtual dispatch).

Overall quality is **good**. The code is well-structured, thoroughly tested, and handles edge cases (multi-shape links, octree expansion, disabled objects). Below are specific findings organized by severity.

---

## Issues Found

### ~~1. Potential bug: `setCollisionObjectsTransform(pose1, pose2)` with TransformMap assumes aligned iteration order~~ ✓ Fixed

The lockstep iteration has been replaced with keyed lookup:

```cpp
for (const auto& [name, tf1] : pose1)
{
  auto it2 = pose2.find(name);
  assert(it2 != pose2.end());
  setCollisionObjectsTransform(name, tf1, it2->second);
}
```

### ~~2. `needsCollisionCheck` declared non-inline in header but defined inline in source~~ ✓ Fixed

The spurious `inline` keyword has been removed from the definition in `coal_utils.cpp`; declaration and definition are now consistent.

### 3. `#pragma GCC diagnostic` for non-virtual destructor

**File:** `coal_utils.h:537-547`

```cpp
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
struct CollisionCallback : coal::CollisionCallBackBase { ... };
#pragma GCC diagnostic pop
```

The `CollisionCallback` struct inherits from `coal::CollisionCallBackBase` which apparently lacks a virtual destructor. The pragma suppresses the warning, but the struct declares `virtual ~CollisionCallback() = default;` itself. This is correct for preventing UB when deleting through a base pointer, but the pragma suggests an upstream Coal issue. Consider filing this upstream.

**Severity:** Low — correctly handled, the pragma is just cosmetic since the derived class does declare the virtual destructor.

### ~~4. Clone doesn't preserve cast transforms~~ ✓ Documented

A comment was added to `clone()` explicitly noting that cast transforms are not preserved and that this matches Bullet's behavior:

```cpp
// Note: addCollisionObject creates fresh CastHullShapes with identity cast
// transforms, so any active sweep state (set via setCollisionObjectsTransform
// with pose1/pose2) is not preserved. This matches Bullet's clone behavior.
```

### 5. `computeVolume()` returns AABB volume, not swept hull volume

**File:** `coal_casthullshape.cpp:90-123`

The comment says "conservative AABB via the support function" which is correct — but this overapproximates the actual swept volume. For thin sweeps (small rotation, large translation), the AABB can be much larger than the actual swept hull. This is fine for broadphase but could be misleading if used for mass/inertia calculations.

**Severity:** Low — volume is conservative, which is safe for collision checking purposes.

### 6. Static objects with non-ShapeBase geometry: cast representation used in static manager

**File:** `coal_cast_managers.cpp:576-580` and multiple places

For octree objects, the code registers the **cast** representation in the static manager instead of the regular one. This avoids unsupported CastHull-vs-OcTree collision pairs in Coal's narrowphase. This is a clever workaround but adds complexity throughout the codebase (every broadphase update path needs `hasNonShapeBaseGeometry` checks).

Consider documenting this architectural decision more prominently (perhaps a class-level comment on `CoalCastBVHManager`).

**Severity:** Low — correct but complex.

### 7. Collision cache not invalidated when margin changes

**File:** `coal_cast_managers.cpp:604-641` (`onCollisionMarginDataChanged`)

When collision margins change, the code updates contact distance thresholds and AABBs, but does **not** clear the `collision_cache`. Cached `CollisionRequest` objects have their `security_margin` updated on reuse (line 547), so this is actually handled correctly — the cache stores functors that get updated parameters on each use. This is fine.

**Severity:** None — correctly handled.

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

1. ~~**Fix the TransformMap iteration bug** (Issue #1)~~ ✓ Fixed.
2. Add a class-level architectural comment on `CoalCastBVHManager` explaining the dual-map (link2cow_/link2castcow_) design and the octree special case.
3. Consider adding a test for the `clone()` method's behavior with active cast transforms.
4. ~~The `// TODO: Should the order be flipped?` comments in both managers' `contactTest()` methods should be investigated and resolved.~~ ✓ Resolved (comments removed).
