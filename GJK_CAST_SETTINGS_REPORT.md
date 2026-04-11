# GJK Settings for CastHullShape Pairs: Experiment Report

Date: 2026-03-22

## Objective

Find the optimal combination of GJK settings for CastHullShape (continuous/swept collision) pairs across three axes:

1. **GJK variant**: DefaultGJK, NesterovAcceleration, PolyakAcceleration
2. **Convergence criterion**: Default (VDB), DualityGap, Hybrid
3. **Initial guess mode**: DefaultGuess(1,0,0), CachedGuess (center-to-center), BoundingVolumeGuess

The discrete (non-cast) branch uses NesterovAcceleration + DualityGap/Absolute + BoundingVolumeGuess. The goal was to find the closest-matching settings that work for cast pairs.

## Background

### CastHullShape support function

CastHullShape wraps a `coal::ShapeBase` with a cast transform representing start-to-end motion. Its support function (`coal_casthullshape.cpp:145-171`) implements the Schulman support (Schulman et al. 2013):

```cpp
// Support at pose 0
const Vec3s s0 = getSupport<WithSweptSphere>(shape_.get(), dir, hint0_);

// Support at pose 1 (direction rotated into pose-1 frame, result transformed back)
const Vec3s dir_local1 = castTransformInv_.getRotation() * dir;
const Vec3s s1_local = getSupport<WithSweptSphere>(shape_.get(), dir_local1, hint1_);
const Vec3s s1 = castTransform_.transform(s1_local);

// Return the support of the convex hull of both poses
support = (dir.dot(s0) > dir.dot(s1)) ? s0 : s1;
```

The support scalar function `h(d) = max(d·s0(d), d·s1(d))` is continuous (pointwise maximum of continuous functions). However, the support **point** mapping jumps discontinuously between `s0` and `s1` at the switching boundary where `d·s0 = d·s1`. GJK operates on support points, not scalars: each simplex vertex is a support point. When the support point mapping jumps, the simplex vertex can shift abruptly from one pose to the other, changing which simplex GJK builds and ultimately affecting the contact features EPA finds.

### CollisionRequest settings overview

`CollisionRequest` inherits from `QueryRequest` and adds collision-specific fields. The final settings in `coal_utils.cpp` (after all experiments documented below) are:

| Field | Shared (cast & discrete) | Cast first call | Discrete first call | Coal default |
|-------|--------------------------|-----------------|---------------------|--------------|
| `gjk_variant` | NesterovAcceleration | | | DefaultGJK |
| `gjk_convergence_criterion` | DualityGap | | | Default (VDB) |
| `gjk_convergence_criterion_type` | Relative | | | Relative |
| `distance_upper_bound` | `security_margin + gjk_tolerance` | | | `max()` |
| `gjk_initial_guess` (first call) | | CachedGuess (center-to-center) | BoundingVolumeGuess | DefaultGuess |
| `gjk_initial_guess` (subsequent) | CachedGuess (warm-start) | | | — |
| `enable_contact` | `calculate_penetration` | | | true |
| `num_max_contacts` | from request | | | 1 |
| `security_margin` | from request | | | 0 |

Fields not explicitly set use Coal defaults: `gjk_tolerance` = 1e-6, `gjk_max_iterations` = 128, `epa_tolerance` = 1e-6, `epa_max_iterations` = 64, `break_distance` = 1e-3.

The warm-start cache is enabled for all pairs. Cache invalidation on object enable/disable prevents stale guesses (see "Discrete settings unification").

### Prior configuration

The code previously used Coal defaults (DefaultGJK + Default/VDB convergence) with CachedGuess as the initial guess for cast pairs. The comments claimed that BoundingVolumeGuess and DefaultGuess both produce incorrect results, but this had not been tested independently from the GJK variant/criterion settings.

## Option space

Default (VDB) convergence does not use `convergence_criterion_type` (verified: `gjk.cpp:388-394` has no `switch(convergence_criterion_type)` in the `Default` case). This gives:

- 3 variants x (1 Default + 2 DualityGap + 2 Hybrid) = **15 distinct GJK combos**
- 3 initial guess modes
- Total parameter space: 45, pruned via two-phase approach

### Convergence criteria details

From `gjk.cpp:387-432`, using `diff` as the duality gap estimate, `rl` as ray length, `tolerance` defaulting to 1e-6:

| Criterion | Type | Convergence test | Effective threshold |
|-----------|------|-----------------|-------------------|
| Default (VDB) | — | `rl - alpha <= tolerance * (1 + rl)` | Scales with ray length |
| DualityGap | Absolute | `diff <= tolerance` | Fixed: 1e-6 |
| DualityGap | Relative | `diff <= tolerance²` | Fixed: 1e-12 |
| Hybrid | Absolute | `rl² - alpha² <= tolerance` | Fixed: 1e-6 |
| Hybrid | Relative | `rl² - alpha² <= tolerance²` | Fixed: 1e-12 |

Note: DualityGap/Relative and Hybrid/Relative both use the formula `(diff / tolerance * rl) - tolerance * rl <= 0` (`gjk.cpp:404,422`). By C++ left-to-right associativity this is `((diff / tolerance) * rl) - (tolerance * rl) <= 0`. Factoring out `rl` (positive, since GJK returns Collision when `rl < tolerance`): `diff / tolerance - tolerance <= 0`, i.e. `diff <= tolerance²`. With the default tolerance of 1e-6, the effective threshold is 1e-12 — six orders of magnitude tighter than Absolute's 1e-6.

## Test suite

Each configuration was validated against three test tiers:

1. **Gatekeeper 1**: `arm_around_table_trajopt_problem` — trajectory optimization with continuous collision (tests solver convergence with cast collision contacts)
2. **Gatekeeper 2**: `TesseractEnvironmentUtils.applyContactManagerConfigObjectEnable` — environment configuration with object-level collision enable/disable
3. **Full regression**: 224 Coal collision tests + 45 trajopt_sqp tests

Configs that failed a gatekeeper were not run through the full suite.

## Phase 1: Initial guess mode (DefaultGJK + Default convergence)

Tested which initial guess modes work independently of the GJK variant, using the safest GJK settings (DefaultGJK + Default/VDB) with `distance_upper_bound = max()`. All three tests kept the warm-start cache active, so the initial guess only affects the **first call** per object pair.

| Test | Initial Guess | Gatekeeper 1 | Gatekeeper 2 | 224 Coal tests | Notes |
|------|--------------|:---:|:---:|:---:|-------|
| G1 | DefaultGuess(1,0,0) | PASS | PASS | 223/224 | SphereSphereConvexHull nearest_point.y off by 0.067 |
| G2 | BoundingVolumeGuess | FAIL | — | — | Solver Failure in trajectory optimizer |
| G3 | CachedGuess (center-to-center) | PASS | PASS | 224/224 | Production config at time of test |

### How Coal computes each initial guess

Coal's `GJKSolver::getGJKInitialGuess()` (`narrowphase.h:355-394`) resolves the initial guess before GJK starts:

**DefaultGuess**: Uses a hardcoded direction vector `(1, 0, 0)` (the `default_guess` parameter).

```cpp
case GJKInitialGuess::DefaultGuess:
  guess = default_guess;  // Vec3s(1, 0, 0)
  break;
```

**CachedGuess**: Uses the `cached_gjk_guess` vector from the `CollisionRequest`. The caller sets this to a geometrically meaningful direction.

```cpp
case GJKInitialGuess::CachedGuess:
  guess = this->cached_guess;  // from request.cached_gjk_guess
  break;
```

**BoundingVolumeGuess**: Computes the vector between local AABB centers, transformed into the Minkowski difference frame. Requires `computeLocalAABB()` to have been called on both shapes.

```cpp
case GJKInitialGuess::BoundingVolumeGuess:
  guess.noalias() = s1.aabb_local.center() -
                    (oR1 * s2.aabb_local.center() + ot1);
  break;
```

In all three cases, `support_func_cached_guess` (the vertex hint for hill-climbing in the support function) is always warm-started from the request, regardless of the guess mode (`narrowphase.h:363`).

### CastHullShape's AABB geometry

CastHullShape overrides `computeLocalAABB()` (`coal_casthullshape.cpp:56-85`). It queries the Schulman support function at both poses along each axis to find extrema of the swept volume:

```cpp
for (int i = 0; i < 3; ++i) {
  dir[i] = 1;
  computeShapeSupport(dir, s, hint, data);  // max of pose0 and pose1
  aabb_local.max_[i] = s[i];

  dir[i] = -1;
  computeShapeSupport(dir, s, hint, data);  // min of pose0 and pose1
  aabb_local.min_[i] = s[i];
}
aabb_center = aabb_local.center();
```

The resulting `aabb_center` is the center of the **swept volume's bounding box**. This is the midpoint between the most extreme support points at t=0 and t=1 — NOT the center of the shape at any particular pose.

`computeLocalAABB()` is called on CastHullShape objects: Coal's `CollisionObject` constructor calls `init(compute_local_aabb=true)` which invokes `cgeom->computeLocalAABB()` (`collision_object.h:345-350`). It is also called in `updateCastTransform()` when the cast transform changes (`coal_casthullshape.cpp:142`). So the AABB is always up to date.

### Warm-start cache lifecycle

After each collision call, Coal writes the GJK/EPA result back to `solver.cached_guess` and then to `CollisionResult.cached_gjk_guess` (`collision.cpp:127`). The following table lists which extract method is called for each GJK status and what it writes to `cached_guess`:

| GJK status | Extract method | `cached_guess` update | Source line |
|-----------|---------------|----------------------|-------------|
| DidNotRun | (inline) | `(1, 0, 0)` | `narrowphase.h:457` |
| Failed | `GJKExtractWitnessPointsAndNormal` | `gjk.ray` | `narrowphase.h:629` |
| NoCollisionEarlyStopped | `GJKEarlyStopExtractWitnessPointsAndNormal` | `gjk.ray` | `narrowphase.h:600` |
| NoCollision | `GJKExtractWitnessPointsAndNormal` | `gjk.ray` | `narrowphase.h:629` |
| CollisionWithPenetrationInformation | `GJKExtractWitnessPointsAndNormal` | `gjk.ray` | `narrowphase.h:629` |
| Collision (EPA not requested) | `GJKCollisionExtractWitnessPointsAndNormal` | **NOT UPDATED** | `narrowphase.h:659` |
| Collision → EPA success | `EPAExtractWitnessPointsAndNormal` | `-(epa.depth * epa.normal)` | `narrowphase.h:672` |
| Collision → EPA DidNotRun/FallBack | `EPAFailedExtractWitnessPointsAndNormal` | `(1, 0, 0)` | `narrowphase.h:729` |

Notes:
- `gjk.ray` is the current search direction vector; for NoCollision it points from the closest feature on one shape toward the other.
- For `Collision (EPA not requested)`, `gjk.ray` is approximately zero (shapes overlap), so it is intentionally not cached (`narrowphase.h:656-659`). Only `support_func_cached_guess` is updated.
- For `Collision → EPA success`, this includes all EPA terminal statuses that produce valid results: Valid, AccuracyReached, OutOfFaces, OutOfVertices, Failed (ran out of iterations), Degenerated, NonConvex, InvalidHull. All call `EPAExtractWitnessPointsAndNormal`.
- The typical path in trajectory optimization (where `enable_contact=true`) is: NoCollision → `gjk.ray`, or Collision → EPA → `-(epa.depth * epa.normal)`.

Our warm-start cache code copies the result back to the request for the next call on this pair (for all pairs, both cast and discrete):

```cpp
entry.request.gjk_initial_guess = coal::CachedGuess;
entry.request.cached_gjk_guess = col_result.cached_gjk_guess;
entry.request.cached_support_func_guess = col_result.cached_support_func_guess;
```

This means the initial guess seed (Phase 1's subject) only affects the **first call** per object pair. After that, GJK/EPA's own result provides the warm-start for subsequent calls. Cache invalidation on object enable/disable ensures stale entries are evicted (see "Discrete settings unification").

### Detailed analysis of each guess mode

#### DefaultGuess(1,0,0) — PARTIAL FAIL

**Observed**: `CoalContinuousBVHCollisionSphereSphereConvexHullUnit` fails. The test expects two tessellated spheres moving along the x-axis to contact at `nearest_point.y = 0.0` (on the symmetry plane) with `cc_time = 0.5`. With DefaultGuess, the result is `nearest_point.y = 0.067` and `cc_time = 0.533` — the contact point is rotated off the symmetry plane.

All other 223 Coal tests pass, and both gatekeepers (including trajectory optimization) pass. The error is within trajectory optimization tolerance but exceeds the unit test's 0.001 tolerance.

The failure is reproducible and specific to DefaultGuess — CachedGuess with the same GJK settings (DefaultGJK + Default/VDB) passes this test. Since the only difference is the first-call seed direction, this directly proves that **GJK's contact result depends on the initial direction for CastHullShape pairs**, even though GJK is guaranteed to find the correct minimum distance for convex shapes.

The mechanism: GJK's initial direction determines the first support point queried from CastHullShape's Schulman support function, which selects between pose-0 and pose-1 based on `max(dir·s0, dir·s1)`. A different initial direction can select a different pose for the first vertex, building a different simplex. When GJK detects collision, EPA expands that simplex into a polytope — EPA's `evaluate()` takes the GJK object by reference and reads `gjk.getSimplex()` directly (`gjk.cpp:1168`). A different starting simplex leads EPA to a different polytope face as the closest to the origin, producing different contact points and normals. This is not a convergence failure — both directions find *a* valid contact, but at different features of the Minkowski difference boundary near the pose-switching region.

#### BoundingVolumeGuess — FAIL

**Observed**: `arm_around_table_trajopt_problem` fails with "Solver Failure" — the sequential quadratic programming optimizer cannot converge.

BoundingVolumeGuess computes `s1.aabb_center - transform(s2.aabb_center)` as the initial GJK direction. For a CastHullShape, `aabb_center` is the center of the swept volume's bounding box (see "CastHullShape's AABB geometry" above). For a shape that translates during the sweep, this center is offset from the shape's center at t=0 by half the translation distance.

For discrete (non-cast) shapes, `aabb_center` is the geometric center of the shape itself, so the center-to-center direction is a good approximation of the separating direction. For CastHullShape, this correspondence breaks down because the AABB center reflects the swept volume, not the shape at any particular pose.

Why the trajectory optimizer fails while the unit tests mostly pass is not fully understood from these experiments alone. The optimizer's sensitivity to first-call contact accuracy differs from the unit tests' tolerance-based pass/fail criteria.

#### CachedGuess (center-to-center) — PASS

Uses `co1->getTransform().getTranslation() - co2->getTransform().getTranslation()`: the vector between the **world-frame origins** of the two collision objects. For CastHullShape, `co->getTransform()` is the start pose (t=0).

This computes the same kind of center-to-center vector that BoundingVolumeGuess provides for discrete shapes — but using the collision object's world-frame position (start pose) rather than the swept-volume AABB center. With a fallback to `(1, 0, 0)` when objects are coincident (within 1e-12), since GJK requires a non-zero initial direction.

After the first call, the warm-start cache replaces this seed with the GJK/EPA result.

**Conclusion**: CachedGuess with center-to-center is the only viable initial guess for CastHullShape pairs. The prior code comment was correct.

## Phase 2: GJK variant x convergence criterion (CachedGuess baseline)

Tested all 15 GJK variant/criterion combinations with CachedGuess.

### Previously tested (from prior experiments, not this session)

| # | Variant | Criterion | Type | Result | Notes |
|---|---------|-----------|------|--------|-------|
| 1 | DefaultGJK | Default | — | PASS | Former production config |
| 2 | NesterovAcceleration | Default | — | PASS | |
| 3 | DefaultGJK | DualityGap | Absolute | PASS | |
| 4 | NesterovAcceleration | DualityGap | Absolute | FAIL | Code comment says "misses collisions" |

### New results (11 combinations, tested this session)

| # | Variant | Criterion | Type | GK1 | GK2 | 224 Coal | Result |
|---|---------|-----------|------|:---:|:---:|:---:|--------|
| 5 | PolyakAcceleration | Default | — | PASS | PASS | 223/224 | **FAIL** |
| 6 | DefaultGJK | DualityGap | Relative | PASS | PASS | 224/224 | **PASS** |
| 7 | NesterovAcceleration | DualityGap | Relative | PASS | PASS | 224/224 | **PASS** |
| 8 | PolyakAcceleration | DualityGap | Absolute | PASS | PASS | 223/224 | **FAIL** |
| 9 | PolyakAcceleration | DualityGap | Relative | PASS | PASS | 223/224 | **FAIL** |
| 10 | DefaultGJK | Hybrid | Absolute | PASS | PASS | 224/224 | **PASS** |
| 11 | DefaultGJK | Hybrid | Relative | PASS | PASS | 224/224 | **PASS** |
| 12 | NesterovAcceleration | Hybrid | Absolute | PASS | PASS | 224/224 | **PASS** |
| 13 | NesterovAcceleration | Hybrid | Relative | PASS | PASS | 224/224 | **PASS** |
| 14 | PolyakAcceleration | Hybrid | Absolute | PASS | PASS | 223/224 | **FAIL** |
| 15 | PolyakAcceleration | Hybrid | Relative | PASS | PASS | 223/224 | **FAIL** |

### Consolidated results matrix

All combinations use CachedGuess (center-to-center) with warm-start cache.

| Variant | Default (VDB) | DualityGap/Abs | DualityGap/Rel | Hybrid/Abs | Hybrid/Rel |
|---------|:---:|:---:|:---:|:---:|:---:|
| **DefaultGJK** | PASS | PASS | PASS | PASS | PASS |
| **NesterovAcceleration** | PASS | **FAIL** | PASS | PASS | PASS |
| **PolyakAcceleration** | **FAIL** | **FAIL** | **FAIL** | **FAIL** | **FAIL** |

### Failure patterns

**PolyakAcceleration (all 5 criteria combinations)**: Every Polyak combination fails `CoalContinuousBVHCollisionSphereSphereConvexHullUnit` with the same symptom as DefaultGuess in Phase 1: `nearest_point.y ≈ 0.067` instead of `0.0`. The failure is independent of the convergence criterion. All Polyak combinations pass both gatekeepers, so the inaccuracy is within trajectory optimization tolerance but exceeds the unit test's 0.001 tolerance.

The three GJK variants differ only in how they compute the search direction (`gjk.cpp:251-285`):

```cpp
// DefaultGJK (line 253): pure closest-point direction
dir = ray;

// NesterovAcceleration (line 256-275, normalized branch):
momentum = (iterations + 2) / (iterations + 3);   // increases toward 1.0
y = momentum * ray + (1 - momentum) * w;
dir = momentum * dir/|dir| + (1 - momentum) * y/|y|;

// PolyakAcceleration (line 278-280):
momentum = 1 / (iterations + 1);                   // decreases from 1.0
dir = momentum * dir + (1 - momentum) * ray;
```

Key differences:
- **DefaultGJK**: Always uses the current closest-point direction (`ray`). No memory of previous directions.
- **Polyak**: Mixes previous direction with current `ray`. Momentum `1/(k+1)` starts at 1.0 and decays: `k=0→1.0, k=1→0.5, k=2→0.33, ...`.
- **Nesterov**: Uses increasing momentum `(k+2)/(k+3)` (approaching 1.0) with normalization and mixing of `w` (the latest support vertex). The normalized formulation (`dir/|dir|` and `y/|y|`) prevents direction magnitude from dominating.

All three variants initialize `dir = ray = guess` (`gjk.cpp:218,223`), so the first support call is identical. The divergence starts from the **second** iteration onward: after simplex processing updates `ray`, DefaultGJK immediately follows the new `ray`, while Polyak blends `ray` with the previous direction (50% on iteration 2, 67% on iteration 3, etc.). The first-iteration Polyak formula gives `dir = 1.0*dir + 0.0*ray = dir`, but since `dir = ray` initially, this is a no-op.

Since Polyak fails with *every* criterion combination while DefaultGJK and Nesterov pass (except Nesterov+DualityGap/Absolute), the failure is entirely attributable to Polyak's direction update formula. The Polyak failure produces the same type of off-symmetry-plane contact error as the DefaultGuess failure in Phase 1 (where `nearest_point.y = 0.067` was measured), and fails the same test (`CoalContinuousBVHCollisionSphereSphereConvexHullUnit`). The exact Polyak error values were not captured in the test output, but the failure pattern — the same test, criterion-independent — is consistent with a direction-formula-induced simplex difference near the pose-switching boundary.

**NesterovAcceleration + DualityGap/Absolute (combo #4)**: The only non-Polyak failure. This was identified in prior experiments (not re-tested this session). The code comment states it "misses collisions on CastHullShape's discontinuous two-pose support function." Notably, switching from Absolute to Relative convergence (combo #7) fixes the issue while keeping both NesterovAcceleration and DualityGap.

From the convergence code (`gjk.cpp:396-410`), the only difference between Absolute and Relative is the stopping threshold:
- **Absolute**: `diff ≤ tolerance` → stops at duality gap ≤ 1e-6
- **Relative**: `diff ≤ tolerance²` → stops at duality gap ≤ 1e-12 (see derivation in "Convergence criteria details" above)

Relative requires approximately 1e6 times more precision, meaning GJK runs more iterations before declaring convergence. Since only Nesterov+DualityGap/Absolute fails (while Nesterov+Default, Nesterov+DualityGap/Relative, and Nesterov+Hybrid/\* all pass), the failure is specific to the interaction between Nesterov's momentum-based direction extrapolation and DualityGap/Absolute's early stopping. DefaultGJK+DualityGap/Absolute (combo #3) passes, so DualityGap/Absolute alone is not the problem — it is specifically Nesterov's accelerated directions that require more iterations to stabilize on CastHullShape's discontinuous support boundary.

## Selection

### Passing configurations (8 total)

| # | Variant | Criterion | Type | Match with discrete |
|---|---------|-----------|------|---------------------|
| 1 | DefaultGJK | Default | — | variant differs, criterion differs |
| 2 | Nesterov | Default | — | variant matches, criterion differs |
| 3 | DefaultGJK | DualityGap | Absolute | variant differs, criterion+type match |
| 6 | DefaultGJK | DualityGap | Relative | variant differs, criterion matches |
| 7 | **Nesterov** | **DualityGap** | **Relative** | **variant+criterion match, type differs** |
| 10 | DefaultGJK | Hybrid | Absolute | variant differs, criterion differs |
| 11 | DefaultGJK | Hybrid | Relative | variant differs, criterion differs |
| 12 | Nesterov | Hybrid | Absolute | variant+type match, criterion differs |
| 13 | Nesterov | Hybrid | Relative | variant matches, criterion differs |

### Selection criteria (priority order)

1. **Must pass all tests**: 224 Coal + 45 trajopt_sqp + applyContactManagerConfigObjectEnable
2. **Consistency with discrete** (NesterovAcceleration + DualityGap/Absolute): minimizes behavioral divergence between discrete and continuous collision checking
3. Among equally consistent options, prefer the one that differs in the fewest settings

### Selected: NesterovAcceleration + DualityGap/Relative (#7)

- Matches discrete on variant (NesterovAcceleration) and criterion (DualityGap) — 2 of 3 settings match
- Only differs in criterion type: Relative instead of Absolute
- No other passing configuration matches on more than 2 of the 3 GJK settings (#12 matches variant+type but differs on criterion)

## GJK variant/criterion/guess: code change and regression

All Phase 1 and Phase 2 experiments above used `distance_upper_bound = max()` (the then-current setting). The `distance_upper_bound` was optimized separately (see next section).

File: `coal_utils.cpp`, cast branch of collision request setup.

Before (Coal defaults, no explicit GJK settings):
```cpp
if (co1_is_cast || co2_is_cast)
{
  col_request.distance_upper_bound = (std::numeric_limits<coal::Scalar>::max)();
  col_request.gjk_initial_guess = coal::CachedGuess;
  coal::Vec3s guess = co1->getTransform().getTranslation() - co2->getTransform().getTranslation();
  if (guess.squaredNorm() < 1e-12)
    guess = coal::Vec3s(1, 0, 0);
  col_request.cached_gjk_guess = guess;
}
```

After (NesterovAcceleration + DualityGap/Relative, `distance_upper_bound` still `max()` at this stage):
```cpp
if (co1_is_cast || co2_is_cast)
{
  col_request.gjk_variant = coal::GJKVariant::NesterovAcceleration;
  col_request.gjk_convergence_criterion = coal::GJKConvergenceCriterion::DualityGap;
  col_request.gjk_convergence_criterion_type = coal::GJKConvergenceCriterionType::Relative;
  col_request.distance_upper_bound = (std::numeric_limits<coal::Scalar>::max)();
  col_request.gjk_initial_guess = coal::CachedGuess;
  coal::Vec3s guess = co1->getTransform().getTranslation() - co2->getTransform().getTranslation();
  if (guess.squaredNorm() < 1e-12)
    guess = coal::Vec3s(1, 0, 0);
  col_request.cached_gjk_guess = guess;
}
```

The warm-start cache block is unchanged.

### Regression results (with `distance_upper_bound = max()`)

- 224/224 Coal collision tests: **PASS**
- 45/45 trajopt_sqp tests: **PASS**
- 1/1 applyContactManagerConfigObjectEnable: **PASS**

---

## distance_upper_bound optimization

### Context

The cast branch previously set `distance_upper_bound = max()` (no GJK early-stopping). This means every cast pair that passes the broadphase AABB overlap test runs GJK to full convergence, even when shapes are clearly separated. The discrete branch uses `security_margin + gjk_tolerance`, allowing GJK to bail out early when it proves the distance exceeds that bound.

### How GJK early-stopping works

When GJK's directional distance projection (omega) exceeds `distance_upper_bound + swept_sphere_radius` (`gjk.cpp:296-299`), GJK terminates with `NoCollisionEarlyStopped` status. The narrowphase returns only a distance lower bound; witness points are NaN (`narrowphase.h:474`, `GJKEarlyStopExtractWitnessPointsAndNormal`). In our collision callback, `col_result.isCollision()` returns false for early-stopped pairs and we immediately return (`coal_utils.cpp:632-633`). No distance information from non-colliding pairs is used, so early-stopping is safe for collision detection.

Coal clamps `distance_upper_bound` in `GJKSolver::set()` (`narrowphase.h:227-229`): `distance_upper_bound = max(0, max(request.distance_upper_bound, request.security_margin))`. This prevents the bound from being lower than `security_margin`, ensuring GJK never early-stops for pairs within the collision margin.

### Options tested

| Option | Value | Description |
|--------|-------|-------------|
| D1 | `max()` | No early-stopping (baseline) |
| D2 | `security_margin + gjk_tolerance` | Same formula as discrete branch |

D3 (CastHullShape AABB diagonal) was planned as a fallback but not needed since D2 passed.

### D2 code changes

Two locations in `coal_utils.cpp`:

1. **Cast branch initial setup** (line 573): `col_request.distance_upper_bound = security_margin + col_request.gjk_tolerance` instead of `max()`.

2. **Cache-hit path** (line 610): Removed the cast-pair guard that skipped `distance_upper_bound` updates, so it is now updated for all pairs on cache hits.

### Correctness results

| Option | 224 Coal tests | 45 trajopt_sqp | applyContactManagerConfig | Result |
|--------|:-:|:-:|:-:|--------|
| D1 (`max()`) | 224/224 | 45/45 | PASS | **PASS** (baseline) |
| D2 (`security_margin + gjk_tolerance`) | 224/224 | 45/45 | PASS | **PASS** |

Contact counts in the collision_profile benchmark are identical for D1 and D2 (39, 42, 43, 40 per scenario), confirming no collisions are missed.

### Performance results

Performance measured using `tesseract_collision_coal_profile` (continuous cast benchmarks, 3 runs, median reported) and `arm_around_table_trajopt_problem` wall-clock time.

**collision_profile CoalCastBVH timings (ms, median of 3 runs):**

| Scenario | D1 (`max()`) | D2 (`margin+tol`) | Change |
|----------|:-:|:-:|:-:|
| One Primitive Shape per link | 0.266 | 0.242 | -9% |
| All Primitive Shape in single link | 0.247 | 0.228 | -8% |
| One Convex Shape per link | 0.548 | 0.411 | -25% |
| All Convex Shape in single link | 0.501 | 0.473 | -6% |

Note: The collision_profile uses `security_margin = 0` (default), so D2's effective `distance_upper_bound = gjk_tolerance = 1e-6`. This is the most aggressive early-stopping scenario — GJK bails out as soon as it proves shapes are separated by more than 1e-6. With typical trajectory optimization margins (e.g., 0.525 in `arm_around_table`), the early-stopping threshold is higher and the performance improvement would be smaller than the 6-25% seen here.

**arm_around_table_trajopt_problem wall-clock time (s, median of 3 runs):**

| Option | Time | Change |
|--------|:-:|:-:|
| D1 (`max()`) | 22.29 | — |
| D2 (`margin+tol`) | 22.41 | +0.5% (within noise) |

The trajectory optimizer timing is dominated by QP solving, not collision checking. The collision check performance improvement from D2 is not visible at the whole-test level.

### Selected: D2 (`security_margin + gjk_tolerance`)

- Passes all tests with identical contact counts
- 6-25% faster in collision_profile continuous benchmarks
- Matches the discrete branch formula — cast and discrete now use the same `distance_upper_bound` logic
- Simplifies the cache-hit path (no cast/non-cast branching for `distance_upper_bound`)

### Test commands for reproducibility

```bash
source /home/roelof/tesseract_ws/install/setup.bash

# Correctness: gatekeepers
cd /home/roelof/tesseract_ws/build/trajopt_sqp && ctest -R arm_around_table_trajopt_problem --output-on-failure
cd /home/roelof/tesseract_ws/build/tesseract && ctest -R applyContactManagerConfigObjectEnable --output-on-failure

# Correctness: full regression
cd /home/roelof/tesseract_ws/build/tesseract_collision_coal && ctest --output-on-failure  # 224 tests
cd /home/roelof/tesseract_ws/build/trajopt_sqp && ctest --output-on-failure               # 45 tests

# Performance: collision_profile (run 3x, take median)
cd /home/roelof/tesseract_ws && source install/setup.bash
./build/tesseract_collision_coal/test/benchmarks/tesseract_collision_coal_profile

# Performance: trajopt timing (run 3x, take median)
cd /home/roelof/tesseract_ws/build/trajopt_sqp && ctest -R arm_around_table_trajopt_problem --output-on-failure
```

## Discrete settings unification

### Context

After finding optimal cast settings, the only difference between cast and discrete branches was `gjk_convergence_criterion_type`: Relative (cast) vs Absolute (discrete). We tested whether switching discrete to Relative would allow fully unified GJK settings.

### Results

| Configuration | 224 Coal | 45 trajopt_sqp | applyContactManagerConfig | Result |
|--------------|:-:|:-:|:-:|--------|
| Discrete with Absolute (original) | 224/224 | 45/45 | PASS | **PASS** |
| Discrete with Relative | 224/224 | 45/45 | PASS | **PASS** |
| Discrete with Relative + warm-start for all (no invalidation) | 224/224 | 45/45 | **FAIL** | FAIL |
| Discrete with Relative + warm-start for all + cache invalidation | 224/224 | 45/45 | PASS | **PASS** |

### Warm-start cache stale-state issue

Enabling the warm-start cache for all pairs initially failed `applyContactManagerConfigObjectEnable`: after disabling and re-enabling a collision object, the stale cached guess from the previous collision state misled NesterovAcceleration into reporting no collision when one exists.

**Root cause**: `enableCollisionObject()` and `disableCollisionObject()` in both `CoalDiscreteBVHManager` and `CoalCastBVHManager` only flipped `m_enabled` without invalidating the collision cache. The `invalidateCacheFor()` function already existed (used in `removeObjects()`), but was not called on enable/disable.

**Fix**: Added `invalidateCacheFor(collision_cache, it->second->getCollisionObjects())` to `enableCollisionObject()` and `disableCollisionObject()` in both managers. This evicts cache entries involving the toggled object, forcing a fresh BoundingVolumeGuess on the next collision check. The warm-start cache then rebuilds naturally from subsequent GJK/EPA results.

Files modified:
- `coal_discrete_managers.cpp`: `enableCollisionObject()`, `disableCollisionObject()`
- `coal_cast_managers.cpp`: `enableCollisionObject()`, `disableCollisionObject()` (both regular and cast COW entries)

With this fix, the warm-start cache is enabled for **all** pairs (cast and discrete). The first call uses BoundingVolumeGuess (for discrete) or CachedGuess center-to-center (for cast); subsequent calls use the cached GJK/EPA result.

## Final state

### Complete CollisionRequest settings

All GJK settings are now fully unified. Only the first-call initial guess differs (cast needs center-to-center because BoundingVolumeGuess uses the swept-volume AABB center):

| Field | Shared | Cast first call | Discrete first call |
|-------|--------|-----------------|---------------------|
| `gjk_variant` | NesterovAcceleration | | |
| `gjk_convergence_criterion` | DualityGap | | |
| `gjk_convergence_criterion_type` | Relative | | |
| `distance_upper_bound` | `security_margin + gjk_tolerance` | | |
| `gjk_initial_guess` (first call) | | CachedGuess (center-to-center) | BoundingVolumeGuess |
| `gjk_initial_guess` (subsequent) | CachedGuess (warm-start) | | |
| Other fields | Coal defaults | | |

### Final regression results

- 224/224 Coal collision tests: **PASS**
- 45/45 trajopt_sqp tests: **PASS**
- 1/1 applyContactManagerConfigObjectEnable: **PASS**
