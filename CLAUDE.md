# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Coal-based collision checking plugin for the Tesseract motion planning framework. Implements discrete and continuous (swept/cast) collision detection using the Coal library, as an alternative to the existing Bullet and FCL backends. Architecture closely mirrors Tesseract's Bullet collision implementation.

The reference Bullet and FCL implementations live in `src/tesseract/tesseract_collision/`.

## Coal fork requirement

Continuous collision detection requires `GEOM_CUSTOM` support in Coal for the `CastHullShape` class. The workspace's `src/coal/` fork provides this. Without it, Coal's narrowphase dispatcher won't recognize custom shapes.

## Architecture

### Key design concepts

- **COW (Collision Object Wrapper):** Each link becomes a COW containing one or more Coal `CollisionObject`s (one per shape). Tracks link name, world pose, filter group, and per-shape local transforms.
- **Dual broadphase managers:** Both discrete and cast managers maintain `static_manager_` and `dynamic_manager_` trees. Only kinematic-to-kinematic and kinematic-to-static pairs are checked. Objects move between managers via `setActiveCollisionObjects()`.
- **Active list management:** `active_` controls which objects are kinematic vs static. Adding a new collision object does **not** auto-add it to `active_`; removing cleans it from `active_`.
- **CastHullShape (continuous collision):** Wraps a `coal::ShapeBase` with a cast transform (start-to-end motion). Uses support functions at both poses for swept volume — no vertex materialization. Independent GJK support hints per pose.
- **Dual COW maps (cast manager):** `link2cow_` has regular geometry; `link2castcow_` has CastHullShape-wrapped versions.
- **Octree handling:** Static octrees use the raw OcTree in the broadphase (Coal has native CastHullShape-vs-OcTree support). Active octrees are expanded into individual boxes per voxel, each wrapped in CastHullShape, for sweep support.
- **Collision functor cache:** `ComputeCollision` functors and `CollisionRequest` objects cached per object pair to preserve GJK warm-start hints.

### Plugin registration

Two factory classes (`CoalDiscreteBVHManagerFactory`, `CoalCastBVHManagerFactory`) registered via `TESSERACT_ADD_DISCRETE_MANAGER_PLUGIN` / `TESSERACT_ADD_CONTINUOUS_MANAGER_PLUGIN`. Built as separate `tesseract_collision_coal_factories` library.

### Test organization (three layers)

1. **Test suite headers** (`test_suite/include/`) — 26 reusable `.hpp` headers defining `addCollisionObjects()` + `runTest()` for each scenario. Headers-only INTERFACE library shared across collision backends.
2. **Integration tests** (`test/*.cpp`) — Instantiate Coal managers and call test suite `runTest()` functions. ~24 tests covering discrete (box-sphere, mesh-mesh, octomap, large dataset, multi-threaded) and continuous (cast box-box, sphere-sphere, octomap cast, multi-shape cast).
3. **Coal-specific unit tests** (`test/coal/`) — 7 tests for internal components: CastHullShape behavior, functor caching/GJK warm-start, geometry cache, collision object wrapper AABB inflation, shape conversion.

Cast tests gated behind `TESSERACT_COLLISION_COAL_ENABLE_COAL_CAST_TESTS` CMake option (default OFF).

## Known issues

See `REVIEW.md` for a detailed code review. Key items:
- `setCollisionObjectsTransform(pose1, pose2)` iterates two TransformMaps in lockstep assuming identical key sets — should use keyed lookup instead.

## Commit conventions

Follow the existing style: imperative mood, concise subject line explaining the "why", body for details when needed.
