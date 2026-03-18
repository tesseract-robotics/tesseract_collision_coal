# tesseract_collision_coal

Coal-based collision checking plugin for the Tesseract motion planning framework.

## What this is

This repository implements discrete and continuous (swept/cast) collision detection using the [Coal](https://github.com/coal-library/coal) library, as an alternative to the existing Bullet and FCL backends in Tesseract. The architecture closely mirrors Tesseract's Bullet collision implementation.

The reference Bullet and FCL implementations live at:
https://github.com/tesseract-robotics/tesseract/tree/master/collision

## Coal fork requirement

Continuous collision detection requires custom geometry support (`GEOM_CUSTOM`) in Coal for the `CastHullShape` class. This is provided by the following branch, which must be used instead of upstream Coal:

https://github.com/rjoomen/coal/tree/copilot/review-custom-cast-shape-support

Without this branch, `CastHullShape` (the swept convex hull used for cast collision) will not work — Coal's narrowphase dispatcher won't recognize `GEOM_CUSTOM` shapes.

## Repository structure

```
tesseract/collision/
├── coal/
│   ├── include/tesseract/collision/coal/
│   │   ├── coal_discrete_managers.h      # DiscreteContactManager implementation
│   │   ├── coal_cast_managers.h          # ContinuousContactManager implementation
│   │   ├── coal_utils.h                  # COW, collision callbacks, filter logic
│   │   ├── coal_collision_object_wrapper.h  # AABB-inflated CollisionObject
│   │   ├── coal_casthullshape.h          # Swept geometry via support functions
│   │   ├── coal_collision_geometry_cache.h
│   │   └── coal_factories.h             # Plugin factory registration
│   └── src/                             # Corresponding implementations
├── test/
│   ├── coal/                            # Coal-specific unit tests
│   └── *.cpp                            # Integration tests (shared test suite)
└── test_suite/include/                  # Shared test harness headers
```

## Key design concepts

### Collision Object Wrapper (COW)
Each Tesseract link becomes a `COW` containing one or more Coal `CollisionObject`s (one per shape in the link). The COW tracks the link name, world pose, filter group, and per-shape local transforms.

### Dual broadphase managers
Both discrete and cast managers maintain separate `static_manager_` and `dynamic_manager_` broadphase trees. Only kinematic-to-kinematic and kinematic-to-static pairs are checked. Objects move between managers when `setActiveCollisionObjects()` is called.

### Active list management
The `active_` list controls which objects are kinematic (checked against everything) vs static (checked only against kinematic). Callers manage this via `setActiveCollisionObjects()`. Adding a new collision object does **not** auto-add it to `active_`; removing an object does clean it from `active_`.

### CastHullShape (continuous collision)
Wraps a `coal::ShapeBase` with a cast transform (start-to-end motion). Uses the support function of the underlying shape at both poses to compute the convex hull of the swept volume — no vertex materialization needed. Each pose maintains independent GJK support hints.

### Dual COW maps for cast manager
`CoalCastBVHManager` maintains two parallel COW hierarchies:
- `link2cow_` — regular geometry
- `link2castcow_` — CastHullShape-wrapped versions for swept collision

### Octree handling
Coal lacks native CastHull-vs-OcTree narrowphase, so octrees are expanded into individual boxes per occupied voxel, each wrapped in CastHullShape.

### Collision functor cache
`ComputeCollision` functors and `CollisionRequest` objects are cached per object pair to avoid reconstruction cost and to preserve GJK warm-start hints.

## Building

The project uses CMake with `ros_industrial_cmake_boilerplate`. Source directory is `tesseract/collision/`.

```bash
mkdir build && cd build
cmake ../tesseract/collision -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

Dependencies: Coal, Eigen3, octomap, boost_plugin_loader, tesseract (collision + geometry interfaces).

Cast tests are gated behind `TESSERACT_COLLISION_COAL_ENABLE_COAL_CAST_TESTS` (default OFF).

## Known issues

See `REVIEW.md` for a detailed code review. Key items:
- `setCollisionObjectsTransform(pose1, pose2)` iterates two TransformMaps in lockstep assuming identical key sets — should use keyed lookup instead.

## Commit conventions

Follow the existing style: imperative mood, concise subject line explaining the "why", body for details when needed.
