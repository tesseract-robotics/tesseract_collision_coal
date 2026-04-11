# String-Key Elimination Plan

## Problem

String-keyed lookups are the largest remaining actionable bottleneck (~8% CPU). Every broadphase pair triggers string hashing, comparison, and copying through multiple maps.

### Current perf numbers (post-optimizations A-I)

| Symbol | % | Source |
|--------|---|--------|
| `__memcmp_avx2_movbe` | 2.03% | String comparisons across all hash/tree lookups |
| `TransformMap::operator[]` | 2.01% | FK result map (string-keyed, upstream in kinematics) |
| `boost::hash_range<char*>` | 1.19% | String hashing for unordered_map lookups |
| `ACM _M_find_before_node` (pair\<string,string\>) | 0.86% | Allowed collision matrix lookup |
| `collectCastTransformUpdate` | 0.80% | Iterates link2cow_/link2castcow_ (string-keyed std::map) |
| `hash<pair<string,string>>` | 0.80% | Hashing string pairs for ACM/margin |
| `CollisionCallback::collide` | 0.75% | Contains string ops internally |
| `_Hash_bytes` | 0.69% | MurmurHash for std::string keys |
| `string::_M_assign` | 0.50% | String copies (ContactResult link_names, makeOrderedLinkPair) |
| `_Rb_tree<string, COW::Ptr>::find` | 0.40% | link2cow_ lookup in setCollisionObjectsTransform |
| `margin _M_find_before_node` | 0.30% | Collision margin pair lookup |
| `makeOrderedLinkPair` | 0.23% | String comparison + 2 string copies |

**Note:** `CollisionCacheMap::_M_find_before_node` (1.07%) is pointer-pair keyed, not string — out of scope.

---

## Architecture

### Two strategies for two problems

**Problem A: Associative lookups (ACM, margin, link2cow_).** These map string keys to values. Fix: integer link IDs for O(1) lookup with trivial hash/compare.

**Problem B: Transform pipeline (setCollisionObjectsTransform).** The manager receives a string-keyed FK map, then does a *second* string lookup in `link2cow_` per entry. Fix: pre-resolve COW pointers at setup time. The FK map string lookup is unavoidable (upstream interface), but the `link2cow_.find()` is not.

These are separated because pre-resolved COW pointers are simpler and faster for the transform path, while integer IDs are the right tool for associative containers.

### ID registry: per-manager, on the COW

Each collision manager assigns a `LinkId` (uint16_t) to each COW during `addCollisionObject()`. The ID is stored on the COW itself. Clone copies it (COW::clone copies all fields). IDs are dense (0..N-1) enabling vector-indexed lookup.

### Layering: no upstream changes

`AllowedCollisionMatrix` and `CollisionMarginData` live in tesseract_common and don't know about `LinkId`. The integer fast paths live on the **collision manager**, not on the upstream types. The manager builds integer lookup tables from the string-based ACM/margin data at setup/clone time.

The `ContactAllowedValidator` virtual interface (`operator()(const string&, const string&)`) is not modified. Instead, `needsCollisionCheck` is changed to check an integer ACM *before* falling through to the string-based validator for cases not covered (e.g. custom validators).

---

## Steps

### Step 1: LinkId on COW + cow_by_id_ on manager

**Files:** `coal_utils.h`, `coal_discrete_managers.h/cpp`, `coal_cast_managers.h/cpp`

- Add `using LinkId = uint16_t;` (in coal_utils.h)
- Add `LinkId link_id_{0};` member + `LinkId getLinkId() const` accessor to `CollisionObjectWrapper`
- Add `LinkId next_link_id_{0};` and `std::vector<COW::Ptr> cow_by_id_;` to both managers
- In `addCollisionObject()`: assign `cow->link_id_ = next_link_id_++`, append to `cow_by_id_`
- `clone()` copies these naturally (addCollisionObject on cloned COWs preserves the ID since the counter starts fresh and links are added in the same order — verify this or explicitly copy the ID)

**Keep `link2cow_`** for name-based APIs (hasCollisionObject, removeCollisionObject, etc.). `cow_by_id_` is the fast path.

### Step 2: Pre-resolved COW pointers for transform updates

**Files:** `coal_discrete_managers.h/cpp`, `coal_cast_managers.h/cpp`

**Current hot path:**
```
setCollisionObjectsTransform(const TransformMap& transforms):
  for (auto& [name, tf] : transforms)
    auto it = link2cow_.find(name);  // O(log n) string compare per link
    collectTransformUpdate(it, tf);
```

**Change:**
- Add `std::vector<std::pair<std::string, COW*>> active_cow_cache_;` to each manager
- Populate in `setActiveCollisionObjects()`: for each active link name, resolve `link2cow_.find(name)` once and store the raw `COW*`
- New `setCollisionObjectsTransform` implementation: iterate `active_cow_cache_`, look up transform in input map by the cached string key, call `collectTransformUpdate` with the pre-resolved COW pointer directly
- For cast manager: same pattern with parallel `active_castcow_cache_`

The raw `COW*` is safe because: (a) COWs are owned by `link2cow_` (shared_ptr), (b) the active set and link2cow_ are frozen after setup in trajectory optimization, (c) any mutation (add/remove object) must rebuild the cache

**Impact:** Eliminates `_Rb_tree::find` (0.40%) and most of `collectCastTransformUpdate` string overhead (0.80%). The remaining cost is the input TransformMap lookup (unordered_map, O(1) amortized).

### Step 3: Integer ACM on the collision manager

**Files:** `coal_utils.h`, `coal_discrete_managers.h/cpp`, `coal_cast_managers.h/cpp`, `coal_utils.cpp`

**Current:** `needsCollisionCheck()` → `isContactAllowed(cd1->getName(), cd2->getName(), validator)` → `validator(name1, name2)` → `ACM::isCollisionAllowed()` does `unordered_map<pair<string,string>, string>::find`.

**Change:**
- Add helper: `inline uint32_t makeLinkIdPair(LinkId a, LinkId b)` (ordered, packed)
- Add `std::unordered_set<uint32_t> acm_integer_;` to each manager
- Populate in `setContactAllowedValidator()`: iterate all link pairs in `cow_by_id_`, query the string-based validator, store allowed pairs in `acm_integer_`
- Also rebuild when `setActiveCollisionObjects()` changes (active set determines which pairs matter)
- In `needsCollisionCheck()`: check `acm_integer_.count(makeLinkIdPair(cd1->getLinkId(), cd2->getLinkId()))` — single uint32_t hash+lookup
- Fall through to string-based validator only if the manager detects a custom (non-ACM) validator that might have dynamic behavior

**Impact:** Eliminates ACM `_M_find_before_node` (0.86%), `hash<pair<string,string>>` (0.80%), `isCollisionAllowed` (0.20%), `isContactAllowed` (0.21%), `makeOrderedLinkPair` (0.23%). Major reduction in `__memcmp_avx2_movbe` and `_Hash_bytes`.

### Step 4: Integer margin lookup on the collision manager

**Files:** `coal_utils.h`, `coal_discrete_managers.h/cpp`, `coal_cast_managers.h/cpp`, `coal_utils.cpp`

**Current:** `collide()` calls `collision_margin_data.getCollisionMargin(cd1->getName(), cd2->getName())` — string hashing + comparison.

**Change:**
- Add `std::unordered_map<uint32_t, double> margin_integer_;` to each manager (or flat vector if pair count is small)
- Populate in `setCollisionMarginData()`: iterate pair margins, resolve names to IDs, store in `margin_integer_`
- Add `getCollisionMargin(LinkId a, LinkId b)` method on manager that checks `margin_integer_` first, falls back to default margin
- Call from `collide()` using COW link IDs

**Impact:** Eliminates margin `_M_find_before_node` (0.30%) and related hashing.

### Step 5: Eliminate string copies in ContactResult

**Files:** `coal_utils.cpp`

**Current:** `contact.link_names[0] = cd1->getName();` — 2 string copies per contact point.

**Change:**
- Store `const std::string*` (pointer to COW's name) instead of copying: `contact.link_names_ptr[0] = &cd1->getName();`
- Or: add `std::array<LinkId, 2> link_ids` to ContactResult, populate from COW IDs, defer string population to when consumers actually need names (debugging, output)
- Hot-path consumers in trajopt (`getCollisionMargin`, `getCollisionCoeff`) would use link IDs

**Risk:** Medium. ContactResult is a public type in tesseract_collision core. Adding fields is safe; removing/changing `link_names` requires downstream audit. Safest approach: add `link_ids` alongside `link_names`, stop populating `link_names` in the Coal backend, add a lazy resolver.

**Impact:** Eliminates `string::_M_assign` (0.50%).

### Out of scope

- **`TransformMap::operator[]` (2.01%):** This is the FK side — `KDLStateSolver::calculateTransformsHelper` writing transforms via `operator[]` on an `unordered_map<string, Isometry3d>`. Eliminating this requires changing the kinematics interface to produce integer-keyed or vector-indexed transforms. That's a cross-package boundary change in tesseract_kinematics — significant scope, separate effort.
- **`CollisionCacheMap::_M_find_before_node` (1.07%):** Pointer-pair keyed, not string.

---

## Expected impact

| Step | Eliminates | Estimated savings |
|------|-----------|------------------|
| Step 1: LinkId on COW | (foundation) | — |
| Step 2: Pre-resolved COW ptrs | `_Rb_tree::find` (0.40%), part of `collectCast...` (0.80%) | ~1.0% |
| Step 3: Integer ACM | ACM find (0.86%), hash (0.80%), isAllowed (0.41%), makeOrdered (0.23%) | ~1.5-2.0% |
| Step 4: Integer margin | margin find (0.30%) + hashing | ~0.3% |
| Step 5: Integer ContactResult | `string::_M_assign` (0.50%) | ~0.5% |
| Indirect: reduced memcmp/hash | part of memcmp (2.03%) + hash_bytes (0.69%) | ~1.0% |
| **Total** | | **~4-5%** |

## Constraints

- **No upstream changes:** AllowedCollisionMatrix, CollisionMarginData, ContactAllowedValidator, and the FK interface are untouched. Integer fast paths live on the Coal collision managers.
- **API compatibility:** String-based public APIs preserved. Integer paths are internal.
- **Incremental:** Steps 1→2→3 are the critical path. Steps 4-5 are independent follow-ups.
- **Clone safety:** IDs are on COWs (copied by clone). Pre-resolved caches rebuilt in `setActiveCollisionObjects()`. No dangling pointers across clone boundaries.
