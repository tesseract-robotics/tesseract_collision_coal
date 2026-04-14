/**
 * @file coal_utils.h
 * @brief Tesseract Coal Utility Functions.
 *
 * @author Roelof Oomen, Levi Armstrong
 * @date Dec 18, 2017
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (BSD)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TESSERACT_COLLISION_COAL_UTILS_H
#define TESSERACT_COLLISION_COAL_UTILS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <boost/functional/hash.hpp>
#include <coal/broadphase/broadphase_collision_manager.h>
#include <coal/collision.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/types.h>
#include <tesseract/collision/common.h>

#include <tesseract/collision/coal/coal_collision_object_wrapper.h>

namespace tesseract::collision::tesseract_collision_coal
{
using CollisionGeometryPtr = std::shared_ptr<coal::CollisionGeometry>;
using CollisionObjectPtr = std::shared_ptr<CoalCollisionObjectWrapper>;
using CollisionObjectRawPtr = coal::CollisionObject*;

/** @brief A pair of collision objects used as map key */
using CollisionObjectPair = std::pair<coal::CollisionObject*, coal::CollisionObject*>;

/** @brief Hash functor for CollisionObjectPair */
struct CollisionObjectPairHash
{
  std::size_t operator()(const CollisionObjectPair& p) const noexcept { return boost::hash_value(p); }
};

/** @brief Cached collision functor and GJK warm-start state for a collision object pair */
struct CollisionCacheEntry
{
  coal::CollisionRequest request;
  coal::ComputeCollision functor;
  bool is_cast{ false };  ///< Cached at creation to avoid dynamic_cast on every re-seed.
                          ///< Controls GJK seed strategy: cast pairs cannot use BoundingVolumeGuess.
  uint32_t gen0{ 0 };     ///< COW generation when GJK guess was last seeded (shape 0).
  uint32_t gen1{ 0 };     ///< COW generation when GJK guess was last seeded (shape 1).
};

/** @brief Cache mapping collision object pairs to their precomputed collision functor and warm-start state */
using CollisionCacheMap = std::unordered_map<CollisionObjectPair, CollisionCacheEntry, CollisionObjectPairHash>;

/// Default GJK guess validity threshold (5mm). Stale GJK warm-start guesses from larger
/// moves can cause solver failures (zero gradients, degraded contact accuracy).
/// Configurable per-manager via the plugin YAML config key `gjk_guess_threshold`.
inline constexpr double kDefaultGJKGuessThreshold = 5e-3;

/// Default d_arc compensation setting (disabled). When enabled, CastHullShape's swept-sphere
/// radius is set to the arc-chord sagitta of the shape's rotation, compensating for the gap
/// between the convex hull (chord) and the true swept arc in continuous collision checks.
/// Only used by the cast (continuous) manager. Configurable via the plugin YAML config key
/// `d_arc_compensation`.
inline constexpr bool kDefaultDArcCompensation = false;

/** @brief Compute a tight AABB for any ShapeBase subclass by dispatching to the
 *  type-specific computeBV specialization based on getNodeType().
 *  Falls back to computeBV<AABB, ShapeBase> for unrecognized types (GEOM_CUSTOM etc.).
 *  @pre s.computeLocalAABB() must have been called (fallback reads aabb_local). */
void computeShapeAABB(const coal::ShapeBase& s, const coal::Transform3s& tf, coal::AABB& bv);

/** @brief Erase cache entries involving any of the given collision objects (for object removal) */
void invalidateCacheFor(CollisionCacheMap& cache, const std::vector<CollisionObjectPtr>& objects);

enum CollisionFilterGroups : std::int8_t
{
  DefaultFilter = 1,
  StaticFilter = 2,
  KinematicFilter = 4,
  AllFilter = -1  // all bits sets: DefaultFilter | StaticFilter | KinematicFilter
};

/**
 * @brief This is a Tesseract link collision object wrapper which add items specific to tesseract. It is a wrapper
 * around a tesseract link which may contain several collision objects.
 */
class CollisionObjectWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CollisionObjectWrapper>;
  using ConstPtr = std::shared_ptr<const CollisionObjectWrapper>;

  CollisionObjectWrapper() = default;
  CollisionObjectWrapper(tesseract::common::LinkId id,
                         const int& type_id,
                         CollisionShapesConst shapes,
                         tesseract::common::VectorIsometry3d shape_poses);

  short int m_collisionFilterGroup{ CollisionFilterGroups::StaticFilter };
  short int m_collisionFilterMask{ CollisionFilterGroups::KinematicFilter };
  bool m_enabled{ true };
  uint32_t gjk_generation_{ 1 };  ///< Monotonic counter for GJK guess invalidation. Starts at 1 so new cache entries
                                  ///< (initialized to 0) trigger initial GJK seeding on first use.

  const tesseract::common::LinkId& getLinkId() const { return link_id_; }
  const int& getTypeID() const { return type_id_; }

  const CollisionShapesConst& getCollisionGeometries() const { return shapes_; }

  CollisionShapesConst& getCollisionGeometries() { return shapes_; }

  const tesseract::common::VectorIsometry3d& getCollisionGeometriesTransforms() const { return shape_poses_; }

  tesseract::common::VectorIsometry3d& getCollisionGeometriesTransforms() { return shape_poses_; }

  void setCollisionObjectsTransform(const Eigen::Isometry3d& pose);

  void setContactDistanceThreshold(double contact_distance);

  double getContactDistanceThreshold() const { return contact_distance_; }

  const Eigen::Isometry3d& getCollisionObjectsTransform() const { return world_pose_; }

  const std::vector<CollisionObjectPtr>& getCollisionObjects() const { return collision_objects_; }

  std::vector<CollisionObjectPtr>& getCollisionObjects() { return collision_objects_; }

  /** @brief Append raw pointers from this wrapper's collision objects into @p out. */
  void appendCollisionObjectsRaw(std::vector<CollisionObjectRawPtr>& out) const;

  std::shared_ptr<CollisionObjectWrapper> clone() const;

  /**
   * @brief Given Coal collision shape get the index to the links collision shape
   * @param co Coal collision shape
   * @return links collision shape index
   */
  static int getShapeIndex(const coal::CollisionObject* co);

protected:
  tesseract::common::LinkId link_id_;                              // id derived from name, also carries the name string
  int type_id_{ -1 };                                             // user defined type id
  Eigen::Isometry3d world_pose_{ Eigen::Isometry3d::Identity() }; /**< @brief Collision Object World Transformation */
  CollisionShapesConst shapes_;
  tesseract::common::VectorIsometry3d shape_poses_;
  std::vector<CollisionGeometryPtr> collision_geometries_;
  std::vector<CollisionObjectPtr> collision_objects_;

  double contact_distance_{ 0 }; /**< @brief The contact distance threshold */
};

CollisionGeometryPtr createShapePrimitive(const CollisionShapeConstPtr& geom);

using COW = CollisionObjectWrapper;
using Link2COW =
    std::unordered_map<tesseract::common::LinkId, COW::Ptr, tesseract::common::LinkId::Hash>;

COW::Ptr createCoalCollisionObject(const tesseract::common::LinkId& id,
                                   const int& type_id,
                                   const CollisionShapesConst& shapes,
                                   const tesseract::common::VectorIsometry3d& shape_poses,
                                   bool enabled);

/** @brief Check if two transforms differ beyond the tolerance threshold */
bool transformChanged(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b);

/**
 * @brief Apply the collision filter mask based on the current filter group.
 *
 * StaticFilter groups can only collide with KinematicFilter groups.
 * KinematicFilter groups can collide with both StaticFilter and KinematicFilter groups.
 */
void applyCollisionFilterMask(COW& cow);

/**
 * @brief Update collision objects filters
 * @param active_ids Set of active collision object LinkIds
 * @param cow The collision object to update
 * @param static_manager Broadphase manager for static objects
 * @param dynamic_manager Broadphase manager for dynamic objects
 */
void updateCollisionObjectFilters(
    const std::unordered_set<tesseract::common::LinkId, tesseract::common::LinkId::Hash>& active_ids,
    const COW::Ptr& cow,
    const std::unique_ptr<coal::BroadPhaseCollisionManager>& static_manager,
    const std::unique_ptr<coal::BroadPhaseCollisionManager>& dynamic_manager);

/**
 * @brief Update collision objects filters for continuous collision checking
 * @param active_ids Set of active collision object LinkIds
 * @param cow The regular collision object
 * @param cast_cow The cast collision object
 * @param static_manager Broadphase manager for static objects
 * @param dynamic_manager Broadphase manager for dynamic objects
 */
void updateCollisionObjectFilters(
    const std::unordered_set<tesseract::common::LinkId, tesseract::common::LinkId::Hash>& active_ids,
    const COW::Ptr& cow,
    COW::Ptr& cast_cow,
    const std::unique_ptr<coal::BroadPhaseCollisionManager>& static_manager,
    const std::unique_ptr<coal::BroadPhaseCollisionManager>& dynamic_manager);

/**
 * @brief Create a cast collision object from a regular collision object
 * @param cow The collision object to convert
 * @param expand_octrees When false, octree shapes are kept as raw OcTree geometry (deferred expansion)
 * @return A new collision object with shapes converted to CastHullShapes
 */
COW::Ptr makeCastCollisionObject(const COW::Ptr& cow, bool expand_octrees = true);

/**
 * @brief Check if a cast COW contains unexpanded octrees that need expansion for sweep support.
 * Expanded shapes have GEOM_CUSTOM (CastHullShape); unexpanded octrees have GEOM_OCTREE.
 */
inline bool castCowNeedsOctreeExpansion(const COW::Ptr& cast_cow)
{
  for (const auto& co : cast_cow->getCollisionObjects())
    if (co->collisionGeometry()->getNodeType() == coal::GEOM_OCTREE)
      return true;
  return false;
}

/**
 * @brief This is used to check if a collision check is required between the provided two collision objects
 * @param cow1 The first collision object
 * @param cow2 The second collision object
 * @param validator  The contact allowed validator
 * @param verbose Indicate if verbose information should be printed to the terminal
 * @return True if the two collision objects should be checked for collision, otherwise false
 */
bool needsCollisionCheck(const CollisionObjectWrapper* cd1,
                         const CollisionObjectWrapper* cd2,
                         const std::shared_ptr<const tesseract::common::ContactAllowedValidator>& validator,
                         bool verbose = false);

struct ContactTestDataWrapper : ContactTestData
{
  CollisionCacheMap* collision_cache{ nullptr };
};

// Disable warnings about non-virtual destructor for coal::CollisionCallBackBase
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"

struct CollisionCallback : coal::CollisionCallBackBase
{
  ContactTestDataWrapper* cdata{};
  bool collide(coal::CollisionObject* o1, coal::CollisionObject* o2) override;
  virtual ~CollisionCallback() = default;
};

#pragma GCC diagnostic pop

}  // namespace tesseract::collision::tesseract_collision_coal
#endif  // TESSERACT_COLLISION_COAL_UTILS_H
