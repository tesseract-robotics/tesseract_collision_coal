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

class CastHullShape;

/** @brief Cached collision functor and GJK warm-start state for a collision object pair */
struct CollisionCacheEntry
{
  coal::CollisionRequest request;
  coal::ComputeCollision functor;
  bool is_cast{ false };       ///< Cached at creation to avoid a dynamic_cast per contact.
                               ///< Gates population of the continuous-collision fields on each result.
  bool multi_leaf{ false };    ///< Cached at creation. Marks a pair with no usable warm start, which
                               ///< keeps the bounding-volume guess it was constructed with.
  bool sweeps_first{ false };  ///< Whether pair_hull sweeps the key's first object rather than its second.
  uint32_t gen0{ 0 };          ///< COW generation when GJK guess was last seeded (shape 0).
  uint32_t gen1{ 0 };          ///< COW generation when GJK guess was last seeded (shape 1).
  /// Set for a pair of two swept objects, null for every other pair. Such a pair is collided as one
  /// object's plain shape against this hull: the scratch hull of the other object, which every pair
  /// sweeping that object shares, and which is therefore rewritten with the object's motion relative to
  /// its partner before every query. Not owned: it lives as long as the swept object's own hull, and an
  /// entry does not outlive its objects.
  CastHullShape* pair_hull{ nullptr };
  /// The geometry the functor collides as its first object.
  const coal::CollisionGeometry* first_geometry{ nullptr };
};

/** @brief Cache mapping collision object pairs to their precomputed collision functor and warm-start state */
using CollisionCacheMap = std::unordered_map<CollisionObjectPair, CollisionCacheEntry, CollisionObjectPairHash>;

/// Default d_arc compensation setting (disabled). When enabled, CastHullShape's swept-sphere
/// radius is set to the arc-chord sagitta of the shape's rotation, compensating for the gap
/// between the convex hull (chord) and the true swept arc in continuous collision checks.
/// A pair of two moving links is collided through a hull of their relative motion, which is
/// padded by the sagitta of their relative turn instead, computed on every narrowphase query
/// of the pair. That is the arc the pair's hull cuts only when one joint joins the two links or
/// one of them does not move; see CoalCastBVHManager. Only used by the cast (continuous)
/// manager. Configurable via the plugin YAML config key `d_arc_compensation`.
inline constexpr bool kDefaultDArcCompensation = false;

/** @brief Compute an AABB for a ShapeBase at transform tf, dispatching on getNodeType().
 *  Primitive shapes (Box, Sphere, Capsule, ...) use their exact analytic computeBV
 *  specialization. The shapes with no such specialization -- convex hulls, GEOM_CUSTOM,
 *  future types -- are bounded from @p local_aabb by the conservative O(1) computeBV
 *  formula rather than the exact O(num_points) per-vertex fit, which is too costly for the
 *  tightness it buys on the per-check cast path (computeLocalAABB).
 *  The shape's swept-sphere radius is read live and added on every branch, so a caller must
 *  not expand for it again. No branch reads s.aabb_local, which coal refreshes only on
 *  computeLocalAABB() and leaves stale on setSweptSphereRadius(), so no bound here can
 *  inherit a stale radius and come out smaller than the shape.
 *  @param local_aabb the shape's local AABB *without* its swept-sphere radius. Passing a
 *  bound that already includes the radius counts it twice. */
void computeShapeAABB(const coal::ShapeBase& s,
                      const coal::Transform3s& tf,
                      const coal::AABB& local_aabb,
                      coal::AABB& bv);

/** @brief Bound a shape in its own frame, exactly, without writing to it.
 *
 *  The bound excludes the shape's swept-sphere radius, matching computeShapeAABB's @p local_aabb
 *  contract. Primitives take their analytic computeBV specialization; convex hulls take the exact
 *  O(num_points) fit, which is affordable because callers bound a shape once when they wrap it
 *  rather than once per check.
 *
 *  Unlike coal::CollisionGeometry::computeLocalAABB, this writes nothing to @p s. Collision
 *  geometry is shared between per-thread manager clones and, through CoalCollisionGeometryCache,
 *  between environments, so refreshing a shape to read its bound is a write another thread may be
 *  reading.
 *
 *  @return false for a node type with no exact form here -- GEOM_CUSTOM and any third-party
 *  ShapeBase -- leaving @p bv untouched. A bound for those would have to come from s.aabb_local,
 *  which may be unset and would risk a bound smaller than the shape. */
bool computeTightLocalAABB(const coal::ShapeBase& s, coal::AABB& bv);

/** @brief Erase cache entries involving any of the given collision objects (for object removal) */
void invalidateCacheFor(CollisionCacheMap& cache, const std::vector<CollisionObjectPtr>& objects);

/** @brief Erase cache entries involving any object of either list, in a single pass over the cache
 *
 *  Cheaper than two calls of the single-list overload: a pass is linear in the cache, which holds an
 *  entry per pair ever checked, while the pointer set it tests against is linear in the objects. */
void invalidateCacheFor(CollisionCacheMap& cache,
                        const std::vector<CollisionObjectPtr>& objects,
                        const std::vector<CollisionObjectPtr>& other_objects);

/** @brief Unregister collision objects from a broadphase manager, leaving the cache untouched
 *
 *  @pre Every object is registered in @p manager. Coal's dynamic AABB tree looks an object up
 *  unconditionally and removes the node it gets back, so unregistering one the manager does not hold
 *  removes a null node. */
void unregisterObjects(const std::vector<CollisionObjectPtr>& objects, coal::BroadPhaseCollisionManager& manager);

/** @brief Unregister collision objects from a broadphase manager and erase the cache entries naming them */
void removeObjects(CollisionCacheMap& cache,
                   const std::vector<CollisionObjectPtr>& objects,
                   coal::BroadPhaseCollisionManager& manager);

/**
 * @brief Broadphase filter groups, mirroring the Bullet backend's set.
 *
 * A wrapper's m_collisionFilterGroup only ever holds StaticFilter or KinematicFilter. DefaultFilter is
 * carried over from Bullet and unused here. Test the group through isStatic()/isKinematic() rather
 * than comparing it directly.
 */
enum CollisionFilterGroups : std::int8_t
{
  DefaultFilter = 1,
  StaticFilter = 2,
  KinematicFilter = 4,
  AllFilter = -1  // all bits sets: DefaultFilter | StaticFilter | KinematicFilter
};

class CollisionObjectWrapper;

/**
 * @brief What a link's regular and cast wrapper share: the link's shapes, filter state and collision objects.
 *
 * Not used on its own: setting the link's pose is left to each wrapper type. A collision object's user data
 * names its wrapper as this type.
 */
class CollisionObjectWrapperBase
{
public:
  // Neither copied nor moved: a wrapper's collision objects name it by address, through their user data.
  // CollisionObjectWrapper::clone makes a copy with collision objects of its own.
  CollisionObjectWrapperBase(const CollisionObjectWrapperBase&) = delete;
  CollisionObjectWrapperBase& operator=(const CollisionObjectWrapperBase&) = delete;
  CollisionObjectWrapperBase(CollisionObjectWrapperBase&&) = delete;
  CollisionObjectWrapperBase& operator=(CollisionObjectWrapperBase&&) = delete;

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

  void setContactDistanceThreshold(double contact_distance);

  double getContactDistanceThreshold() const { return contact_distance_; }

  const Eigen::Isometry3d& getCollisionObjectsTransform() const { return world_pose_; }

  const std::vector<CollisionObjectPtr>& getCollisionObjects() const { return collision_objects_; }

  std::vector<CollisionObjectPtr>& getCollisionObjects() { return collision_objects_; }

  /** @brief Append raw pointers from this wrapper's collision objects into @p out. */
  void appendCollisionObjectsRaw(std::vector<CollisionObjectRawPtr>& out) const;

  /**
   * @brief Given Coal collision shape get the index to the links collision shape
   * @param co Coal collision shape
   * @return links collision shape index
   */
  static int getShapeIndex(const coal::CollisionObject* co);

protected:
  CollisionObjectWrapperBase() = default;
  CollisionObjectWrapperBase(tesseract::common::LinkId&& id,
                             const int& type_id,
                             CollisionShapesConst&& shapes,
                             tesseract::common::VectorIsometry3d&& shape_poses);
  ~CollisionObjectWrapperBase() = default;

  /** @brief Set the link's pose, and place every collision object by it. */
  void setCollisionObjectsTransform(const Eigen::Isometry3d& pose);

  /** @brief Make this wrapper a copy of @p other, with collision objects of its own. Only a regular wrapper is
   *  copied: a copy of a cast wrapper would share its hulls.
   *  @pre This wrapper holds no collision objects. */
  void cloneFrom(const CollisionObjectWrapper& other);

  tesseract::common::LinkId link_id_;                             // id derived from name, also carries the name string
  int type_id_{ -1 };                                             // user defined type id
  Eigen::Isometry3d world_pose_{ Eigen::Isometry3d::Identity() }; /**< @brief Collision Object World Transformation */
  CollisionShapesConst shapes_;
  tesseract::common::VectorIsometry3d shape_poses_;
  std::vector<CollisionObjectPtr> collision_objects_;

  double contact_distance_{ 0 }; /**< @brief The contact distance threshold */
};

/**
 * @brief This is a Tesseract link collision object wrapper which add items specific to tesseract. It is a wrapper
 * around a tesseract link which may contain several collision objects.
 */
class CollisionObjectWrapper : public CollisionObjectWrapperBase
{
public:
  using Ptr = std::shared_ptr<CollisionObjectWrapper>;
  using ConstPtr = std::shared_ptr<const CollisionObjectWrapper>;

  CollisionObjectWrapper() = default;
  CollisionObjectWrapper(tesseract::common::LinkId id,
                         const int& type_id,
                         CollisionShapesConst shapes,
                         tesseract::common::VectorIsometry3d shape_poses)
    : CollisionObjectWrapperBase(std::move(id), type_id, std::move(shapes), std::move(shape_poses))
  {
  }

  /** @brief Set the link's pose. */
  using CollisionObjectWrapperBase::setCollisionObjectsTransform;

  std::shared_ptr<CollisionObjectWrapper> clone() const;
};

/// The displacement of a link that is not swept: exactly the identity.
inline const Eigen::Isometry3d kNoSweepDisplacement{ Eigen::Isometry3d::Identity() };

/**
 * @brief The wrapper of a link as a continuous manager sweeps it: the link's shapes as CastHullShapes, and
 * the sweep they are set to.
 *
 * The sweep is held twice - per shape in the hulls, which the narrowphase collides, and per link here,
 * which a contact's fields and a pair of two swept links read - and the two agree only while nothing writes
 * one without the other. setSweep writes both, and is the only way this type offers to set the link's pose.
 *
 * A wrapper made for a static link is deferred: it holds the link's own geometry in place of hulls and takes
 * no sweep until it is built. See makeCastCollisionObject and castCowNeedsSweptBuild.
 *
 * A cast wrapper is built from its link's regular wrapper and has no clone, because a copy would share its
 * hulls.
 */
class CastCollisionObjectWrapper final : public CollisionObjectWrapperBase
{
public:
  using Ptr = std::shared_ptr<CastCollisionObjectWrapper>;
  using ConstPtr = std::shared_ptr<const CastCollisionObjectWrapper>;

  /** @brief A copy of @p link as CollisionObjectWrapper::clone makes one, not swept.
   *  @param link The link's regular wrapper
   *  @param d_arc_compensation See getDArcCompensation */
  CastCollisionObjectWrapper(const CollisionObjectWrapper& link, bool d_arc_compensation);

  /**
   * @brief Set the link's sweep: from @p pose1 to @p pose2
   *
   * Equal poses are the unswept state, in the hulls and here. Otherwise each hull is padded by the arc sagitta
   * of the link's turn if the wrapper was made with arc compensation.
   *
   * The hulls are written first and the wrapper's poses after: a hull's cast transform determines its
   * geometry's local AABB, and setting the wrapper's poses derives each object's own AABB from that. So the
   * objects' bounds are current on return, whether or not the link moved.
   *
   * @pre The wrapper is not deferred. A built cast wrapper holds CastHullShapes and nothing else -
   * makeCastCollisionObject either converts a shape or throws - so the downcast inside is safe exactly then.
   * Check castCowNeedsSweptBuild first.
   *
   * @param pose1 The link's world pose at the start of the sweep; may be a pose this wrapper holds
   * @param pose2 The link's world pose at the end of the sweep; may be a pose this wrapper holds
   * @return Whether any hull was rewritten. A hull that changes while the link stays put still has to reach
   * the broadphase, so a change here is not implied by the link having moved.
   */
  bool setSweep(const Eigen::Isometry3d& pose1, const Eigen::Isometry3d& pose2);

  /** @brief The pose the link's current sweep ends at; its world pose when it is not swept. */
  const Eigen::Isometry3d& getSweepEndTransform() const { return swept_ ? sweep_end_pose_ : world_pose_; }

  /** @brief Whether the link's current sweep ends at a pose other than the one it starts at. */
  bool isSwept() const { return swept_; }

  /** @brief Whether a swept hull of this link is padded by the arc sagitta of the turn it is swept through; see
   *  kDefaultDArcCompensation. Fixed when the wrapper is made. */
  bool getDArcCompensation() const { return d_arc_compensation_; }

  /** @brief The link's world displacement over its sweep, end * start^-1. Exactly the identity when the
   *  sweep ends at the pose it starts at.
   *
   *  Worked out on the first call after a sweep is set, of this or of the inverse, so not safe to call from
   *  two threads at once. */
  const Eigen::Isometry3d& getSweepDisplacement() const
  {
    if (!swept_)
      return kNoSweepDisplacement;
    if (!sweep_displacement_current_)
      computeSweepDisplacement();
    return sweep_displacement_;
  }

  /** @brief The inverse of getSweepDisplacement, start * end^-1; exactly the identity likewise. */
  const Eigen::Isometry3d& getSweepDisplacementInverse() const
  {
    if (!swept_)
      return kNoSweepDisplacement;
    if (!sweep_displacement_current_)
      computeSweepDisplacement();
    return sweep_displacement_inverse_;
  }

private:
  /** @brief See getDArcCompensation. */
  bool d_arc_compensation_;
  /** @brief Whether the link is swept. The sweep members below are read only while it is. */
  bool swept_{ false };
  /** @brief Whether the two displacements below match the sweep. Few swept links meet another one in the
   *  narrowphase, so they are worked out for the first reader rather than on every sweep set. */
  mutable bool sweep_displacement_current_{ false };

  Eigen::Isometry3d sweep_end_pose_{ Eigen::Isometry3d::Identity() }; /**< @brief World pose the sweep ends at */
  mutable Eigen::Isometry3d sweep_displacement_{ Eigen::Isometry3d::Identity() };         /**< @brief end * start^-1 */
  mutable Eigen::Isometry3d sweep_displacement_inverse_{ Eigen::Isometry3d::Identity() }; /**< @brief start * end^-1 */

  /** @brief Bring the two displacements up to date with the sweep. */
  void computeSweepDisplacement() const;
};

CollisionGeometryPtr createShapePrimitive(const CollisionShapeConstPtr& geom);

using COW = CollisionObjectWrapper;
using Link2COW = std::unordered_map<tesseract::common::LinkId, COW::Ptr>;
using CastCOW = CastCollisionObjectWrapper;
using Link2CastCOW = std::unordered_map<tesseract::common::LinkId, CastCOW::Ptr>;

COW::Ptr createCoalCollisionObject(const tesseract::common::LinkId& id,
                                   const int& type_id,
                                   const CollisionShapesConst& shapes,
                                   const tesseract::common::VectorIsometry3d& shape_poses,
                                   bool enabled);

/**
 * @brief Build the collision object wrappers for a batch of specs
 *
 * Reproduces the per-id semantics of adding the specs one at a time: a repeated id displaces its earlier
 * entry, so the last spec naming an id decides both the wrapper and its position in @p cows, and an id whose
 * last spec fails to build gets no entry at all even when an earlier spec for it succeeded. The caller still
 * owes the manager side of that contract: every id named by @p objects must be removed from the manager
 * before @p cows is registered, including the ids that produced no wrapper.
 *
 * @param objects The specs to build, in the caller's order
 * @param cows Cleared, then filled with the wrappers in the order the manager must register them
 * @return False if any spec failed to produce a wrapper
 */
bool buildCoalCollisionObjects(const std::vector<CollisionObjectSpec>& objects, std::vector<COW::Ptr>& cows);

/**
 * @brief Apply the collision filter mask based on the current filter group.
 *
 * StaticFilter groups can only collide with KinematicFilter groups.
 * KinematicFilter groups can collide with both StaticFilter and KinematicFilter groups.
 */
void applyCollisionFilterMask(CollisionObjectWrapperBase& cow);

/**
 * @brief Check whether a collision object is filtered as static, i.e. its link is not active.
 *
 * Static is the single value StaticFilter, and the exact complement of isKinematic. Ask through this
 * rather than comparing m_collisionFilterGroup: == StaticFilter and != KinematicFilter agree only for
 * as long as the group holds nothing else.
 */
bool isStatic(const CollisionObjectWrapperBase& cow);

/**
 * @brief Check whether a collision object is filtered as kinematic, i.e. its link is active.
 *
 * Kinematic is every value but StaticFilter, and the exact complement of isStatic. Ask through this
 * rather than comparing m_collisionFilterGroup: != StaticFilter and == KinematicFilter agree only for
 * as long as the group holds nothing else.
 */
bool isKinematic(const CollisionObjectWrapperBase& cow);

/**
 * @brief Set a collision object's contact distance threshold from the current margin data.
 *
 * Thresholding before broadphase registration is what puts the margin-expanded AABB into the tree:
 * the setter grows every collision object's AABB unconditionally, so a wrapper already carrying the
 * current threshold is left untouched.
 * @param cow The collision object wrapper to update
 * @param margin_data The current collision margin data
 * @return True if the threshold changed
 */
bool applyCollisionMarginThreshold(CollisionObjectWrapperBase& cow,
                                   const tesseract::common::CollisionMarginData& margin_data);

/**
 * @brief Update collision objects filters
 * @param active_ids Set of active collision object LinkIds
 * @param cow The collision object to update
 * @param static_manager Broadphase manager for static objects
 * @param dynamic_manager Broadphase manager for dynamic objects
 */
void updateCollisionObjectFilters(const std::unordered_set<tesseract::common::LinkId>& active_ids,
                                  const COW::Ptr& cow,
                                  const std::unique_ptr<coal::BroadPhaseCollisionManager>& static_manager,
                                  const std::unique_ptr<coal::BroadPhaseCollisionManager>& dynamic_manager);

/**
 * @brief Update collision objects filters for continuous collision checking
 *
 * This is the only place a link changes between static and kinematic, and the swap registers the incoming
 * wrapper exactly as it stands. Promoting a link to kinematic therefore also brings its cast wrapper
 * current: the pose the regular wrapper has been carrying, no sweep, and a bumped GJK generation. A new
 * registration site must do the same, or the broadphase receives a link at wherever it used to be.
 *
 * @param active_ids Set of active collision object LinkIds
 * @param cow The regular collision object
 * @param cast_cow The cast collision object
 * @param static_manager Broadphase manager for static objects
 * @param dynamic_manager Broadphase manager for dynamic objects
 */
void updateCollisionObjectFilters(const std::unordered_set<tesseract::common::LinkId>& active_ids,
                                  const COW::Ptr& cow,
                                  CastCOW::Ptr& cast_cow,
                                  const std::unique_ptr<coal::BroadPhaseCollisionManager>& static_manager,
                                  const std::unique_ptr<coal::BroadPhaseCollisionManager>& dynamic_manager);

/**
 * @brief Which shape of a pair of two swept objects is the swept one
 *
 * The smaller shape, and between shapes of one size the shape of the link whose name sorts last;
 * CoalCastBVHManager states why. For shapes of two links, giving the shapes the other way round gives the
 * other answer, so the choice does not follow the order a caller holds the pair in.
 *
 * @param size1 The first shape's size, as CastHullShape::getShapeBoundRadius measures it
 * @param size2 The second shape's size
 * @param link1 The name of the first shape's link
 * @param link2 The name of the second shape's link
 * @return Whether the first shape is the swept one
 */
bool pairSweepsFirst(coal::Scalar size1, coal::Scalar size2, const std::string& link1, const std::string& link2);

/**
 * @brief Create a cast collision object from a regular collision object
 *
 * @param cow The collision object to convert
 * @param build_swept When true, every shape is converted to a sweep-capable form - a convex shape is wrapped
 * in a CastHullShape, an octree is expanded into castable per-voxel boxes - and a shape with neither form
 * throws. When false the wrapper is a plain clone carrying the link's own geometry: a static link is collided
 * through its regular wrapper, so its cast wrapper is a placeholder that nothing reads until
 * updateCollisionObjectFilters builds it on promotion to kinematic. Deferring is what lets a static link hold
 * geometry Coal can collide but not sweep, such as a mesh. A plane throws too when built swept: it has no
 * support function for a CastHullShape to sweep.
 * @param d_arc_compensation Whether the wrapper pads its swept hulls by an arc sagitta. A deferred wrapper
 * keeps it for the wrapper updateCollisionObjectFilters builds in its place.
 * @return A cast collision object, built or deferred
 */
CastCOW::Ptr makeCastCollisionObject(const COW::Ptr& cow, bool build_swept, bool d_arc_compensation);

/**
 * @brief Check whether a cast COW is still deferred, and so must be built before it can carry a sweep.
 *
 * A built wrapper holds CastHullShape (GEOM_CUSTOM) throughout; a deferred one holds the link's own
 * geometry, whatever that is. Node type is therefore the whole test.
 *
 * An empty wrapper reads as built. The deferred placeholder is therefore required to carry the link's
 * own collision objects; a placeholder with none would make promotion skip the build silently.
 */
inline bool castCowNeedsSweptBuild(const CastCollisionObjectWrapper& cast_cow)
{
  for (const auto& co : cast_cow.getCollisionObjects())
    if (co->collisionGeometryPtr()->getNodeType() != coal::GEOM_CUSTOM)
      return true;
  return false;
}

/**
 * @brief This is used to check if a collision check is required between the provided two collision objects
 * @param cd1 The first collision object
 * @param cd2 The second collision object
 * @param pair The link ids of the two collision objects
 * @param validator  The contact allowed validator
 * @return True if the two collision objects should be checked for collision, otherwise false
 */
bool needsCollisionCheck(const CollisionObjectWrapperBase* cd1,
                         const CollisionObjectWrapperBase* cd2,
                         const tesseract::common::LinkIdPair& pair,
                         const std::shared_ptr<const tesseract::common::ContactAllowedValidator>& validator);

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

  /** @brief Reused across candidate pairs, so the per-pair lookups are allocation-free */
  tesseract::common::LinkIdPair link_pair;

  bool collide(coal::CollisionObject* o1, coal::CollisionObject* o2) override;
  virtual ~CollisionCallback() = default;
};

#pragma GCC diagnostic pop

}  // namespace tesseract::collision::tesseract_collision_coal
#endif  // TESSERACT_COLLISION_COAL_UTILS_H
