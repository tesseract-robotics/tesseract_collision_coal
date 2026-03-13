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
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <boost/functional/hash.hpp>
#include <console_bridge/console.h>
#include <coal/broadphase/broadphase_collision_manager.h>
#include <coal/collision.h>
#include <coal/octree.h>
#include <coal/shape/geometric_shapes.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/types.h>
#include <tesseract/collision/common.h>

#include <tesseract/collision/coal/coal_collision_object_wrapper.h>
#include <tesseract/collision/coal/coal_casthullshape.h>

namespace tesseract::collision::tesseract_collision_coal
{
using CollisionGeometryPtr = std::shared_ptr<coal::CollisionGeometry>;
using CollisionObjectPtr = std::shared_ptr<CoalCollisionObjectWrapper>;
using CollisionObjectRawPtr = coal::CollisionObject*;
using CollisionObjectConstPtr = std::shared_ptr<const coal::CollisionObject>;

/** @brief A pair of collision objects used as map key */
using CollisionObjectPair = std::pair<coal::CollisionObject*, coal::CollisionObject*>;

/** @brief Hash functor for CollisionObjectPair */
struct CollisionObjectPairHash
{
  std::size_t operator()(const CollisionObjectPair& p) const noexcept { return boost::hash_value(p); }
};

/** @brief Cache mapping collision object pairs to their precomputed collision functor and request */
using CollisionCacheMap = std::unordered_map<CollisionObjectPair,
                                             std::pair<coal::ComputeCollision, coal::CollisionRequest>,
                                             CollisionObjectPairHash>;

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
  CollisionObjectWrapper(std::string name,
                         const int& type_id,
                         CollisionShapesConst shapes,
                         tesseract::common::VectorIsometry3d shape_poses);

  short int m_collisionFilterGroup{ CollisionFilterGroups::KinematicFilter };
  short int m_collisionFilterMask{ CollisionFilterGroups::StaticFilter | CollisionFilterGroups::KinematicFilter };
  bool m_enabled{ true };

  const std::string& getName() const { return name_; }
  const int& getTypeID() const { return type_id_; }
  /** \brief Check if two objects point to the same source object */
  bool sameObject(const CollisionObjectWrapper& other) const
  {
    return name_ == other.name_ && type_id_ == other.type_id_ && shapes_.size() == other.shapes_.size() &&
           shape_poses_.size() == other.shape_poses_.size() &&
           std::equal(shapes_.begin(), shapes_.end(), other.shapes_.begin()) &&
           std::equal(shape_poses_.begin(),
                      shape_poses_.end(),
                      other.shape_poses_.begin(),
                      [](const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2) { return t1.isApprox(t2); });
  }

  const CollisionShapesConst& getCollisionGeometries() const { return shapes_; }

  CollisionShapesConst& getCollisionGeometries() { return shapes_; }

  const tesseract::common::VectorIsometry3d& getCollisionGeometriesTransforms() const { return shape_poses_; }

  tesseract::common::VectorIsometry3d& getCollisionGeometriesTransforms() { return shape_poses_; }

  void setCollisionObjectsTransform(const Eigen::Isometry3d& pose)
  {
    world_pose_ = pose;
    for (auto& co : collision_objects_)
    {
      auto tf = pose * shape_poses_[static_cast<std::size_t>(co->getShapeIndex())];
      co->setTransform(coal::Transform3s(tf.rotation(), tf.translation()));
      co->updateAABB();  // This a tesseract function that updates the aabb to take into account contact distance
    }
  }

  void setContactDistanceThreshold(double contact_distance)
  {
    contact_distance_ = contact_distance;
    for (auto& co : collision_objects_)
      co->setContactDistanceThreshold(contact_distance_);
  }

  double getContactDistanceThreshold() const { return contact_distance_; }

  const Eigen::Isometry3d& getCollisionObjectsTransform() const { return world_pose_; }

  const std::vector<CollisionObjectPtr>& getCollisionObjects() const { return collision_objects_; }

  std::vector<CollisionObjectPtr>& getCollisionObjects() { return collision_objects_; }

  const std::vector<CollisionObjectRawPtr>& getCollisionObjectsRaw() const { return collision_objects_raw_; }

  std::vector<CollisionObjectRawPtr>& getCollisionObjectsRaw() { return collision_objects_raw_; }

  std::shared_ptr<CollisionObjectWrapper> clone() const
  {
    auto clone_cow = std::make_shared<CollisionObjectWrapper>();
    clone_cow->name_ = name_;
    clone_cow->type_id_ = type_id_;
    clone_cow->shapes_ = shapes_;
    clone_cow->shape_poses_ = shape_poses_;
    clone_cow->collision_geometries_ = collision_geometries_;

    clone_cow->collision_objects_.reserve(collision_objects_.size());
    clone_cow->collision_objects_raw_.reserve(collision_objects_.size());
    for (const auto& co : collision_objects_)
    {
      assert(std::dynamic_pointer_cast<CoalCollisionObjectWrapper>(co) != nullptr);
      auto collObj =
          std::make_shared<CoalCollisionObjectWrapper>(*std::static_pointer_cast<CoalCollisionObjectWrapper>(co));
      collObj->setUserData(clone_cow.get());
      collObj->setTransform(co->getTransform());
      collObj->updateAABB();
      clone_cow->collision_objects_.push_back(collObj);
      clone_cow->collision_objects_raw_.push_back(collObj.get());
    }

    clone_cow->m_collisionFilterGroup = m_collisionFilterGroup;
    clone_cow->m_collisionFilterMask = m_collisionFilterMask;
    clone_cow->m_enabled = m_enabled;
    return clone_cow;
  }

  /**
   * @brief Given Coal collision shape get the index to the links collision shape
   * @param co Coal collision shape
   * @return links collision shape index
   */
  static int getShapeIndex(const coal::CollisionObject* co);

protected:
  std::string name_;                                              // name of the collision object
  int type_id_{ -1 };                                             // user defined type id
  Eigen::Isometry3d world_pose_{ Eigen::Isometry3d::Identity() }; /**< @brief Collision Object World Transformation */
  CollisionShapesConst shapes_;
  tesseract::common::VectorIsometry3d shape_poses_;
  std::vector<CollisionGeometryPtr> collision_geometries_;
  std::vector<CollisionObjectPtr> collision_objects_;
  /**
   * @brief The raw pointer is also stored because Coal accepts vectors for batch process.
   * Note: They are updating the API to Shared Pointers but the broadphase has not been updated yet.
   */
  std::vector<CollisionObjectRawPtr> collision_objects_raw_;

  double contact_distance_{ 0 }; /**< @brief The contact distance threshold */
};

CollisionGeometryPtr createShapePrimitive(const CollisionShapeConstPtr& geom);

using COW = CollisionObjectWrapper;
using Link2COW = std::map<std::string, COW::Ptr>;
using Link2ConstCOW = std::map<std::string, COW::ConstPtr>;

inline COW::Ptr createCoalCollisionObject(const std::string& name,
                                          const int& type_id,
                                          const CollisionShapesConst& shapes,
                                          const tesseract::common::VectorIsometry3d& shape_poses,
                                          bool enabled)
{
  // dont add object that does not have geometry
  if (shapes.empty() || shape_poses.empty() || (shapes.size() != shape_poses.size()))
  {
    CONSOLE_BRIDGE_logDebug("ignoring link %s", name.c_str());
    return nullptr;
  }

  auto new_cow = std::make_shared<COW>(name, type_id, shapes, shape_poses);

  new_cow->m_enabled = enabled;
  // CONSOLE_BRIDGE_logDebug("Created collision object for link %s", new_cow->getName().c_str());
  return new_cow;
}

/**
 * @brief Update collision objects filters
 * @param active The active collision objects
 * @param cow The collision object to update
 * @param static_manager Broadphase manager for static objects
 * @param dynamic_manager Broadphase manager for dynamic objects
 */
inline void updateCollisionObjectFilters(const std::vector<std::string>& active,
                                         const COW::Ptr& cow,
                                         const std::unique_ptr<coal::BroadPhaseCollisionManager>& static_manager,
                                         const std::unique_ptr<coal::BroadPhaseCollisionManager>& dynamic_manager)
{
  // For discrete checks we can check static to kinematic and kinematic to
  // kinematic
  if (!isLinkActive(active, cow->getName()))
  {
    if (cow->m_collisionFilterGroup != CollisionFilterGroups::StaticFilter)
    {
      const std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
      // This link was dynamic but is now static
      for (const auto& co : objects)
        dynamic_manager->unregisterObject(co.get());

      for (const auto& co : objects)
        static_manager->registerObject(co.get());
    }
    cow->m_collisionFilterGroup = CollisionFilterGroups::StaticFilter;
  }
  else
  {
    if (cow->m_collisionFilterGroup != CollisionFilterGroups::KinematicFilter)
    {
      const std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
      // This link was static but is now dynamic
      for (const auto& co : objects)
        static_manager->unregisterObject(co.get());

      for (const auto& co : objects)
        dynamic_manager->registerObject(co.get());
    }
    cow->m_collisionFilterGroup = CollisionFilterGroups::KinematicFilter;
  }

  // If the group is StaticFilter then the Mask is KinematicFilter, then StaticFilter groups can only collide with
  // KinematicFilter groups. If the group is KinematicFilter then the mask is StaticFilter and KinematicFilter meaning
  // that KinematicFilter groups can collide with both StaticFilter and KinematicFilter groups.
  if (cow->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
  {
    cow->m_collisionFilterMask = CollisionFilterGroups::KinematicFilter;
  }
  else
  {
    cow->m_collisionFilterMask = CollisionFilterGroups::StaticFilter | CollisionFilterGroups::KinematicFilter;
  }
}

/**
 * @brief Update collision objects filters for continuous collision checking
 * @param active The active collision objects
 * @param cow The regular collision object
 * @param cast_cow The cast collision object
 * @param static_manager Broadphase manager for static objects
 * @param dynamic_manager Broadphase manager for dynamic objects
 */
inline void updateCollisionObjectFilters(const std::vector<std::string>& active,
                                         const COW::Ptr& cow,
                                         const COW::Ptr& cast_cow,
                                         const std::unique_ptr<coal::BroadPhaseCollisionManager>& static_manager,
                                         const std::unique_ptr<coal::BroadPhaseCollisionManager>& dynamic_manager)
{
  const std::vector<CollisionObjectPtr>& reg_objects = cow->getCollisionObjects();
  const std::vector<CollisionObjectPtr>& cast_objects = cast_cow->getCollisionObjects();
  const bool regular_has_non_shape_base =
      std::any_of(reg_objects.begin(), reg_objects.end(), [](const CollisionObjectPtr& co) {
        return (dynamic_cast<const coal::ShapeBase*>(co->collisionGeometry().get()) == nullptr);
      });
  // For inactive objects, use static manager; for non-ShapeBase geometry
  // (e.g., octree) keep cast representation to stay on a supported path.
  if (!isLinkActive(active, cow->getName()))
  {
    if (cow->m_collisionFilterGroup != CollisionFilterGroups::StaticFilter)
    {
      // This link was dynamic but is now static
      for (const auto& co : cast_objects)
      {
        dynamic_manager->unregisterObject(co.get());
      }
      // Use cast representation for static objects that cannot be represented
      // as ShapeBase (e.g., octree), so narrowphase never sees CastHull-vs-OcTree.
      const auto& static_objects = regular_has_non_shape_base ? cast_objects : reg_objects;
      if (regular_has_non_shape_base)
      {
        coal::Transform3s identity_tf;
        identity_tf.setIdentity();

        // Clear any stale swept extent before reusing cast-backed objects as
        // static proxies. Without this, a previously active octree can remain
        // registered with a swept CastHullShape volume after demotion.
        for (const auto& co : cast_objects)
        {
          auto* cast_shape = dynamic_cast<CastHullShape*>(co->collisionGeometry().get());
          if (cast_shape != nullptr)
          {
            cast_shape->updateCastTransform(identity_tf);
            co->updateAABB();
          }
        }
      }
      for (const auto& co : static_objects)
      {
        static_manager->registerObject(co.get());
      }
    }
    cow->m_collisionFilterGroup = CollisionFilterGroups::StaticFilter;
    cast_cow->m_collisionFilterGroup = CollisionFilterGroups::StaticFilter;
  }
  // For active objects, we want the cast version in the dynamic manager
  else
  {
    if (cow->m_collisionFilterGroup != CollisionFilterGroups::KinematicFilter)
    {
      // This link was static but is now dynamic.
      // For non-ShapeBase geometries (e.g., octree) the cast representation was
      // registered in the static manager, so unregister those objects.
      const auto& static_objects = regular_has_non_shape_base ? cast_objects : reg_objects;
      for (const auto& co : static_objects)
      {
        static_manager->unregisterObject(co.get());
      }
      for (const auto& co : cast_objects)
      {
        dynamic_manager->registerObject(co.get());
      }
    }
    cow->m_collisionFilterGroup = CollisionFilterGroups::KinematicFilter;
    cast_cow->m_collisionFilterGroup = CollisionFilterGroups::KinematicFilter;
  }

  // If the group is StaticFilter then the Mask is KinematicFilter, then StaticFilter groups can only collide with
  // KinematicFilter groups. If the group is KinematicFilter then the mask is StaticFilter and KinematicFilter meaning
  // that KinematicFilter groups can collide with both StaticFilter and KinematicFilter groups.
  if (cow->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
  {
    cow->m_collisionFilterMask = CollisionFilterGroups::KinematicFilter;
  }
  else
  {
    cow->m_collisionFilterMask = CollisionFilterGroups::StaticFilter | CollisionFilterGroups::KinematicFilter;
  }
  if (cast_cow->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
  {
    cast_cow->m_collisionFilterMask = CollisionFilterGroups::KinematicFilter;
  }
  else
  {
    cast_cow->m_collisionFilterMask = CollisionFilterGroups::StaticFilter | CollisionFilterGroups::KinematicFilter;
  }
}

/**
 * @brief Create a cast collision object from a regular collision object
 * @param cow The collision object to convert
 * @return A new collision object with shapes converted to CastHullShapes
 */
inline COW::Ptr makeCastCollisionObject(const COW::Ptr& cow)
{
  auto cast_cow = cow->clone();

  // Create the vector of new collision objects
  std::vector<CollisionObjectPtr> new_collision_objects;
  std::vector<CollisionObjectRawPtr> new_collision_objects_raw;

  // Identity transform for initial state
  coal::Transform3s identity_tf;
  identity_tf.setIdentity();

  const auto link_tf = cast_cow->getCollisionObjectsTransform();
  const auto current_shapes = cast_cow->getCollisionGeometries();
  const auto current_shape_poses = cast_cow->getCollisionGeometriesTransforms();

  const auto& current_collision_objects = cast_cow->getCollisionObjects();
  new_collision_objects.reserve(current_collision_objects.size());
  new_collision_objects_raw.reserve(current_collision_objects.size());

  CollisionShapesConst new_shapes;
  tesseract::common::VectorIsometry3d new_shape_poses;
  new_shapes.reserve(current_shapes.size());
  new_shape_poses.reserve(current_shape_poses.size());

  for (const auto& co : current_collision_objects)
  {
    const auto old_shape_index = static_cast<std::size_t>(co->getShapeIndex());
    assert(old_shape_index < current_shapes.size());
    assert(old_shape_index < current_shape_poses.size());

    auto geo = co->collisionGeometry();
    auto* shape_base_ptr = dynamic_cast<coal::ShapeBase*>(geo.get());
    if (shape_base_ptr != nullptr)
    {
      // Create a cast hull shape from the original shape
      auto cast_shape = std::make_shared<CastHullShape>(std::static_pointer_cast<coal::ShapeBase>(geo), identity_tf);

      // Create a new collision object with the cast shape
      auto cast_co = std::make_shared<CoalCollisionObjectWrapper>(cast_shape, co->getTransform());
      cast_co->setShapeIndex(static_cast<int>(new_shape_poses.size()));
      cast_co->setSourceShapeIndex(static_cast<int>(old_shape_index));
      cast_co->setSourceSubshapeIndex(co->getSourceSubshapeIndex());
      cast_co->setContactDistanceThreshold(co->getContactDistanceThreshold());
      cast_co->setUserData(cast_cow.get());

      // Store everything
      new_collision_objects.push_back(cast_co);
      new_collision_objects_raw.push_back(cast_co.get());
      new_shapes.push_back(current_shapes[old_shape_index]);
      new_shape_poses.push_back(current_shape_poses[old_shape_index]);
    }
    else
    {
      if (auto octree_geo = std::dynamic_pointer_cast<coal::OcTree>(geo); octree_geo != nullptr)
      {
        // Expand occupied octree cells into castable box sub-shapes.
        const auto tree = octree_geo->getTree();
        assert(tree != nullptr);
        const auto& base_shape_pose = current_shape_poses[old_shape_index];
        int octree_subshape_index = 0;

        // Reuse one box shape per tree depth level (all voxels at the same depth
        // have the same size), matching Bullet's managed_shapes pattern.
        std::vector<std::shared_ptr<coal::Box>> managed_boxes(tree->getTreeDepth() + 1);

        for (octomap::OcTree::iterator it = tree->begin(static_cast<unsigned char>(tree->getTreeDepth())),
                                       end = tree->end();
             it != end;
             ++it)
        {
          if (!octree_geo->isNodeOccupied(&(*it)))
            continue;

          auto& box_shape = managed_boxes.at(it.getDepth());
          if (box_shape == nullptr)
          {
            const double size = it.getSize();
            box_shape = std::make_shared<coal::Box>(size, size, size);
          }
          auto cast_shape = std::make_shared<CastHullShape>(box_shape, identity_tf);

          Eigen::Isometry3d voxel_pose = Eigen::Isometry3d::Identity();
          voxel_pose.translation() = Eigen::Vector3d(it.getX(), it.getY(), it.getZ());

          const Eigen::Isometry3d shape_pose = base_shape_pose * voxel_pose;
          const Eigen::Isometry3d world_pose = link_tf * shape_pose;

          auto cast_co = std::make_shared<CoalCollisionObjectWrapper>(
              cast_shape, coal::Transform3s(world_pose.rotation(), world_pose.translation()));
          cast_co->setShapeIndex(static_cast<int>(new_shape_poses.size()));
          cast_co->setSourceShapeIndex(static_cast<int>(old_shape_index));
          cast_co->setSourceSubshapeIndex(octree_subshape_index++);
          cast_co->setContactDistanceThreshold(co->getContactDistanceThreshold());
          cast_co->setUserData(cast_cow.get());

          new_collision_objects.push_back(cast_co);
          new_collision_objects_raw.push_back(cast_co.get());
          new_shapes.push_back(current_shapes[old_shape_index]);
          new_shape_poses.push_back(shape_pose);
        }
      }
      else
      {
        throw std::runtime_error("I can only continuous collision check convex shapes, compound shapes made of "
                                 "convex shapes, and octree boxes");
      }
    }
  }

  // Replace the collision objects in the cast_cow with the cast versions
  cast_cow->getCollisionGeometries() = new_shapes;
  cast_cow->getCollisionGeometriesTransforms() = new_shape_poses;
  cast_cow->getCollisionObjects() = new_collision_objects;
  cast_cow->getCollisionObjectsRaw() = new_collision_objects_raw;

  return cast_cow;
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
  ContactTestDataWrapper(CollisionMarginData collision_margin_data,
                         std::shared_ptr<const tesseract::common::ContactAllowedValidator> validator,
                         ContactRequest req,
                         ContactResultMap& res,
                         CollisionCacheMap& collision_cache)
    : collision_cache(&collision_cache)
  {
    this->collision_margin_data = std::move(collision_margin_data);
    this->validator = std::move(validator);
    this->req = std::move(req);
    this->res = &res;
  }

  CollisionCacheMap* collision_cache;
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
