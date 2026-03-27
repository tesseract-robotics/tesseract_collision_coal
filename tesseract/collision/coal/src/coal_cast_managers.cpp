/**
 * @file coal_cast_managers.cpp
 * @brief Tesseract Coal contact checker implementation.
 *
 * @author Roelof Oomen
 * @date Aug 04, 2025
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/geometry.h>
#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_collision_geometry_cache.h>
#include <tesseract/collision/coal/coal_casthullshape.h>
#include <tesseract/collision/coal/coal_utils.h>

namespace tesseract::collision::tesseract_collision_coal
{
static const CollisionShapesConst EMPTY_COLLISION_SHAPES_CONST;
static const tesseract::common::VectorIsometry3d EMPTY_COLLISION_SHAPES_TRANSFORMS;

/// Insert all raw collision object pointers from a COW into the set.
static void insertCowPointers(CollisionObjectPtrSet& ptrs, const COW::Ptr& cow)
{
  for (const auto& co : cow->getCollisionObjects())
    ptrs.insert(co.get());
}

CoalCastBVHManager::CoalCastBVHManager(std::string name) : name_(std::move(name))
{
  static_manager_ = std::make_unique<coal::DynamicAABBTreeCollisionManager>();
  dynamic_manager_ = std::make_unique<coal::DynamicAABBTreeCollisionManager>();
  collision_margin_data_ = CollisionMarginData(0);
}

std::string CoalCastBVHManager::getName() const { return name_; }

ContinuousContactManager::UPtr CoalCastBVHManager::clone() const
{
  CoalCollisionGeometryCache::prune();

  // Note: addCollisionObject creates fresh CastHullShapes with identity cast
  // transforms, so any active sweep state (set via setCollisionObjectsTransform
  // with pose1/pose2) is not preserved. This matches Bullet's clone behavior.
  auto manager = std::make_unique<CoalCastBVHManager>(name_);

  for (const auto& cow : link2cow_)
  {
    COW::Ptr new_cow = cow.second->clone();
    new_cow->setCollisionObjectsTransform(cow.second->getCollisionObjectsTransform());
    new_cow->setContactDistanceThreshold(collision_margin_data_.getMaxCollisionMargin(new_cow->getName()));
    manager->addCollisionObject(new_cow);
  }

  manager->setActiveCollisionObjects(active_);
  manager->setCollisionMarginData(collision_margin_data_);
  manager->setContactAllowedValidator(validator_);

  return manager;
}

bool CoalCastBVHManager::addCollisionObject(const std::string& name,
                                            const int& mask_id,
                                            const CollisionShapesConst& shapes,
                                            const tesseract::common::VectorIsometry3d& shape_poses,
                                            bool enabled)
{
  if (link2cow_.find(name) != link2cow_.end())
    removeCollisionObject(name);

  const COW::Ptr new_cow = createCoalCollisionObject(name, mask_id, shapes, shape_poses, enabled);
  if (new_cow != nullptr)
  {
    addCollisionObject(new_cow);
    return true;
  }

  return false;
}

const CollisionShapesConst& CoalCastBVHManager::getCollisionObjectGeometries(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (cow != link2cow_.end()) ? cow->second->getCollisionGeometries() : EMPTY_COLLISION_SHAPES_CONST;
}

const tesseract::common::VectorIsometry3d&
CoalCastBVHManager::getCollisionObjectGeometriesTransforms(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (cow != link2cow_.end()) ? cow->second->getCollisionGeometriesTransforms() : EMPTY_COLLISION_SHAPES_TRANSFORMS;
}

bool CoalCastBVHManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool CoalCastBVHManager::removeCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    auto it_obj = std::find(collision_objects_.begin(), collision_objects_.end(), name);
    if (it_obj != collision_objects_.end())
      collision_objects_.erase(it_obj);
    const std::vector<CollisionObjectPtr>& objects = it->second->getCollisionObjects();
    coal_co_count_ -= objects.size();
    if (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
      removeObjects(objects, *static_manager_);
    link2cow_.erase(name);

    auto it_active = std::find(active_.begin(), active_.end(), name);
    if (it_active != active_.end())
      active_.erase(it_active);

    // Also remove from cast map
    auto it_cast = link2castcow_.find(name);
    if (it_cast != link2castcow_.end())
    {
      const std::vector<CollisionObjectPtr>& objects_cast = it_cast->second->getCollisionObjects();
      if (it_cast->second->m_collisionFilterGroup == CollisionFilterGroups::KinematicFilter)
        removeObjects(objects_cast, *dynamic_manager_);
      link2castcow_.erase(name);
    }

    return true;
  }

  return false;
}

void CoalCastBVHManager::removeObjects(const std::vector<CollisionObjectPtr>& objects,
                                       coal::BroadPhaseCollisionManager& manager)
{
  for (const auto& co : objects)
    manager.unregisterObject(co.get());
  invalidateCacheFor(collision_cache, objects);
}

bool CoalCastBVHManager::enableCollisionObject(const std::string& name)
{
  return setCollisionObjectEnabled(name, true);
}

bool CoalCastBVHManager::disableCollisionObject(const std::string& name)
{
  return setCollisionObjectEnabled(name, false);
}

bool CoalCastBVHManager::setCollisionObjectEnabled(const std::string& name, bool enabled)
{
  auto it = link2cow_.find(name);
  if (it == link2cow_.end())
    return false;

  it->second->m_enabled = enabled;

  auto cast_it = link2castcow_.find(name);
  if (cast_it != link2castcow_.end())
  {
    cast_it->second->m_enabled = enabled;
    invalidateCachedGJKGuessFor(
        collision_cache, it->second->getCollisionObjects(), cast_it->second->getCollisionObjects());
  }
  else
  {
    invalidateCachedGJKGuessFor(collision_cache, it->second->getCollisionObjects());
  }

  return true;
}

bool CoalCastBVHManager::isCollisionObjectEnabled(const std::string& name) const
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
    return it->second->m_enabled;

  return false;
}

void CoalCastBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    static_update_.clear();
    dynamic_update_.clear();
    collectTransformUpdate(it, pose);
    flushBatchUpdate();
  }
}

void CoalCastBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                      const tesseract::common::VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  static_update_.clear();
  dynamic_update_.clear();
  for (auto i = 0U; i < names.size(); ++i)
  {
    auto it = link2cow_.find(names[i]);
    if (it != link2cow_.end())
      collectTransformUpdate(it, poses[i]);
  }
  flushBatchUpdate();
}

void CoalCastBVHManager::setCollisionObjectsTransform(const tesseract::common::TransformMap& transforms)
{
  static_update_.clear();
  dynamic_update_.clear();
  for (const auto& transform : transforms)
  {
    auto it = link2cow_.find(transform.first);
    if (it != link2cow_.end())
      collectTransformUpdate(it, transform.second);
  }
  flushBatchUpdate();
}

void CoalCastBVHManager::setCollisionObjectsTransform(const std::string& name,
                                                      const Eigen::Isometry3d& pose1,
                                                      const Eigen::Isometry3d& pose2)
{
  auto cast_it = link2castcow_.find(name);
  if (cast_it != link2castcow_.end())
  {
    static_update_.clear();
    dynamic_update_.clear();
    auto reg_it = link2cow_.find(name);
    collectCastTransformUpdate(cast_it, reg_it, pose1, pose2);
    // Mark GJK warm-start hints invalid for affected object pairs. The guess is
    // re-seeded lazily in collide() where actual transforms are available.
    if (reg_it != link2cow_.end())
      invalidateCachedGJKGuessFor(
          collision_cache, cast_it->second->getCollisionObjects(), reg_it->second->getCollisionObjects());
    else
      invalidateCachedGJKGuessFor(collision_cache, cast_it->second->getCollisionObjects());
    flushBatchUpdate();
  }
}

void CoalCastBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                      const tesseract::common::VectorIsometry3d& pose1,
                                                      const tesseract::common::VectorIsometry3d& pose2)
{
  assert(names.size() == pose1.size());
  assert(names.size() == pose2.size());
  static_update_.clear();
  dynamic_update_.clear();
  CollisionObjectPtrSet updated_ptrs;
  updated_ptrs.reserve(coal_co_count_);
  for (auto i = 0U; i < names.size(); ++i)
  {
    auto cast_it = link2castcow_.find(names[i]);
    if (cast_it != link2castcow_.end())
    {
      auto reg_it = link2cow_.find(names[i]);
      collectCastTransformUpdate(cast_it, reg_it, pose1[i], pose2[i]);
      insertCowPointers(updated_ptrs, cast_it->second);
      if (reg_it != link2cow_.end())
        insertCowPointers(updated_ptrs, reg_it->second);
    }
  }
  invalidateCachedGJKGuessFor(collision_cache, updated_ptrs);
  flushBatchUpdate();
}

void CoalCastBVHManager::setCollisionObjectsTransform(const tesseract::common::TransformMap& pose1,
                                                      const tesseract::common::TransformMap& pose2)
{
  assert(pose1.size() == pose2.size());
  static_update_.clear();
  dynamic_update_.clear();
  CollisionObjectPtrSet updated_ptrs;
  updated_ptrs.reserve(coal_co_count_);
  for (const auto& [name, tf1] : pose1)
  {
    auto cast_it = link2castcow_.find(name);
    if (cast_it != link2castcow_.end())
    {
      auto it2 = pose2.find(name);
      assert(it2 != pose2.end());
      auto reg_it = link2cow_.find(name);
      collectCastTransformUpdate(cast_it, reg_it, tf1, it2->second);
      insertCowPointers(updated_ptrs, cast_it->second);
      if (reg_it != link2cow_.end())
        insertCowPointers(updated_ptrs, reg_it->second);
    }
  }
  invalidateCachedGJKGuessFor(collision_cache, updated_ptrs);
  flushBatchUpdate();
}

const std::vector<std::string>& CoalCastBVHManager::getCollisionObjects() const { return collision_objects_; }

void CoalCastBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;

  for (auto& co : link2cow_)
  {
    COW::Ptr& cow = co.second;
    // Get the cast collision object
    COW::Ptr& cast_cow = link2castcow_[cow->getName()];

    // Use the specialized function that properly handles both regular and cast objects
    updateCollisionObjectFilters(active_, cow, cast_cow, static_manager_, dynamic_manager_);
  }

  // This causes a refit on the bvh tree.
  dynamic_manager_->update();
  static_manager_->update();
}

const std::vector<std::string>& CoalCastBVHManager::getActiveCollisionObjects() const { return active_; }

void CoalCastBVHManager::setCollisionMarginData(CollisionMarginData collision_margin_data)
{
  collision_margin_data_ = std::move(collision_margin_data);
  onCollisionMarginDataChanged();
}

const CollisionMarginData& CoalCastBVHManager::getCollisionMarginData() const { return collision_margin_data_; }

void CoalCastBVHManager::setCollisionMarginPairData(const CollisionMarginPairData& pair_margin_data,
                                                    CollisionMarginPairOverrideType override_type)
{
  collision_margin_data_.apply(pair_margin_data, override_type);
  onCollisionMarginDataChanged();
}

void CoalCastBVHManager::setDefaultCollisionMargin(double default_collision_margin)
{
  collision_margin_data_.setDefaultCollisionMargin(default_collision_margin);
  onCollisionMarginDataChanged();
}

void CoalCastBVHManager::setCollisionMarginPair(const std::string& name1,
                                                const std::string& name2,
                                                double collision_margin)
{
  collision_margin_data_.setCollisionMargin(name1, name2, collision_margin);
  onCollisionMarginDataChanged();
}

void CoalCastBVHManager::incrementCollisionMargin(double increment)
{
  collision_margin_data_.incrementMargins(increment);
  onCollisionMarginDataChanged();
}

void CoalCastBVHManager::setContactAllowedValidator(
    std::shared_ptr<const tesseract::common::ContactAllowedValidator> validator)
{
  validator_ = std::move(validator);
}

std::shared_ptr<const tesseract::common::ContactAllowedValidator> CoalCastBVHManager::getContactAllowedValidator() const
{
  return validator_;
}

void CoalCastBVHManager::contactTest(ContactResultMap& collisions, const ContactRequest& request)
{
  ContactTestDataWrapper cdata(collision_margin_data_, validator_, request, collisions, collision_cache);

  CollisionCallback collisionCallback;
  collisionCallback.cdata = &cdata;

  // Check static-vs-dynamic first (typically the larger pair set), then
  // dynamic-vs-dynamic (self-check). Order is not significant for correctness
  // but checking static-vs-dynamic first allows early exit via FIRST mode
  // before the self-check.
  if (!static_manager_->empty())
    static_manager_->collide(dynamic_manager_.get(), &collisionCallback);

  if (!cdata.done && !dynamic_manager_->empty())
    dynamic_manager_->collide(&collisionCallback);
}

void CoalCastBVHManager::addCollisionObject(const COW::Ptr& cow)
{
  const std::size_t cnt = cow->getCollisionObjects().size();
  coal_co_count_ += cnt;
  static_update_.reserve(coal_co_count_);
  dynamic_update_.reserve(coal_co_count_);
  link2cow_[cow->getName()] = cow;
  collision_objects_.push_back(cow->getName());

  // Create cast collision object
  COW::Ptr cast_cow = makeCastCollisionObject(cow);

  // Add it to the cast map
  link2castcow_[cast_cow->getName()] = cast_cow;

  if (cow->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
  {
    const std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
    for (const auto& co : objects)
      static_manager_->registerObject(co.get());
  }
  else
  {
    const std::vector<CollisionObjectPtr>& cast_objects = cast_cow->getCollisionObjects();
    for (const auto& co : cast_objects)
      dynamic_manager_->registerObject(co.get());
  }

  /// If active links is not empty update filters to replace the active links list
  if (!active_.empty())
    updateCollisionObjectFilters(active_, cow, cast_cow, static_manager_, dynamic_manager_);

  // This causes a refit on the bvh tree.
  dynamic_manager_->update();
  static_manager_->update();

  // Pre-reserve collision cache to avoid rehashing during contactTest.
  // Accounts for static-vs-dynamic and dynamic-vs-dynamic (self-check) pairs.
  const auto& n_static = static_manager_->size();
  const auto& n_dynamic = dynamic_manager_->size();
  collision_cache.reserve((n_static * n_dynamic) + (n_dynamic * (n_dynamic - 1) / 2));
}

void CoalCastBVHManager::collectTransformUpdate(Link2COW::iterator it, const Eigen::Isometry3d& pose)
{
  const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
  if (!transformChanged(cur_tf, pose))
    return;

  it->second->setCollisionObjectsTransform(pose);

  // Also update the cast version if it exists
  auto cast_it = link2castcow_.find(it->first);
  if (cast_it != link2castcow_.end())
  {
    cast_it->second->setCollisionObjectsTransform(pose);

    if (it->second->m_collisionFilterGroup == CollisionFilterGroups::KinematicFilter)
    {
      // For kinematic objects, only the cast COW is in the broadphase — the regular
      // COW is not registered in any manager, so skip its broadphase update.
      cast_it->second->appendCollisionObjectsRaw(dynamic_update_);
      return;
    }
  }

  auto& update_vec =
      (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter) ? static_update_ : dynamic_update_;
  it->second->appendCollisionObjectsRaw(update_vec);
}

void CoalCastBVHManager::collectCastTransformUpdate(Link2COW::iterator cast_it,
                                                    Link2COW::iterator reg_it,
                                                    const Eigen::Isometry3d& pose1,
                                                    const Eigen::Isometry3d& pose2)
{
  COW::Ptr& cow = cast_it->second;

  // Keep regular object aligned at the start transform
  if (reg_it != link2cow_.end())
    reg_it->second->setCollisionObjectsTransform(pose1);

  // Match Bullet behavior: do not update cast sweep state/AABB for disabled objects.
  // Still sync the cast COW's transform so it's correct when re-enabled.
  if (!cow->m_enabled)
  {
    cow->setCollisionObjectsTransform(pose1);
    return;
  }

  // Convert to coal::Transform3s format
  const auto tf1 = coal::Transform3s(pose1.rotation(), pose1.translation());
  const auto tf2 = coal::Transform3s(pose2.rotation(), pose2.translation());

  const auto& shape_poses = cow->getCollisionGeometriesTransforms();

  // Update cast transforms first so computeLocalAABB reflects the swept volume.
  // All objects in link2castcow_ are CastHullShape-wrapped (by makeCastCollisionObject).
  for (auto& co : cow->getCollisionObjects())
  {
    auto* cast_shape = static_cast<CastHullShape*>(co->collisionGeometry().get());
    assert(cast_shape != nullptr);
    // Compute per-shape relative transform accounting for local offset.
    // Each shape's world transform is link_tf * local_tf, so the relative
    // motion in the shape's local frame is:
    //   (tf1 * local_tf)^-1 * (tf2 * local_tf)
    // This matches Bullet's compound shape handling where each child gets
    // its own delta_tf = (tf1 * local_tf).inverseTimes(tf2 * local_tf).
    const auto& shape_pose = shape_poses[static_cast<std::size_t>(co->getShapeIndex())];
    const auto local_tf = coal::Transform3s(shape_pose.rotation(), shape_pose.translation());
    const auto shape_tf1 = tf1 * local_tf;
    const auto shape_tf2 = tf2 * local_tf;
    cast_shape->updateCastTransform(shape_tf1.inverseTimes(shape_tf2));
  }

  // Re-apply world transform so CoalCollisionObjectWrapper::updateAABB uses the
  // updated CastHullShape local AABB (swept volume).
  cow->setCollisionObjectsTransform(pose1);

  // Note: GJK cache invalidation is NOT done here — callers are responsible for
  // invalidating after all links are updated, to avoid O(N*M) repeated cache scans.

  // Append to broadphase update vectors (flushed by caller)
  auto& update_vec =
      (cow->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter) ? static_update_ : dynamic_update_;
  cow->appendCollisionObjectsRaw(update_vec);
}

void CoalCastBVHManager::flushBatchUpdate()
{
  if (!static_update_.empty())
    static_manager_->update(static_update_);

  if (!dynamic_update_.empty())
    dynamic_manager_->update(dynamic_update_);
}

void CoalCastBVHManager::onCollisionMarginDataChanged()
{
  static_update_.clear();
  dynamic_update_.clear();

  // Update regular collision objects (only static ones are in the broadphase;
  // kinematic links use the cast version in the dynamic manager instead)
  for (auto& cow : link2cow_)
  {
    cow.second->setContactDistanceThreshold(collision_margin_data_.getMaxCollisionMargin(cow.second->getName()));
    if (cow.second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
      cow.second->appendCollisionObjectsRaw(static_update_);
  }

  // Also update cast collision objects
  for (auto& cast_cow : link2castcow_)
  {
    cast_cow.second->setContactDistanceThreshold(
        collision_margin_data_.getMaxCollisionMargin(cast_cow.second->getName()));
    // Only add to update if this is a dynamic object (static objects use the non-cast version)
    if (cast_cow.second->m_collisionFilterGroup == CollisionFilterGroups::KinematicFilter)
      cast_cow.second->appendCollisionObjectsRaw(dynamic_update_);
  }

  flushBatchUpdate();
}
}  // namespace tesseract::collision::tesseract_collision_coal
