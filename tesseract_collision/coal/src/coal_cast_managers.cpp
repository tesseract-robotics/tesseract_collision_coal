/**
 * @file coal_cast_managers.cpp
 * @brief Tesseract Coal contact checker implementation.
 *
 * @author Roelof Oomen
 * @date Aug 04, 2025
 * @version TODO
 * @bug No known bugs
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
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>
#include <tesseract_collision/coal/coal_cast_managers.h>
#include <tesseract_collision/coal/coal_casthullshape.h>
#include <tesseract_collision/coal/coal_utils.h>

namespace tesseract_collision::tesseract_collision_coal
{
static const CollisionShapesConst EMPTY_COLLISION_SHAPES_CONST;
static const tesseract_common::VectorIsometry3d EMPTY_COLLISION_SHAPES_TRANSFORMS;

CoalCastBVHManager::CoalCastBVHManager(std::string name) : name_(std::move(name))
{
  static_manager_ = std::make_unique<coal::DynamicAABBTreeCollisionManager>();
  dynamic_manager_ = std::make_unique<coal::DynamicAABBTreeCollisionManager>();
  collision_margin_data_ = CollisionMarginData(0);
}

std::string CoalCastBVHManager::getName() const { return name_; }

ContinuousContactManager::UPtr CoalCastBVHManager::clone() const
{
  auto manager = std::make_unique<CoalCastBVHManager>();

  for (const auto& cow : link2cow_)
  {
    manager->addCollisionObject(cow.second->clone());
  }

  manager->setActiveCollisionObjects(active_);
  manager->setCollisionMarginData(collision_margin_data_);
  manager->setContactAllowedValidator(validator_);

  return manager;
}

bool CoalCastBVHManager::addCollisionObject(const std::string& name,
                                            const int& mask_id,
                                            const CollisionShapesConst& shapes,
                                            const tesseract_common::VectorIsometry3d& shape_poses,
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

const tesseract_common::VectorIsometry3d&
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
    collision_objects_.erase(std::find(collision_objects_.begin(), collision_objects_.end(), name));
    const std::vector<CollisionObjectPtr>& objects = it->second->getCollisionObjects();
    coal_co_count_ -= objects.size();
    removeObjects(objects);
    link2cow_.erase(name);

    // Also remove from cast map
    auto it_cast = link2castcow_.find(name);
    if (it_cast != link2castcow_.end())
    {
      const std::vector<CollisionObjectPtr>& objects_cast = it_cast->second->getCollisionObjects();
      removeObjects(objects_cast);
      link2castcow_.erase(name);
    }

    return true;
  }

  return false;
}

void CoalCastBVHManager::removeObjects(const std::vector<CollisionObjectPtr>& objects)
{
  std::vector<coal::CollisionObject*> static_objs;
  static_manager_->getObjects(static_objs);

  std::vector<coal::CollisionObject*> dynamic_objs;
  dynamic_manager_->getObjects(dynamic_objs);

  // Must check if object exists in the manager before calling unregister.
  // If it does not exist and unregister is called it is undefined behavior
  for (const auto& co : objects)
  {
    auto static_it = std::find(static_objs.begin(), static_objs.end(), co.get());
    if (static_it != static_objs.end())
      static_manager_->unregisterObject(co.get());

    auto dynamic_it = std::find(dynamic_objs.begin(), dynamic_objs.end(), co.get());
    if (dynamic_it != dynamic_objs.end())
      dynamic_manager_->unregisterObject(co.get());
  }
}

bool CoalCastBVHManager::enableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;

    // Also enable the cast version
    auto cast_it = link2castcow_.find(name);
    if (cast_it != link2castcow_.end())
      cast_it->second->m_enabled = true;

    return true;
  }

  return false;
}

bool CoalCastBVHManager::disableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;

    // Also disable the cast version
    auto cast_it = link2castcow_.find(name);
    if (cast_it != link2castcow_.end())
      cast_it->second->m_enabled = false;

    return true;
  }

  return false;
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
    const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
    // Note: If the transform has not changed do not updated to prevent unnecessary re-balancing of the BVH tree
    if (!cur_tf.translation().isApprox(pose.translation(), 1e-8) || !cur_tf.rotation().isApprox(pose.rotation(), 1e-8))
    {
      it->second->setCollisionObjectsTransform(pose);

      // Also update the cast version if it exists
      auto cast_it = link2castcow_.find(name);
      if (cast_it != link2castcow_.end())
      {
        cast_it->second->setCollisionObjectsTransform(pose);
      }

      if (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
      {
        // Note: Calling update causes a re-balance of the AABB tree, which is expensive
        static_manager_->update(it->second->getCollisionObjectsRaw());
      }
      else
      {
        // If this is an active/kinematic object, update the cast version in the dynamic manager
        if (cast_it != link2castcow_.end())
        {
          dynamic_manager_->update(cast_it->second->getCollisionObjectsRaw());
        }
        else
        {
          // Note: Calling update causes a re-balance of the AABB tree, which is expensive
          dynamic_manager_->update(it->second->getCollisionObjectsRaw());
        }
      }
    }
  }
}

void CoalCastBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                      const tesseract_common::VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  static_update_.clear();
  dynamic_update_.clear();
  for (auto i = 0U; i < names.size(); ++i)
  {
    auto it = link2cow_.find(names[i]);
    if (it != link2cow_.end())
    {
      const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
      // Note: If the transform has not changed do not updated to prevent unnecessary re-balancing of the BVH tree
      if (!cur_tf.translation().isApprox(poses[i].translation(), 1e-8) ||
          !cur_tf.rotation().isApprox(poses[i].rotation(), 1e-8))
      {
        it->second->setCollisionObjectsTransform(poses[i]);

        // Also update the cast version if it exists
        auto cast_it = link2castcow_.find(names[i]);
        if (cast_it != link2castcow_.end())
        {
          cast_it->second->setCollisionObjectsTransform(poses[i]);

          if (it->second->m_collisionFilterGroup == CollisionFilterGroups::KinematicFilter)
          {
            // For kinematic objects, add the cast version to the dynamic update
            std::vector<CollisionObjectRawPtr>& co = cast_it->second->getCollisionObjectsRaw();
            dynamic_update_.insert(dynamic_update_.end(), co.begin(), co.end());
            continue;  // Skip adding the regular version
          }
        }

        std::vector<CollisionObjectRawPtr>& co = it->second->getCollisionObjectsRaw();
        if (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
        {
          static_update_.insert(static_update_.end(), co.begin(), co.end());
        }
        else
        {
          dynamic_update_.insert(dynamic_update_.end(), co.begin(), co.end());
        }
      }
    }
  }

  // This is because Coal supports batch update which only re-balances the tree once
  if (!static_update_.empty())
    static_manager_->update(static_update_);

  if (!dynamic_update_.empty())
    dynamic_manager_->update(dynamic_update_);
}

void CoalCastBVHManager::setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms)
{
  static_update_.clear();
  dynamic_update_.clear();
  for (const auto& transform : transforms)
  {
    auto it = link2cow_.find(transform.first);
    if (it != link2cow_.end())
    {
      const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
      // Note: If the transform has not changed do not updated to prevent unnecessary re-balancing of the BVH tree
      if (!cur_tf.translation().isApprox(transform.second.translation(), 1e-8) ||
          !cur_tf.rotation().isApprox(transform.second.rotation(), 1e-8))
      {
        it->second->setCollisionObjectsTransform(transform.second);

        // Also update the cast version if it exists
        auto cast_it = link2castcow_.find(transform.first);
        if (cast_it != link2castcow_.end())
        {
          cast_it->second->setCollisionObjectsTransform(transform.second);

          if (it->second->m_collisionFilterGroup == CollisionFilterGroups::KinematicFilter)
          {
            // For kinematic objects, add the cast version to the dynamic update
            std::vector<CollisionObjectRawPtr>& co = cast_it->second->getCollisionObjectsRaw();
            dynamic_update_.insert(dynamic_update_.end(), co.begin(), co.end());
            continue;  // Skip adding the regular version
          }
        }

        std::vector<CollisionObjectRawPtr>& co = it->second->getCollisionObjectsRaw();
        if (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
        {
          static_update_.insert(static_update_.end(), co.begin(), co.end());
        }
        else
        {
          dynamic_update_.insert(dynamic_update_.end(), co.begin(), co.end());
        }
      }
    }
  }

  // This is because Coal supports batch update which only re-balances the tree once
  if (!static_update_.empty())
    static_manager_->update(static_update_);

  if (!dynamic_update_.empty())
    dynamic_manager_->update(dynamic_update_);
}

void CoalCastBVHManager::setCollisionObjectsTransform(const std::string& name,
                                                      const Eigen::Isometry3d& pose1,
                                                      const Eigen::Isometry3d& pose2)
{
  auto it = link2castcow_.find(name);
  if (it != link2castcow_.end())
  {
    COW::Ptr& cow = it->second;

    const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
    if (!cur_tf.translation().isApprox(pose1.translation(), 1e-8) ||
        !cur_tf.rotation().isApprox(pose1.rotation(), 1e-8))
    {
      it->second->setCollisionObjectsTransform(pose1);

      // Convert to coal::Transform3s format
      const auto tf1 = coal::Transform3s(pose1.rotation(), pose1.translation());
      const auto tf2 = coal::Transform3s(pose2.rotation(), pose2.translation());

      // Calculate relative transform directly in Coal format
      const auto relative_transform = tf1.inverseTimes(tf2);

      for (auto& co : cow->getCollisionObjects())
      {
        if (auto* cast_shape = dynamic_cast<CastHullShape*>(co->collisionGeometry().get()))
        {
          // Update the cast transform with the relative transform
          cast_shape->updateCastTransform(relative_transform);
        }
      }

      // Now update Broadphase AABB
      if (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
      {
        static_manager_->update(it->second->getCollisionObjectsRaw());
      }
      else
      {
        dynamic_manager_->update(it->second->getCollisionObjectsRaw());
      }
    }
  }
}

void CoalCastBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                      const tesseract_common::VectorIsometry3d& pose1,
                                                      const tesseract_common::VectorIsometry3d& pose2)
{
  assert(names.size() == pose1.size());
  assert(names.size() == pose2.size());
  for (auto i = 0U; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], pose1[i], pose2[i]);
}

void CoalCastBVHManager::setCollisionObjectsTransform(const tesseract_common::TransformMap& pose1,
                                                      const tesseract_common::TransformMap& pose2)
{
  assert(pose1.size() == pose2.size());
  auto it1 = pose1.begin();
  auto it2 = pose2.begin();
  while (it1 != pose1.end())
  {
    assert(pose1.find(it1->first) != pose2.end());
    setCollisionObjectsTransform(it1->first, it1->second, it2->second);
    std::advance(it1, 1);
    std::advance(it2, 1);
  }
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
    std::shared_ptr<const tesseract_common::ContactAllowedValidator> validator)
{
  validator_ = std::move(validator);
}

std::shared_ptr<const tesseract_common::ContactAllowedValidator> CoalCastBVHManager::getContactAllowedValidator() const
{
  return validator_;
}

void CoalCastBVHManager::contactTest(ContactResultMap& collisions, const ContactRequest& request)
{
  ContactTestData cdata(active_, collision_margin_data_, validator_, request, collisions);

  if (collision_margin_data_.getMaxCollisionMargin() > 0)
  {
    DistanceCallback distanceCallback;
    distanceCallback.cdata = &cdata;

    // TODO: Should the order be flipped?
    if (!static_manager_->empty())
      static_manager_->collide(dynamic_manager_.get(), &distanceCallback);

    // It looks like the self check is as fast as selfDistanceContactTest even though it is N^2
    if (!cdata.done && !dynamic_manager_->empty())
      dynamic_manager_->collide(&distanceCallback);
  }
  else
  {
    CollisionCallback collisionCallback;
    collisionCallback.cdata = &cdata;

    // TODO: Should the order be flipped?
    if (!static_manager_->empty())
      static_manager_->collide(dynamic_manager_.get(), &collisionCallback);

    // It looks like the self check is as fast as selfDistanceContactTest even though it is N^2
    if (!cdata.done && !dynamic_manager_->empty())
      dynamic_manager_->collide(&collisionCallback);
  }
}

void CoalCastBVHManager::addCollisionObject(const COW::Ptr& cow)
{
  const std::size_t cnt = cow->getCollisionObjectsRaw().size();
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
    // If static add to static manager
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
}

void CoalCastBVHManager::onCollisionMarginDataChanged()
{
  static_update_.clear();
  dynamic_update_.clear();

  // Update regular collision objects
  for (auto& cow : link2cow_)
  {
    cow.second->setContactDistanceThreshold(collision_margin_data_.getMaxCollisionMargin() / 2.0);
    std::vector<CollisionObjectRawPtr>& co = cow.second->getCollisionObjectsRaw();
    if (cow.second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
    {
      static_update_.insert(static_update_.end(), co.begin(), co.end());
    }
    else
    {
      dynamic_update_.insert(dynamic_update_.end(), co.begin(), co.end());
    }
  }

  // Also update cast collision objects
  for (auto& cast_cow : link2castcow_)
  {
    cast_cow.second->setContactDistanceThreshold(collision_margin_data_.getMaxCollisionMargin() / 2.0);
    // Only add to update if this is a dynamic object (static objects use the non-cast version)
    if (cast_cow.second->m_collisionFilterGroup == CollisionFilterGroups::KinematicFilter)
    {
      std::vector<CollisionObjectRawPtr>& co = cast_cow.second->getCollisionObjectsRaw();
      dynamic_update_.insert(dynamic_update_.end(), co.begin(), co.end());
    }
  }

  if (!static_update_.empty())
    static_manager_->update(static_update_);

  if (!dynamic_update_.empty())
    dynamic_manager_->update(dynamic_update_);
}

}  // namespace tesseract_collision::tesseract_collision_coal
