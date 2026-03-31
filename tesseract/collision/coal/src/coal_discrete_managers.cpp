/**
 * @file coal_discrete_managers.cpp
 * @brief Tesseract Coal contact checker implementation.
 *
 * @author Roelof Oomen, Levi Armstrong
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

#include <tesseract/collision/coal/coal_collision_geometry_cache.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>

namespace tesseract::collision::tesseract_collision_coal
{
static const CollisionShapesConst EMPTY_COLLISION_SHAPES_CONST;
static const tesseract::common::VectorIsometry3d EMPTY_COLLISION_SHAPES_TRANSFORMS;

CoalDiscreteBVHManager::CoalDiscreteBVHManager(std::string name, double gjk_guess_threshold)
  : name_(std::move(name))
  , gjk_guess_threshold_(gjk_guess_threshold)
  , gjk_guess_threshold_sq_(gjk_guess_threshold * gjk_guess_threshold)
{
  static_manager_ = std::make_unique<coal::DynamicAABBTreeCollisionManager>();
  dynamic_manager_ = std::make_unique<coal::DynamicAABBTreeCollisionManager>();
  collision_margin_data_ = CollisionMarginData(0);
}

std::string CoalDiscreteBVHManager::getName() const { return name_; }

DiscreteContactManager::UPtr CoalDiscreteBVHManager::clone() const
{
  CoalCollisionGeometryCache::prune();

  auto manager = std::make_unique<CoalDiscreteBVHManager>(name_, gjk_guess_threshold_);

  for (const auto& cow : link2cow_)
  {
    manager->addCollisionObject(cow.second->clone());
  }

  manager->setActiveCollisionObjects(active_);
  manager->setCollisionMarginData(collision_margin_data_);
  manager->setContactAllowedValidator(validator_);

  return manager;
}

bool CoalDiscreteBVHManager::addCollisionObject(const std::string& name,
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

const CollisionShapesConst& CoalDiscreteBVHManager::getCollisionObjectGeometries(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (cow != link2cow_.end()) ? cow->second->getCollisionGeometries() : EMPTY_COLLISION_SHAPES_CONST;
}

const tesseract::common::VectorIsometry3d&
CoalDiscreteBVHManager::getCollisionObjectGeometriesTransforms(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (cow != link2cow_.end()) ? cow->second->getCollisionGeometriesTransforms() : EMPTY_COLLISION_SHAPES_TRANSFORMS;
}

bool CoalDiscreteBVHManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool CoalDiscreteBVHManager::removeCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    auto it_obj = std::find(collision_objects_.begin(), collision_objects_.end(), name);
    if (it_obj != collision_objects_.end())
      collision_objects_.erase(it_obj);
    const std::vector<CollisionObjectPtr>& objects = it->second->getCollisionObjects();
    coal_co_count_ -= objects.size();
    removeObjects(objects, it->second->m_collisionFilterGroup);
    link2cow_.erase(name);

    auto it_active = std::find(active_.begin(), active_.end(), name);
    if (it_active != active_.end())
      active_.erase(it_active);

    return true;
  }

  return false;
}

void CoalDiscreteBVHManager::removeObjects(const std::vector<CollisionObjectPtr>& objects, short int filter_group)
{
  auto& manager = (filter_group == CollisionFilterGroups::StaticFilter) ? static_manager_ : dynamic_manager_;
  for (const auto& co : objects)
    manager->unregisterObject(co.get());

  invalidateCacheFor(collision_cache, objects);
}

bool CoalDiscreteBVHManager::enableCollisionObject(const std::string& name)
{
  return setCollisionObjectEnabled(name, true);
}

bool CoalDiscreteBVHManager::disableCollisionObject(const std::string& name)
{
  return setCollisionObjectEnabled(name, false);
}

bool CoalDiscreteBVHManager::setCollisionObjectEnabled(const std::string& name, bool enabled)
{
  auto it = link2cow_.find(name);
  if (it == link2cow_.end())
    return false;

  it->second->m_enabled = enabled;
  it->second->gjk_generation_++;
  return true;
}

bool CoalDiscreteBVHManager::isCollisionObjectEnabled(const std::string& name) const
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
    return it->second->m_enabled;

  return false;
}

void CoalDiscreteBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
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

void CoalDiscreteBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
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

void CoalDiscreteBVHManager::setCollisionObjectsTransform(const tesseract::common::TransformMap& transforms)
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

const std::vector<std::string>& CoalDiscreteBVHManager::getCollisionObjects() const { return collision_objects_; }

void CoalDiscreteBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;

  for (auto& co : link2cow_)
  {
    updateCollisionObjectFilters(active_, co.second, static_manager_, dynamic_manager_);
  }

  // This causes a refit on the bvh tree.
  dynamic_manager_->update();
  static_manager_->update();
}

const std::vector<std::string>& CoalDiscreteBVHManager::getActiveCollisionObjects() const { return active_; }

void CoalDiscreteBVHManager::setCollisionMarginData(CollisionMarginData collision_margin_data)
{
  collision_margin_data_ = std::move(collision_margin_data);
  onCollisionMarginDataChanged();
}

const CollisionMarginData& CoalDiscreteBVHManager::getCollisionMarginData() const { return collision_margin_data_; }

void CoalDiscreteBVHManager::setCollisionMarginPairData(const CollisionMarginPairData& pair_margin_data,
                                                        CollisionMarginPairOverrideType override_type)
{
  collision_margin_data_.apply(pair_margin_data, override_type);
  onCollisionMarginDataChanged();
}

void CoalDiscreteBVHManager::setDefaultCollisionMargin(double default_collision_margin)
{
  collision_margin_data_.setDefaultCollisionMargin(default_collision_margin);
  onCollisionMarginDataChanged();
}

void CoalDiscreteBVHManager::setCollisionMarginPair(const std::string& name1,
                                                    const std::string& name2,
                                                    double collision_margin)
{
  collision_margin_data_.setCollisionMargin(name1, name2, collision_margin);
  onCollisionMarginDataChanged();
}

void CoalDiscreteBVHManager::incrementCollisionMargin(double increment)
{
  collision_margin_data_.incrementMargins(increment);
  onCollisionMarginDataChanged();
}

void CoalDiscreteBVHManager::setContactAllowedValidator(
    std::shared_ptr<const tesseract::common::ContactAllowedValidator> validator)
{
  validator_ = std::move(validator);
}

std::shared_ptr<const tesseract::common::ContactAllowedValidator>
CoalDiscreteBVHManager::getContactAllowedValidator() const
{
  return validator_;
}

void CoalDiscreteBVHManager::contactTest(ContactResultMap& collisions, const ContactRequest& request)
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

void CoalDiscreteBVHManager::addCollisionObject(const COW::Ptr& cow)
{
  const std::size_t cnt = cow->getCollisionObjects().size();
  coal_co_count_ += cnt;
  static_update_.reserve(coal_co_count_);
  dynamic_update_.reserve(coal_co_count_);
  link2cow_[cow->getName()] = cow;
  collision_objects_.push_back(cow->getName());

  const std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
  if (cow->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
  {
    // If static add to static manager
    for (const auto& co : objects)
      static_manager_->registerObject(co.get());
  }
  else
  {
    for (const auto& co : objects)
      dynamic_manager_->registerObject(co.get());
  }

  // If active links is not empty update filters to replace the active links list
  if (!active_.empty())
    updateCollisionObjectFilters(active_, cow, static_manager_, dynamic_manager_);

  // This causes a refit on the bvh tree.
  dynamic_manager_->update();
  static_manager_->update();

  // Pre-reserve collision cache to avoid rehashing during contactTest.
  // Accounts for static-vs-dynamic and dynamic-vs-dynamic (self-check) pairs.
  const auto& n_static = static_manager_->size();
  const auto& n_dynamic = dynamic_manager_->size();
  collision_cache.reserve((n_static * n_dynamic) + (n_dynamic * (n_dynamic - 1) / 2));
}

void CoalDiscreteBVHManager::collectTransformUpdate(Link2COW::iterator it, const Eigen::Isometry3d& pose)
{
  const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
  if (transformChanged(cur_tf, pose))
  {
    if ((pose.translation() - cur_tf.translation()).squaredNorm() > gjk_guess_threshold_sq_)
      it->second->gjk_generation_++;
    it->second->setCollisionObjectsTransform(pose);
    auto& update_vec =
        (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter) ? static_update_ : dynamic_update_;
    it->second->appendCollisionObjectsRaw(update_vec);
  }
}

void CoalDiscreteBVHManager::flushBatchUpdate()
{
  if (!static_update_.empty())
    static_manager_->update(static_update_);

  if (!dynamic_update_.empty())
    dynamic_manager_->update(dynamic_update_);
}

void CoalDiscreteBVHManager::onCollisionMarginDataChanged()
{
  static_update_.clear();
  dynamic_update_.clear();

  for (auto& cow : link2cow_)
  {
    cow.second->setContactDistanceThreshold(collision_margin_data_.getMaxCollisionMargin(cow.second->getName()));
    auto& update_vec =
        (cow.second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter) ? static_update_ : dynamic_update_;
    cow.second->appendCollisionObjectsRaw(update_vec);
  }

  flushBatchUpdate();
}
}  // namespace tesseract::collision::tesseract_collision_coal
