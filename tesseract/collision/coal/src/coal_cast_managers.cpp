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
#include <algorithm>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/geometry.h>
#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_collision_geometry_cache.h>
#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/common/utils.h>

namespace tesseract::collision::tesseract_collision_coal
{
static const CollisionShapesConst EMPTY_COLLISION_SHAPES_CONST;
static const tesseract::common::VectorIsometry3d EMPTY_COLLISION_SHAPES_TRANSFORMS;

CoalCastBVHManager::CoalCastBVHManager(std::string name, bool d_arc_compensation)
  : name_(std::move(name)), d_arc_compensation_(d_arc_compensation)
{
  static_manager_ = std::make_unique<coal::DynamicAABBTreeCollisionManager>();
  dynamic_manager_ = std::make_unique<coal::DynamicAABBTreeCollisionManager>();
  contact_test_data_.collision_margin_data = CollisionMarginData(0);
  contact_test_data_.collision_cache = &collision_cache;
}

std::string CoalCastBVHManager::getName() const { return name_; }

ContinuousContactManager::UPtr CoalCastBVHManager::clone() const
{
  CoalCollisionGeometryCache::prune();

  auto manager = std::make_unique<CoalCastBVHManager>(name_, d_arc_compensation_);

  std::vector<COW::Ptr> cloned_cows;
  cloned_cows.reserve(collision_objects_.size());
  for (const auto& id : collision_objects_)
    cloned_cows.push_back(link2cow_.at(id)->clone());

  manager->setCollisionMarginData(contact_test_data_.collision_margin_data);
  // The refit is deferred to the setActiveCollisionObjects below, which rebuilds both trees before any query.
  // FCL's clone deliberately does not defer, because on that backend the two build orders produce measurably
  // different normals; see the note in FCLDiscreteBVHManager::clone.
  manager->addCollisionObjects(cloned_cows, /*defer_update=*/true);
  manager->setActiveCollisionObjects(active_);
  manager->setContactAllowedValidator(contact_test_data_.validator);

  return manager;
}

bool CoalCastBVHManager::addCollisionObject(const tesseract::common::LinkId& id,
                                            const int& mask_id,
                                            const CollisionShapesConst& shapes,
                                            const tesseract::common::VectorIsometry3d& shape_poses,
                                            bool enabled)
{
  if (link2cow_.find(id) != link2cow_.end())
    removeCollisionObject(id);

  const COW::Ptr new_cow = createCoalCollisionObject(id, mask_id, shapes, shape_poses, enabled);
  if (new_cow != nullptr)
  {
    addCollisionObject(new_cow);
    return true;
  }

  return false;
}

bool CoalCastBVHManager::addCollisionObjects(const std::vector<CollisionObjectSpec>& objects)
{
  std::vector<COW::Ptr> cows;
  const bool success = buildCoalCollisionObjects(objects, cows);

  // The primitive does not displace an already-registered object, so do here what the single-object entry point
  // does. Skipping this orphans the old object's broadphase proxy. Every id the batch names is removed, including
  // one whose spec failed to build: the single-object form removes before it creates, so a failed spec leaves that
  // id unregistered. Bulk removal skips the ids it does not hold, so the batch is passed whole and its return,
  // which reports only those absences, is not the return of this call.
  std::vector<tesseract::common::LinkId> displaced;
  displaced.reserve(objects.size());
  for (const auto& obj : objects)
    displaced.push_back(obj.id);

  removeCollisionObjects(displaced);

  if (!cows.empty())
    addCollisionObjects(cows, /*defer_update=*/false);

  return success;
}

const CollisionShapesConst& CoalCastBVHManager::getCollisionObjectGeometries(const tesseract::common::LinkId& id) const
{
  auto cow = link2cow_.find(id);
  return (cow != link2cow_.end()) ? cow->second->getCollisionGeometries() : EMPTY_COLLISION_SHAPES_CONST;
}

const tesseract::common::VectorIsometry3d&
CoalCastBVHManager::getCollisionObjectGeometriesTransforms(const tesseract::common::LinkId& id) const
{
  auto cow = link2cow_.find(id);
  return (cow != link2cow_.end()) ? cow->second->getCollisionGeometriesTransforms() : EMPTY_COLLISION_SHAPES_TRANSFORMS;
}

bool CoalCastBVHManager::hasCollisionObject(const tesseract::common::LinkId& id) const
{
  return (link2cow_.find(id) != link2cow_.end());
}

const CastCollisionObjectWrapper* CoalCastBVHManager::getCastCollisionObject(const tesseract::common::LinkId& id) const
{
  auto it = link2castcow_.find(id);
  return (it == link2castcow_.end()) ? nullptr : it->second.get();
}

std::size_t CoalCastBVHManager::getCollisionCacheSize() const { return collision_cache.size(); }

bool CoalCastBVHManager::removeCollisionObject(const tesseract::common::LinkId& id)
{
  auto it = link2cow_.find(id);
  if (it != link2cow_.end())
  {
    auto it_obj = std::find(collision_objects_.begin(), collision_objects_.end(), id);
    if (it_obj != collision_objects_.end())
      collision_objects_.erase(it_obj);

    const std::vector<CollisionObjectPtr>& objects = it->second->getCollisionObjects();
    const bool is_kinematic = isKinematic(*it->second);
    coal_co_count_ -= objects.size();

    // Every link in link2cow_ has a mate in link2castcow_; both add paths write the two maps together
    // and this is the only place either is erased.
    auto it_cast = link2castcow_.find(id);
    assert(it_cast != link2castcow_.end());

    // Add registers a static link through its regular wrapper and any other link through its cast
    // wrapper. Removal decides on the same question, or the broadphase keeps pointers into a wrapper
    // the maps no longer own.
    if (!is_kinematic)
      unregisterObjects(objects, *static_manager_);
    else
      unregisterObjects(it_cast->second->getCollisionObjects(), *dynamic_manager_);

    // Cache entries do not follow that question. They are keyed on raw collision object addresses and
    // survive the promotion or demotion that swaps which wrapper is registered, so the wrapper that is
    // not unregistered here can still be named by entries, and both wrappers are destroyed below. One
    // pass covers both, which matters because a pass is linear in the whole cache.
    invalidateCacheFor(collision_cache, objects, it_cast->second->getCollisionObjects());

    link2cow_.erase(it);
    active_.erase(id);
    link2castcow_.erase(it_cast);

    return true;
  }

  return false;
}

bool CoalCastBVHManager::removeCollisionObjects(const std::vector<tesseract::common::LinkId>& ids)
{
  std::vector<CollisionObjectPtr> static_objs;
  std::vector<CollisionObjectPtr> dynamic_objs;
  std::vector<CollisionObjectPtr> all_regular_objs;
  std::vector<CollisionObjectPtr> all_cast_objs;
  std::unordered_set<tesseract::common::LinkId> removed;
  removed.reserve(ids.size());

  bool success{ true };
  for (const auto& id : ids)
  {
    auto it = link2cow_.find(id);
    if (it == link2cow_.end())
    {
      success = false;
      continue;
    }

    const std::vector<CollisionObjectPtr>& objects = it->second->getCollisionObjects();
    coal_co_count_ -= objects.size();

    // Every link in link2cow_ has a mate in link2castcow_; both add paths write the two maps together
    // and this is the only place either is erased.
    auto it_cast = link2castcow_.find(id);
    assert(it_cast != link2castcow_.end());
    const std::vector<CollisionObjectPtr>& cast_objects = it_cast->second->getCollisionObjects();

    // Add registers a static link through its regular wrapper and any other link through its cast
    // wrapper. Removal decides on the same question, or the broadphase keeps pointers into a wrapper
    // the maps no longer own.
    if (!isKinematic(*it->second))
      static_objs.insert(static_objs.end(), objects.begin(), objects.end());
    else
      dynamic_objs.insert(dynamic_objs.end(), cast_objects.begin(), cast_objects.end());

    // Cache entries do not follow that question. They are keyed on raw collision object addresses and
    // survive the promotion or demotion that swaps which wrapper is registered, so the wrapper that is
    // not unregistered can still be named by entries and both wrappers must reach the sweep. Holding a
    // shared pointer to each object keeps those addresses valid past the erases below.
    all_regular_objs.insert(all_regular_objs.end(), objects.begin(), objects.end());
    all_cast_objs.insert(all_cast_objs.end(), cast_objects.begin(), cast_objects.end());

    removed.insert(id);
    link2cow_.erase(it);
    active_.erase(id);
    link2castcow_.erase(it_cast);
  }

  if (removed.empty())
    return success;

  // One pass over collision_objects_, preserving the order of the survivors.
  collision_objects_.erase(
      std::remove_if(collision_objects_.begin(),
                     collision_objects_.end(),
                     [&removed](const tesseract::common::LinkId& id) { return removed.find(id) != removed.end(); }),
      collision_objects_.end());

  if (!static_objs.empty())
    unregisterObjects(static_objs, *static_manager_);
  if (!dynamic_objs.empty())
    unregisterObjects(dynamic_objs, *dynamic_manager_);

  // A cache pass is linear in the whole cache, so the batch takes one instead of one per link.
  invalidateCacheFor(collision_cache, all_regular_objs, all_cast_objs);

  return success;
}

bool CoalCastBVHManager::enableCollisionObject(const tesseract::common::LinkId& id)
{
  return setCollisionObjectEnabled(id, true);
}

bool CoalCastBVHManager::disableCollisionObject(const tesseract::common::LinkId& id)
{
  return setCollisionObjectEnabled(id, false);
}

bool CoalCastBVHManager::setCollisionObjectEnabled(const tesseract::common::LinkId& id, bool enabled)
{
  auto it = link2cow_.find(id);
  if (it == link2cow_.end())
    return false;

  it->second->m_enabled = enabled;
  it->second->gjk_generation_++;

  auto cast_it = link2castcow_.find(id);
  if (cast_it != link2castcow_.end())
  {
    CastCOW& cast_cow = *cast_it->second;
    const bool was_enabled = cast_cow.m_enabled;
    cast_cow.m_enabled = enabled;
    cast_cow.gjk_generation_++;

    // A sweep set on a disabled link does not reach the broadphase, so enabling the link publishes where it
    // is.
    if (enabled && !was_enabled)
    {
      static_update_.clear();
      dynamic_update_.clear();
      appendCastBroadphaseUpdate(cast_cow);
      flushBatchUpdate();
    }
  }

  return true;
}

bool CoalCastBVHManager::isCollisionObjectEnabled(const tesseract::common::LinkId& id) const
{
  auto it = link2cow_.find(id);
  if (it != link2cow_.end())
    return it->second->m_enabled;

  return false;
}

Eigen::Isometry3d CoalCastBVHManager::getCollisionObjectsTransform(const tesseract::common::LinkId& id) const
{
  // Returns pose1 (start), which link2cow_ tracks. The pose a sweep ends at is on an active link's cast
  // wrapper: getCastCollisionObject(id)->getSweepEndTransform(). A static link's cast wrapper is not kept
  // current.
  return link2cow_.at(id)->getCollisionObjectsTransform();
}

void CoalCastBVHManager::setCollisionObjectsTransform(const tesseract::common::LinkId& id,
                                                      const Eigen::Isometry3d& pose)
{
  auto it = link2cow_.find(id);
  if (it != link2cow_.end())
  {
    static_update_.clear();
    dynamic_update_.clear();
    collectTransformUpdate(it, pose);
    flushBatchUpdate();
  }
}

void CoalCastBVHManager::setCollisionObjectsTransform(const tesseract::common::LinkIdTransformMap& transforms)
{
  static_update_.clear();
  dynamic_update_.clear();
  for (const auto& [id, tf] : transforms)
  {
    auto it = link2cow_.find(id);
    if (it != link2cow_.end())
      collectTransformUpdate(it, tf);
  }
  flushBatchUpdate();
}

void CoalCastBVHManager::setCollisionObjectsTransform(const tesseract::common::LinkId& id,
                                                      const Eigen::Isometry3d& pose1,
                                                      const Eigen::Isometry3d& pose2)
{
  auto cast_it = link2castcow_.find(id);
  if (cast_it != link2castcow_.end())
  {
    static_update_.clear();
    dynamic_update_.clear();
    auto reg_it = link2cow_.find(id);
    collectCastTransformUpdate(cast_it, reg_it, pose1, pose2);
    flushBatchUpdate();
  }
}

void CoalCastBVHManager::setCollisionObjectsTransform(const tesseract::common::LinkIdTransformMap& pose1,
                                                      const tesseract::common::LinkIdTransformMap& pose2)
{
  if (pose1.size() != pose2.size())
    throw std::runtime_error("CoalCastBVHManager, setCollisionObjectsTransform received " +
                             std::to_string(pose1.size()) + " start poses and " + std::to_string(pose2.size()) +
                             " end poses!");
  static_update_.clear();
  dynamic_update_.clear();
  for (const auto& [id, tf1] : pose1)
  {
    auto it2 = pose2.find(id);
    if (it2 == pose2.end())
      throw std::runtime_error("CoalCastBVHManager, setCollisionObjectsTransform received a start pose for link '" +
                               id.name() + "' with no matching end pose!");

    auto cast_it = link2castcow_.find(id);
    if (cast_it == link2castcow_.end())
      continue;

    auto reg_it = link2cow_.find(id);
    collectCastTransformUpdate(cast_it, reg_it, tf1, it2->second);
  }
  flushBatchUpdate();
}

void CoalCastBVHManager::setCollisionObjectsTransform(const std::vector<tesseract::common::LinkId>& ids,
                                                      const tesseract::common::VectorIsometry3d& poses)
{
  if (ids.size() != poses.size())
    throw std::runtime_error("CoalCastBVHManager, setCollisionObjectsTransform received " + std::to_string(ids.size()) +
                             " ids but " + std::to_string(poses.size()) + " poses!");

  static_update_.clear();
  dynamic_update_.clear();
  for (std::size_t i = 0; i < ids.size(); ++i)
  {
    auto it = link2cow_.find(ids[i]);
    if (it != link2cow_.end())
      collectTransformUpdate(it, poses[i]);
  }
  flushBatchUpdate();
}

void CoalCastBVHManager::setCollisionObjectsTransform(const std::vector<tesseract::common::LinkId>& ids,
                                                      const tesseract::common::VectorIsometry3d& pose1,
                                                      const tesseract::common::VectorIsometry3d& pose2)
{
  if (ids.size() != pose1.size() || ids.size() != pose2.size())
    throw std::runtime_error("CoalCastBVHManager, setCollisionObjectsTransform received " + std::to_string(ids.size()) +
                             " ids but " + std::to_string(pose1.size()) + " start poses and " +
                             std::to_string(pose2.size()) + " end poses!");

  static_update_.clear();
  dynamic_update_.clear();
  for (std::size_t i = 0; i < ids.size(); ++i)
  {
    auto cast_it = link2castcow_.find(ids[i]);
    if (cast_it == link2castcow_.end())
      continue;

    auto reg_it = link2cow_.find(ids[i]);
    collectCastTransformUpdate(cast_it, reg_it, pose1[i], pose2[i]);
  }
  flushBatchUpdate();
}

const std::vector<tesseract::common::LinkId>& CoalCastBVHManager::getCollisionObjects() const
{
  return collision_objects_;
}

void CoalCastBVHManager::setActiveCollisionObjects(const std::unordered_set<tesseract::common::LinkId>& ids)
{
  active_ = ids;

  for (auto& [id, cow] : link2cow_)
  {
    // Get the cast collision object
    CastCOW::Ptr& cast_cow = link2castcow_.at(id);

    // Use the specialized function that properly handles both regular and cast objects
    updateCollisionObjectFilters(active_, cow, cast_cow, static_manager_, dynamic_manager_);
  }

  updateBroadphaseAndCache();
}

const std::unordered_set<tesseract::common::LinkId>& CoalCastBVHManager::getActiveCollisionObjects() const
{
  return active_;
}

void CoalCastBVHManager::setCollisionMarginData(CollisionMarginData collision_margin_data)
{
  contact_test_data_.collision_margin_data = std::move(collision_margin_data);
  onCollisionMarginDataChanged();
}

const CollisionMarginData& CoalCastBVHManager::getCollisionMarginData() const
{
  return contact_test_data_.collision_margin_data;
}

void CoalCastBVHManager::setCollisionMarginPairData(const CollisionMarginPairData& pair_margin_data,
                                                    CollisionMarginPairOverrideType override_type)
{
  contact_test_data_.collision_margin_data.apply(pair_margin_data, override_type);
  onCollisionMarginDataChanged();
}

void CoalCastBVHManager::setDefaultCollisionMargin(double default_collision_margin)
{
  contact_test_data_.collision_margin_data.setDefaultCollisionMargin(default_collision_margin);
  onCollisionMarginDataChanged();
}

void CoalCastBVHManager::setCollisionMarginPair(const tesseract::common::LinkId& id1,
                                                const tesseract::common::LinkId& id2,
                                                double collision_margin)
{
  contact_test_data_.collision_margin_data.setCollisionMargin(id1, id2, collision_margin);
  onCollisionMarginDataChanged();
}

void CoalCastBVHManager::incrementCollisionMargin(double increment)
{
  contact_test_data_.collision_margin_data.incrementMargins(increment);
  onCollisionMarginDataChanged();
}

void CoalCastBVHManager::setContactAllowedValidator(
    std::shared_ptr<const tesseract::common::ContactAllowedValidator> validator)
{
  contact_test_data_.validator = std::move(validator);
}

std::shared_ptr<const tesseract::common::ContactAllowedValidator> CoalCastBVHManager::getContactAllowedValidator() const
{
  return contact_test_data_.validator;
}

void CoalCastBVHManager::contactTest(ContactResultMap& collisions, const ContactRequest& request)
{
  contact_test_data_.res = &collisions;
  contact_test_data_.req = request;
  contact_test_data_.done = false;

  CollisionCallback collisionCallback;
  collisionCallback.cdata = &contact_test_data_;

  // Check static-vs-dynamic first (typically the larger pair set), then
  // dynamic-vs-dynamic (self-check). Order is not significant for correctness
  // but checking static-vs-dynamic first allows early exit via FIRST mode
  // before the self-check.
  if (!static_manager_->empty())
    static_manager_->collide(dynamic_manager_.get(), &collisionCallback);

  if (!contact_test_data_.done && !dynamic_manager_->empty())
    dynamic_manager_->collide(&collisionCallback);
}

void CoalCastBVHManager::addCollisionObject(const COW::Ptr& cow)
{
  const auto lid = cow->getLinkId();
  const std::size_t cnt = cow->getCollisionObjects().size();
  coal_co_count_ += cnt;
  static_update_.reserve(coal_co_count_);
  dynamic_update_.reserve(coal_co_count_);
  link2cow_[lid] = cow;
  collision_objects_.push_back(cow->getLinkId());

  // Must precede the cast wrapper's construction below: that wrapper takes its threshold from this one.
  applyCollisionMarginThreshold(*cow, contact_test_data_.collision_margin_data);

  // Create cast collision object. A static link's cast shapes are deferred - it is collided through its
  // regular wrapper - and built when it goes kinematic. Kinematic objects (e.g. during clone) build
  // immediately to avoid a wasted clone.
  const bool is_kinematic = isKinematic(*cow);
  CastCOW::Ptr& cast_ref =
      (link2castcow_[lid] = makeCastCollisionObject(cow, /*build_swept=*/is_kinematic, d_arc_compensation_));

  if (!is_kinematic)
  {
    const std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
    for (const auto& co : objects)
      static_manager_->registerObject(co.get());
  }
  else
  {
    for (const auto& co : cast_ref->getCollisionObjects())
      dynamic_manager_->registerObject(co.get());
  }

  if (!active_.empty())
    updateCollisionObjectFilters(active_, cow, cast_ref, static_manager_, dynamic_manager_);

  updateBroadphaseAndCache();
}

void CoalCastBVHManager::addCollisionObjects(const std::vector<COW::Ptr>& cows, bool defer_update)
{
  std::vector<coal::CollisionObject*> static_objs;
  std::vector<coal::CollisionObject*> dynamic_objs;
  static_objs.reserve(cows.size());
  dynamic_objs.reserve(cows.size());

  for (const auto& cow : cows)
  {
    const auto lid = cow->getLinkId();
    coal_co_count_ += cow->getCollisionObjects().size();
    link2cow_[lid] = cow;
    collision_objects_.push_back(lid);

    // Must precede the cast wrapper's construction below: that wrapper takes its threshold from this one.
    applyCollisionMarginThreshold(*cow, contact_test_data_.collision_margin_data);

    const bool is_kinematic = isKinematic(*cow);
    CastCOW::Ptr& cast_ref =
        (link2castcow_[lid] = makeCastCollisionObject(cow, /*build_swept=*/is_kinematic, d_arc_compensation_));

    if (!is_kinematic)
    {
      for (const auto& co : cow->getCollisionObjects())
        static_objs.push_back(co.get());
    }
    else
    {
      for (const auto& co : cast_ref->getCollisionObjects())
        dynamic_objs.push_back(co.get());
    }
  }

  static_update_.reserve(coal_co_count_);
  dynamic_update_.reserve(coal_co_count_);

  // Bulk init builds balanced trees via topdown construction.
  if (!static_objs.empty())
    static_manager_->registerObjects(static_objs);
  if (!dynamic_objs.empty())
    dynamic_manager_->registerObjects(dynamic_objs);

  if (!defer_update)
  {
    if (!active_.empty())
    {
      for (auto& [id, cow_ref] : link2cow_)
      {
        CastCOW::Ptr& cast_cow = link2castcow_.at(id);
        updateCollisionObjectFilters(active_, cow_ref, cast_cow, static_manager_, dynamic_manager_);
      }
    }

    updateBroadphaseAndCache();
  }
}

void CoalCastBVHManager::appendRegularBroadphaseUpdate(COW& reg_cow)
{
  if (!isKinematic(reg_cow))
    reg_cow.appendCollisionObjectsRaw(static_update_);
}

void CoalCastBVHManager::appendCastBroadphaseUpdate(CastCOW& cast_cow)
{
  if (isKinematic(cast_cow))
    cast_cow.appendCollisionObjectsRaw(dynamic_update_);
}

bool CoalCastBVHManager::collectRegularTransformUpdate(COW& reg_cow, const Eigen::Isometry3d& pose)
{
  const Eigen::Isometry3d& cur_tf = reg_cow.getCollisionObjectsTransform();
  if (tesseract::common::almostEqualRelativeAndAbs(cur_tf, pose))
    return false;

  reg_cow.gjk_generation_++;
  reg_cow.setCollisionObjectsTransform(pose);
  appendRegularBroadphaseUpdate(reg_cow);
  return true;
}

void CoalCastBVHManager::collectTransformUpdate(Link2COW::iterator it, const Eigen::Isometry3d& pose)
{
  const bool moved = collectRegularTransformUpdate(*it->second, pose);

  auto cast_it = link2castcow_.find(it->first);
  if (cast_it == link2castcow_.end())
    return;

  CastCOW& cast_cow = *cast_it->second;

  // A static link's cast wrapper carries no pose or sweep that anything reads: it is in no broadphase, and
  // updateCollisionObjectFilters brings both current at the moment the link is promoted. Writing them here
  // would recompute an AABB per shape, per state update, for state that promotion discards anyway.
  if (!isKinematic(cast_cow))
    return;

  // A pose set without a sweep must leave no sweep behind: re-applying the last dual-pose call's sweep from
  // the new pose would sweep the object through space it never crossed. Whether the wrapper holds a sweep
  // is independent of whether the link moved, so the two are asked separately - setting a link back to the
  // pose a sweep started from moves nothing and must still drop it.
  if (!moved && !cast_cow.isSwept())
    return;

  cast_cow.setSweep(pose, pose);
  cast_cow.gjk_generation_++;
  appendCastBroadphaseUpdate(cast_cow);
}

void CoalCastBVHManager::collectCastTransformUpdate(Link2CastCOW::iterator cast_it,
                                                    Link2COW::iterator reg_it,
                                                    const Eigen::Isometry3d& pose1,
                                                    const Eigen::Isometry3d& pose2)
{
  CastCOW::Ptr& cow = cast_it->second;

  // Publish the regular object before the early return below: for a static link it is what static_manager_
  // holds. Its GJK generation follows whether it changed, which is what the helper reports - whether the
  // cast wrapper changed is a separate question, and one the static path never publishes.
  if (reg_it != link2cow_.end())
    collectRegularTransformUpdate(*reg_it->second, pose1);

  // A static link's cast wrapper is deferred: it holds the link's own geometry, which carries no sweep, and
  // nothing reads its pose until the link is promoted. Setting a sweep on a static link is meaningless in
  // any case.
  if (!isKinematic(*cow))
    return;

  // Read before setSweep overwrites the wrapper's pose.
  const bool moved = !tesseract::common::almostEqualRelativeAndAbs(cow->getCollisionObjectsTransform(), pose1);

  // A disabled link takes its sweep as an enabled one does, so that it is the sweep the link is checked
  // along once enabled.
  if (cow->setSweep(pose1, pose2) || moved)
    cow->gjk_generation_++;

  // Append to the broadphase update vector (flushed by caller). A disabled link is checked against nothing
  // and stays out of it; enabling it publishes the bounds it then has.
  if (cow->m_enabled)
    appendCastBroadphaseUpdate(*cow);
}

void CoalCastBVHManager::flushBatchUpdate()
{
  if (!static_update_.empty())
  {
    if (static_update_.size() * 2 >= static_manager_->size())
      static_manager_->update();
    else
      static_manager_->update(static_update_);
  }

  if (!dynamic_update_.empty())
  {
    // When most dynamic objects changed, a full refit is O(n) vs O(k*log n) for
    // per-object remove+reinsert. In trajectory optimization nearly all kinematic
    // objects move each step, so the refit path is typically faster.
    if (dynamic_update_.size() * 2 >= dynamic_manager_->size())
      dynamic_manager_->update();
    else
      dynamic_manager_->update(dynamic_update_);
  }
}

void CoalCastBVHManager::updateBroadphaseAndCache()
{
  dynamic_manager_->update();
  static_manager_->update();

  const auto n_static = static_manager_->size();
  const auto n_dynamic = dynamic_manager_->size();
  collision_cache.reserve((n_static * n_dynamic) + (n_dynamic * (n_dynamic - 1) / 2));
}

void CoalCastBVHManager::onCollisionMarginDataChanged()
{
  static_update_.clear();
  dynamic_update_.clear();

  // Update regular collision objects (only static ones are in the broadphase;
  // kinematic links use the cast version in the dynamic manager instead)
  for (auto& cow : link2cow_)
  {
    if (applyCollisionMarginThreshold(*cow.second, contact_test_data_.collision_margin_data))
      appendRegularBroadphaseUpdate(*cow.second);
  }

  // Also update cast collision objects
  for (auto& cast_cow : link2castcow_)
  {
    if (applyCollisionMarginThreshold(*cast_cow.second, contact_test_data_.collision_margin_data))
      appendCastBroadphaseUpdate(*cast_cow.second);
  }

  flushBatchUpdate();
}
}  // namespace tesseract::collision::tesseract_collision_coal
