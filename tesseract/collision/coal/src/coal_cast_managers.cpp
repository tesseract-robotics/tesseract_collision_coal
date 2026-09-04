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
#include <stdexcept>
#include <string>
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

/**
 * @brief Retire a wrapper being removed: its cache entries are always invalidated, and it is
 *        unregistered from @p manager only if @p registered says it was actually registered there.
 */
static void retireWrapper(const COW::Ptr& wrapper,
                          bool registered,
                          coal::BroadPhaseCollisionManager& manager,
                          CollisionCacheMap& cache)
{
  if (registered)
    removeObjects(cache, wrapper->getCollisionObjects(), manager);
  else
    invalidateCacheFor(cache, wrapper->getCollisionObjects());
}

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

std::size_t CoalCastBVHManager::getCollisionCacheSize() const { return collision_cache.size(); }

bool CoalCastBVHManager::removeCollisionObject(const tesseract::common::LinkId& id)
{
  auto it = link2cow_.find(id);
  if (it != link2cow_.end())
  {
    auto it_obj = std::find(collision_objects_.begin(), collision_objects_.end(), id);
    if (it_obj != collision_objects_.end())
      collision_objects_.erase(it_obj);

    // Add registers a static link through its regular wrapper and any other link through its cast
    // wrapper. Removal decides on the same question, or the broadphase keeps pointers into a wrapper
    // the maps no longer own. Cache entries do not follow that question: they are keyed on raw
    // collision object addresses, and both wrappers are destroyed below, so the wrapper that is not
    // unregistered still has its entries erased.
    const bool is_kinematic = isKinematic(*it->second);
    coal_co_count_ -= it->second->getCollisionObjects().size();
    retireWrapper(it->second, !is_kinematic, *static_manager_, collision_cache);
    link2cow_.erase(it);

    active_.erase(id);

    // Also remove from cast map
    auto it_cast = link2castcow_.find(id);
    if (it_cast != link2castcow_.end())
    {
      retireWrapper(it_cast->second, is_kinematic, *dynamic_manager_, collision_cache);
      link2castcow_.erase(it_cast);
    }

    return true;
  }

  return false;
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
    cast_it->second->m_enabled = enabled;
    cast_it->second->gjk_generation_++;
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
  // Returns pose1 (start) — link2cow_ tracks the start pose; pose2 is encoded in the cast hull and unrecoverable.
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
    COW::Ptr& cast_cow = link2castcow_.at(id);

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
  COW::Ptr& cast_ref = (link2castcow_[lid] = makeCastCollisionObject(cow, /*build_swept=*/is_kinematic));

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
    COW::Ptr& cast_ref = (link2castcow_[lid] = makeCastCollisionObject(cow, /*build_swept=*/is_kinematic));

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
        COW::Ptr& cast_cow = link2castcow_.at(id);
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

void CoalCastBVHManager::appendCastBroadphaseUpdate(COW& cast_cow)
{
  if (isKinematic(cast_cow))
    cast_cow.appendCollisionObjectsRaw(dynamic_update_);
}

bool CoalCastBVHManager::collectRegularTransformUpdate(COW& reg_cow, const Eigen::Isometry3d& pose)
{
  const Eigen::Isometry3d& cur_tf = reg_cow.getCollisionObjectsTransform();
  if (!transformChanged(cur_tf, pose))
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

  COW& cast_cow = *cast_it->second;

  // A static link's cast wrapper carries no pose or sweep that anything reads: it is in no broadphase, and
  // updateCollisionObjectFilters brings both current at the moment the link is promoted. Writing them here
  // would recompute an AABB per shape, per state update, for state that promotion discards anyway.
  if (!isKinematic(cast_cow))
    return;

  // A pose set without a sweep must leave no sweep behind: the hulls hold whatever the last dual-pose call
  // wrote, and re-applying that delta from the new pose sweeps the object through space it never crossed.
  // Whether a hull holds a stale sweep is independent of whether the link moved, so the two are asked
  // separately - setting a link back to the pose a sweep started from moves nothing and must still drop it.
  const bool swept = updateCastShapeTransforms(cast_cow, pose, pose);
  if (!swept && !moved)
    return;

  cast_cow.gjk_generation_++;

  // Re-applied even when the pose has not moved: this recomputes each object's AABB from the hull's local
  // one, and the broadphase update copies that AABB rather than deriving it.
  cast_cow.setCollisionObjectsTransform(pose);
  appendCastBroadphaseUpdate(cast_cow);
}

/// Rotation axis of `R`, read from its symmetric part:
/// `R + R^T == 2*cos(phi)*I + 2*(1 - cos(phi))*k*k^T`. Used past 120 degrees, where the
/// skew-symmetric part -- which is `2*sin(phi)*k` and gives the axis directly below that -- shrinks
/// toward zero as phi approaches pi, taking the precision of its direction with it, and is exactly
/// zero at a half turn. The two extractions are well conditioned across a wide overlap, so the
/// handoff is not a cliff.
static coal::Vec3s screwAxisFromRotation(const coal::Matrix3s& R, double cos_phi)
{
  const coal::Matrix3s outer = (R + R.transpose()) * 0.5 - cos_phi * coal::Matrix3s::Identity();

  // Columns of k*k^T are k scaled by each k_i, so the largest-diagonal column is the best determined.
  // It cannot vanish: some k_i^2 is at least 1/3, and this extraction runs only past the handoff,
  // where 1 - cos_phi is at least 1.5, so `col` has norm at least 1.5/sqrt(3). The guard below holds
  // only for a caller that ignores that precondition.
  Eigen::Index j = 0;
  outer.diagonal().maxCoeff(&j);
  const coal::Vec3s col = outer.col(j);
  const double n = col.norm();
  if (n == 0.0)
    return coal::Vec3s::UnitX();  // LCOV_EXCL_LINE

  // k*k^T loses the sign. Where the skew part still carries one, agree with it; where it does not,
  // cot(phi/2) has gone to zero with it and nothing downstream can tell the two apart.
  const coal::Vec3s k = col / n;
  const double sign = (R(2, 1) - R(1, 2)) * k.x() + (R(0, 2) - R(2, 0)) * k.y() + (R(1, 0) - R(0, 1)) * k.z();
  return (sign < 0.0) ? coal::Vec3s(-k) : k;
}

/// Precomputed rotation-angle scalars for d_arc computation.
/// These depend only on the rotation angle, which is conjugation-invariant
/// and therefore identical for all shapes on the same link — computed once
/// from the link-level relative rotation before the per-shape loop. The same invariance is what
/// makes `cos_phi` safe as a branch selector: a per-shape conjugation preserves the trace, so every
/// shape on a link lands on the same side of the axis-extraction handoff as the link itself.
struct DArcScalars
{
  double sagitta_factor{ 0.0 };  ///< 1 - cos(phi/2); zero means negligible rotation.
  double inv_4sin2_half{ 0.0 };  ///< 1 / (2*(1 - cos_phi)) == 1 / (4*sin^2(phi/2)); scales the raw skew vector.
  double cos_phi{ 0.0 };         ///< (trace(R) - 1) / 2; selects the axis-extraction branch.
};

/// Compute the rotation-angle scalars from a link-level cast transform.
/// All trig is avoided via half-angle identities on the rotation matrix trace.
/// Returns zero-initialized scalars when the rotation is negligible (phi < ~1e-7 rad).
static DArcScalars computeDArcScalars(const coal::Transform3s& link_cast_tf)
{
  const coal::Matrix3s& R = link_cast_tf.getRotation();
  const double cos_phi = std::clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double one_plus_cos = 1.0 + cos_phi;
  if (one_plus_cos > 2.0 - 1e-14)
    return {};
  const double cos_half = std::sqrt(one_plus_cos * 0.5);
  // Singular only at phi = 0, which the early return above already excludes. sin_half is not needed
  // on the fast path at all: the raw skew vector carries the 2*sin(phi) scale, and this factor
  // absorbs both that and cot(phi/2).
  return { 1.0 - cos_half, 1.0 / (2.0 * (1.0 - cos_phi)), cos_phi };
}

/// Compute the arc-chord sagitta (d_arc) for a single shape, given precomputed scalars.
/// d_arc = r_max * (1 - cos(phi/2)), where r_max is the maximum distance from any
/// point on the shape's bounding sphere to the screw axis.
static double computeDArc(const coal::Transform3s& cast_tf, const coal::ShapeBase& shape, const DArcScalars& s)
{
  // The sagitta reads the shape's cached bound, which wrapping it does not populate. An unbounded
  // shape carries radius -1, reaching the broadphase as a NaN AABB: pairs drop unobserved.
  assert(shape.aabb_radius >= 0.0);

  if (s.sagitta_factor == 0.0)
    return 0.0;

  // Screw axis: closest point to the origin in the shape-local frame.
  // c = t_perp/2 + (k x t_perp) * cos(phi/2) / (2*sin(phi/2))
  const coal::Matrix3s& R = cast_tf.getRotation();
  const coal::Vec3s& t = cast_tf.getTranslation();

  if (s.cos_phi > -0.5)
  {
    // The skew part is used unnormalised: it appears only inside projections, which are scale-free
    // once divided by |w|^2, and scaled by inv_4sin2_half, which absorbs both the 2*sin(phi) and
    // the cot(phi/2). What keeps 1/|w|^2 finite is computeDArcScalars' early return, which rejects
    // phi below ~1.4e-7 rad -- |w| = 2*sin(phi) is not otherwise bounded away from zero here.
    const coal::Vec3s w(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    const double inv_ww = 1.0 / w.squaredNorm();
    const coal::Vec3s t_perp = t - (t.dot(w) * inv_ww) * w;
    const coal::Vec3s c = 0.5 * t_perp + 0.5 * s.inv_4sin2_half * w.cross(t_perp);
    const coal::Vec3s pc = shape.aabb_center - c;
    const double dist_to_axis = (pc - (pc.dot(w) * inv_ww) * w).norm();
    return (dist_to_axis + shape.aabb_radius) * s.sagitta_factor;
  }

  // Past the handoff the skew part shrinks toward zero as phi approaches pi and its direction
  // degrades with it, reaching exactly zero at a half turn -- where a 1/(2*sin(phi)) scale would be
  // +inf and the product 0 * inf a NaN, which coal's `radius < 0` validation passes straight
  // through. Read the symmetric part instead, and recover cot(phi/2) here rather than carrying it,
  // since this branch is rare.
  const coal::Vec3s k = screwAxisFromRotation(R, s.cos_phi);
  const double sin_half = std::sqrt((1.0 - s.cos_phi) * 0.5);
  const double cos_half = 1.0 - s.sagitta_factor;
  const coal::Vec3s t_perp = t - t.dot(k) * k;
  const coal::Vec3s c = 0.5 * t_perp + 0.5 * (cos_half / sin_half) * k.cross(t_perp);
  const coal::Vec3s pc = shape.aabb_center - c;
  const double dist_to_axis = (pc - pc.dot(k) * k).norm();
  return (dist_to_axis + shape.aabb_radius) * s.sagitta_factor;
}

bool CoalCastBVHManager::updateCastShapeTransforms(COW& cast_cow,
                                                   const Eigen::Isometry3d& pose1,
                                                   const Eigen::Isometry3d& pose2) const
{
  assert(isKinematic(cast_cow));
  assert(!castCowNeedsSweptBuild(cast_cow));

  bool changed = false;

  // A zero-length sweep is the unswept state, which every hull resolves to regardless of its local offset,
  // so it is clearSweep's business rather than a per-shape product - and the products would not reach it
  // exactly anyway, since (tf * local)^-1 * (tf * local) leaves rounding noise that defeats the equality
  // test below.
  //
  // The comparison must be exact because it stands in for that computation: whatever it accepts has to
  // produce the identity, and a relative tolerance accepts real motion far from the origin.
  if (pose1.matrix() == pose2.matrix())
  {
    for (const auto& co : cast_cow.getCollisionObjects())
    {
      auto* cast_shape = static_cast<CastHullShape*>(co->collisionGeometryPtr());
      changed = cast_shape->clearSweep() || changed;
    }

    return changed;
  }

  const coal::Transform3s tf1(pose1.rotation(), pose1.translation());
  const coal::Transform3s tf2(pose2.rotation(), pose2.translation());

  // Precompute rotation-angle scalars once per link (conjugation-invariant).
  DArcScalars d_arc_scalars;
  if (d_arc_compensation_)
    d_arc_scalars = computeDArcScalars(tf1.inverseTimes(tf2));

  const auto& shape_poses = cast_cow.getCollisionGeometriesTransforms();

  // Update cast transforms so computeLocalAABB reflects the swept volume.
  for (const auto& co : cast_cow.getCollisionObjects())
  {
    auto* cast_shape = static_cast<CastHullShape*>(co->collisionGeometryPtr());
    assert(cast_shape != nullptr);

    // Compute per-shape relative transform accounting for local offset.
    // Each shape's world transform is link_tf * local_tf, so the relative
    // motion in the shape's local frame is:
    //   (tf1 * local_tf)^-1 * (tf2 * local_tf)
    // This matches Bullet's compound shape handling where each child gets
    // its own delta_tf = (tf1 * local_tf).inverseTimes(tf2 * local_tf).
    const auto& shape_pose = shape_poses[static_cast<std::size_t>(co->getShapeIndex())];
    const auto local_tf = coal::Transform3s(shape_pose.rotation(), shape_pose.translation());
    const coal::Transform3s new_cast_tf = (tf1 * local_tf).inverseTimes(tf2 * local_tf);

    const auto& cur_cast_tf = cast_shape->getCastTransform();
    if (new_cast_tf == cur_cast_tf)
      continue;

    changed = true;
    if (d_arc_compensation_)
      cast_shape->setSweptSphereRadius(computeDArc(new_cast_tf, *cast_shape->getUnderlyingShape(), d_arc_scalars));
    cast_shape->updateCastTransform(new_cast_tf);
  }

  return changed;
}

void CoalCastBVHManager::collectCastTransformUpdate(Link2COW::iterator cast_it,
                                                    Link2COW::iterator reg_it,
                                                    const Eigen::Isometry3d& pose1,
                                                    const Eigen::Isometry3d& pose2)
{
  COW::Ptr& cow = cast_it->second;

  const Eigen::Isometry3d& cur_tf = cow->getCollisionObjectsTransform();

  // Publish the regular object before the early returns below: for a static link it is what static_manager_
  // holds. Its GJK generation follows whether it changed, which is what the helper reports - whether the
  // cast wrapper changed is a separate question, and one the static path never publishes.
  if (reg_it != link2cow_.end())
    collectRegularTransformUpdate(*reg_it->second, pose1);

  // Match Bullet behavior: do not update cast sweep state/AABB for disabled objects.
  // Still sync the cast COW's transform so it's correct when re-enabled.
  if (!cow->m_enabled)
  {
    // Ahead of the write: cur_tf aliases the wrapper's stored pose, so comparing after it would compare
    // pose1 against itself.
    if (transformChanged(cur_tf, pose1))
      cow->gjk_generation_++;
    cow->setCollisionObjectsTransform(pose1);
    return;
  }

  // A static link's cast wrapper is deferred: it holds the link's own geometry, on which the
  // static_cast<CastHullShape*> below would be undefined behaviour. Setting a sweep on a static link is
  // meaningless in any case.
  if (!isKinematic(*cow))
    return;

  // The sweep write is unconditional, so it leads the disjunction rather than being short-circuited away.
  if (updateCastShapeTransforms(*cow, pose1, pose2) || transformChanged(cur_tf, pose1))
    cow->gjk_generation_++;

  // Re-apply world transform so CoalCollisionObjectWrapper::updateAABB uses the
  // updated CastHullShape local AABB (swept volume).
  cow->setCollisionObjectsTransform(pose1);

  // Append to the broadphase update vector (flushed by caller).
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
