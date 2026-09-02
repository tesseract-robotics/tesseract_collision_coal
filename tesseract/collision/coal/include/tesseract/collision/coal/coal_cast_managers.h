/**
 * @file coal_cast_managers.h
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

#ifndef TESSERACT_COLLISION_COAL_CAST_MANAGERS_H
#define TESSERACT_COLLISION_COAL_CAST_MANAGERS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <unordered_set>
#include <coal/broadphase/broadphase_collision_manager.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/coal/coal_utils.h>

namespace tesseract::collision::tesseract_collision_coal
{
/**
 * @brief A Coal implementation of the continuous contact manager.
 *
 * Maintains two parallel maps: link2cow_ (regular collision objects used as
 * static targets) and link2castcow_ (CastHullShape-wrapped objects for swept
 * collision). Active/kinematic links register their cast COW in the dynamic
 * broadphase manager; inactive/static links register their regular COW in the
 * static broadphase manager.
 *
 * A static link's cast wrapper is deferred: it holds the link's own geometry and is built into
 * CastHullShapes only when the link goes kinematic. Because a static link is collided through its regular
 * wrapper, it may hold any geometry Coal can collide - a mesh, a raw octree - whether or not that geometry
 * has a swept form. Promoting a link whose geometry has no swept form throws.
 */
class CoalCastBVHManager : public ContinuousContactManager
{
public:
  using Ptr = std::shared_ptr<CoalCastBVHManager>;
  using ConstPtr = std::shared_ptr<const CoalCastBVHManager>;
  using UPtr = std::unique_ptr<CoalCastBVHManager>;
  using ConstUPtr = std::unique_ptr<const CoalCastBVHManager>;

  explicit CoalCastBVHManager(std::string name = "CoalCastBVHManager",
                              bool d_arc_compensation = kDefaultDArcCompensation);
  ~CoalCastBVHManager() override = default;
  CoalCastBVHManager(const CoalCastBVHManager&) = delete;
  CoalCastBVHManager& operator=(const CoalCastBVHManager&) = delete;
  CoalCastBVHManager(CoalCastBVHManager&&) = delete;
  CoalCastBVHManager& operator=(CoalCastBVHManager&&) = delete;

  // Bring base class overloads into scope (prevents name hiding by the derived overrides)
  using ContinuousContactManager::addCollisionObject;
  using ContinuousContactManager::disableCollisionObject;
  using ContinuousContactManager::enableCollisionObject;
  using ContinuousContactManager::getCollisionObjectGeometries;
  using ContinuousContactManager::getCollisionObjectGeometriesTransforms;
  using ContinuousContactManager::hasCollisionObject;
  using ContinuousContactManager::isCollisionObjectEnabled;
  using ContinuousContactManager::removeCollisionObject;
  using ContinuousContactManager::setActiveCollisionObjects;
  using ContinuousContactManager::setCollisionObjectsTransform;

  std::string getName() const override final;

  ContinuousContactManager::UPtr clone() const override final;

  bool addCollisionObject(const tesseract::common::LinkId& id,
                          const int& mask_id,
                          const CollisionShapesConst& shapes,
                          const tesseract::common::VectorIsometry3d& shape_poses,
                          bool enabled = true) override final;

  const CollisionShapesConst& getCollisionObjectGeometries(const tesseract::common::LinkId& id) const override final;

  const tesseract::common::VectorIsometry3d&
  getCollisionObjectGeometriesTransforms(const tesseract::common::LinkId& id) const override final;

  bool hasCollisionObject(const tesseract::common::LinkId& id) const override final;

  bool removeCollisionObject(const tesseract::common::LinkId& id) override final;

  bool enableCollisionObject(const tesseract::common::LinkId& id) override final;

  bool disableCollisionObject(const tesseract::common::LinkId& id) override final;

  bool isCollisionObjectEnabled(const tesseract::common::LinkId& id) const override final;

  void setCollisionObjectsTransform(const tesseract::common::LinkId& id, const Eigen::Isometry3d& pose) override final;

  Eigen::Isometry3d getCollisionObjectsTransform(const tesseract::common::LinkId& id) const override final;

  void setCollisionObjectsTransform(const tesseract::common::LinkIdTransformMap& transforms) override final;

  void setCollisionObjectsTransform(const tesseract::common::LinkId& id,
                                    const Eigen::Isometry3d& pose1,
                                    const Eigen::Isometry3d& pose2) override final;

  void setCollisionObjectsTransform(const tesseract::common::LinkIdTransformMap& pose1,
                                    const tesseract::common::LinkIdTransformMap& pose2) override final;

  void setCollisionObjectsTransform(const std::vector<tesseract::common::LinkId>& ids,
                                    const tesseract::common::VectorIsometry3d& poses) override final;

  void setCollisionObjectsTransform(const std::vector<tesseract::common::LinkId>& ids,
                                    const tesseract::common::VectorIsometry3d& pose1,
                                    const tesseract::common::VectorIsometry3d& pose2) override final;

  const std::vector<tesseract::common::LinkId>& getCollisionObjects() const override final;

  void setActiveCollisionObjects(const std::unordered_set<tesseract::common::LinkId>& ids) override final;

  const std::unordered_set<tesseract::common::LinkId>& getActiveCollisionObjects() const override final;

  void setCollisionMarginData(CollisionMarginData collision_margin_data) override final;

  const CollisionMarginData& getCollisionMarginData() const override final;

  void setCollisionMarginPairData(
      const CollisionMarginPairData& pair_margin_data,
      CollisionMarginPairOverrideType override_type = CollisionMarginPairOverrideType::REPLACE) override final;

  void setDefaultCollisionMargin(double default_collision_margin) override final;

  void incrementCollisionMargin(double increment) override final;

  void setCollisionMarginPair(const tesseract::common::LinkId& id1,
                              const tesseract::common::LinkId& id2,
                              double collision_margin) override final;

  void setContactAllowedValidator(
      std::shared_ptr<const tesseract::common::ContactAllowedValidator> validator) override final;

  std::shared_ptr<const tesseract::common::ContactAllowedValidator> getContactAllowedValidator() const override final;

  void contactTest(ContactResultMap& collisions, const ContactRequest& request) override final;

  /**
   * @brief Add a Coal collision object to the manager
   * @param cow The tesseract Coal collision object
   */
  void addCollisionObject(const COW::Ptr& cow);

  /**
   * @brief Bulk-add collision objects using balanced tree construction.
   * @param cows Collision objects to add. The manager adopts this order: it is the order the
   *             objects are registered with the broadphase and the order getCollisionObjects()
   *             reports, so a caller reproducing another manager's contents must supply them in
   *             that manager's order.
   * @param defer_update When true, skips update()/filter/cache operations — caller is responsible
   *                     for calling setActiveCollisionObjects or similar before querying.
   */
  void addCollisionObjects(const std::vector<COW::Ptr>& cows, bool defer_update = false);

  /** @brief Get the cast collision object map (for testing deferred cast shape construction) */
  const Link2COW& getCastCollisionObjectMap() const { return link2castcow_; }

private:
  std::string name_;

  /** @brief Broad-phase Collision Manager for static collision objects */
  std::unique_ptr<coal::BroadPhaseCollisionManager> static_manager_;

  /** @brief Broad-phase Collision Manager for active (kinematic) collision objects */
  std::unique_ptr<coal::BroadPhaseCollisionManager> dynamic_manager_;

  /** @brief Cache for collision functors and collision requests */
  CollisionCacheMap collision_cache;

  Link2COW link2cow_;                                    /** @brief A map of all collision objects being managed */
  Link2COW link2castcow_;                                /** @brief A map of cast collision objects being managed. */
  std::unordered_set<tesseract::common::LinkId> active_; /** @brief A list of the active collision objects */
  std::vector<tesseract::common::LinkId> collision_objects_; /** @brief A list of the collision objects */
  ContactTestDataWrapper contact_test_data_; /**< @brief Persistent contact test data (Bullet pattern) */
  std::size_t coal_co_count_{ 0 };           /**< @brief The number of coal collision objects */
  bool d_arc_compensation_; /**< @brief When true, set CastHullShape swept-sphere radius to arc-sagitta */

  /** @brief This is used to store static collision objects to update */
  std::vector<CollisionObjectRawPtr> static_update_;

  /** @brief This is used to store dynamic collision objects to update */
  std::vector<CollisionObjectRawPtr> dynamic_update_;

  /** @brief Append a regular collision object wrapper to the static batch update vector.
   *
   *  static_manager_ holds the regular wrapper of a static link, dynamic_manager_ the cast wrapper of a
   *  kinematic one, and updateCollisionObjectFilters keeps both wrappers' filter groups equal. Appending a
   *  wrapper the broadphase does not hold would flush an object no tree contains. */
  void appendRegularBroadphaseUpdate(COW& reg_cow);

  /** @brief Append a cast collision object wrapper to the dynamic batch update vector.
   *  @see appendRegularBroadphaseUpdate for the registration rule both helpers encode. */
  void appendCastBroadphaseUpdate(COW& cast_cow);

  /** @brief Publish a link's new pose to its regular wrapper, and to the broadphase if it holds it.
   *
   *  The regular wrapper carries no sweep, so it has nothing to publish when the link has not moved. Its
   *  stored pose is also the comparison, so writing a change too small to publish would keep sub-epsilon
   *  moves from ever accumulating into one.
   *
   *  @return Whether anything was rewritten, which is also what the caller's cast wrapper needs to know:
   *          whether the link moved is a property of the link, not of either wrapper. */
  bool collectRegularTransformUpdate(COW& reg_cow, const Eigen::Isometry3d& pose);

  /** @brief Write the sweep from @p pose1 to @p pose2 into a cast wrapper's hulls.
   *
   *  Only a kinematic link's cast wrapper may be passed. A static link's is deferred and still holds the
   *  link's own geometry, on which the static_cast below is undefined behaviour. Callers check the filter
   *  group; the assertion here records that they must.
   *
   *  pose1 == pose2 is a zero-length sweep, which is the unswept state every hull resolves to regardless of
   *  its local offset, so that case defers to CastHullShape::clearSweep rather than computing it per shape.
   *
   *  @return Whether any hull was rewritten. A hull that changes while the link stays put still has to reach
   *          the broadphase, so a change here is not implied by the link having moved. */
  bool updateCastShapeTransforms(COW& cast_cow, const Eigen::Isometry3d& pose1, const Eigen::Isometry3d& pose2) const;

  /** @brief Collect a single link's transform update into the batch update vectors.
   *  Bumps the COW's GJK generation counter whenever anything was rewritten. */
  void collectTransformUpdate(Link2COW::iterator it, const Eigen::Isometry3d& pose);

  /** @brief Collect a single link's cast transform update into the batch update vectors.
   *  Updates cast shape swept volumes, world transforms, and appends to broadphase
   *  update vectors without flushing. Bumps GJK generation counters whenever anything
   *  was rewritten.
   *  @param cast_it Iterator into link2castcow_ for the link to update
   *  @param reg_it Iterator into link2cow_ for the same link (may be link2cow_.end()) */
  void collectCastTransformUpdate(Link2COW::iterator cast_it,
                                  Link2COW::iterator reg_it,
                                  const Eigen::Isometry3d& pose1,
                                  const Eigen::Isometry3d& pose2);

  /** @brief Shared implementation for enableCollisionObject / disableCollisionObject */
  bool setCollisionObjectEnabled(const tesseract::common::LinkId& id, bool enabled);

  /** @brief Flush accumulated batch updates to the broadphase managers */
  void flushBatchUpdate();

  /** @brief Update broadphase trees and reserve collision cache */
  void updateBroadphaseAndCache();

  /** @brief This function will update internal data when margin data has changed */
  void onCollisionMarginDataChanged();
};

}  // namespace tesseract::collision::tesseract_collision_coal

#endif  // TESSERACT_COLLISION_COAL_CAST_MANAGERS_H
