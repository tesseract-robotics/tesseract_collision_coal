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
 *
 * Under relative cast, the default (see kDefaultRelativeCast), two kinematic links are not collided hull
 * against hull, which would report a contact wherever they pass through the same space at different times.
 * Of each pair of their shapes one is collided as it stands and the other is swept by its link's motion
 * relative to the first, so links that move rigidly together are checked as an unswept pair, to rounding:
 * their relative motion is guaranteed to come out as exactly no motion only when neither link is swept. A
 * contact carries one time for both links.
 *
 * The smaller shape of a pair is the swept one, by the diagonal of its bounding box, and between shapes of
 * one size the shape of the link whose name sorts last. A link with several shapes can thus be held for one
 * of them and swept for another.
 *
 * The hull of such a pair holds both end poses exactly and, between them, the straight path of every point
 * of the swept shape as the held link sees it. That misreads a contact inside the step in two ways, as a
 * link's own hull does against a static object:
 *  - Too near, where the held shape lies in the space the hull fills between the swept shape's two
 *    orientations, or on the inside of a path that bends.
 *  - Too far, where it lies on the outside of a path that bends, by an amount that falls with the square of
 *    the step.
 * Which shape is swept decides which of the two a contact meets and how large each is, and no choice is
 * better throughout. Sweeping the smaller shape leaves the least to fill, and keeps down the part its own
 * size adds to its reach from the axis of a relative turn. It is the worse choice where the smaller shape is
 * much the farther from that axis: a small link beside a large one that turns in place is swept about the
 * large one and reads too near, where the large one swept by its own turn would not.
 *
 * How far the path bends depends on what moves the two links:
 *  - When one joint joins the two links, or one of the two does not move, the path is an arc about the axis
 *    of the relative turn. It bends by at most the swept shape's reach from that axis times
 *    `1 - cos(angle / 2)`, which is what d_arc compensation pads: that covers a reading that is too far and
 *    adds to one that is too near. The reach is the shape's size plus its distance from the axis, which is
 *    not known when the swept shape is chosen.
 *  - With several joints between the links the path is no single arc, and the padding is an estimate.
 *  - When the two links move independently, the held link's own turn bends the path as well, by about that
 *    turn times the swept shape's travel over four. The relative turn does not show this: two links turning
 *    by one angle about different axes have none. Neither the hull nor the padding covers it, so under
 *    relative cast such a pair is not conservative with compensation on.
 * Only a shorter step bounds the last two.
 *
 * With relative cast off, each link's own hull is collided against the other's, and a contact carries a
 * time per link. That reads too near wherever the two links pass through the same space at different times,
 * and too far by at most the sum of how far each link's path leaves its own hull: d_arc compensation covers
 * that for a link moved by a single joint and estimates it otherwise. With compensation on it is therefore
 * conservative for two links each moved by a single joint, which relative cast is not. It gives no such
 * guarantee where several joints move a link, as along one arm, and without compensation only a shorter step
 * bounds it.
 *
 * Either way a pair is checked only if the two links' own hulls overlap in the broadphase, and each of those
 * holds its link's two end poses in the world and the straight path between them. A link that turns can
 * therefore pass through another inside the step unseen, where its arc leaves its own hull: d_arc
 * compensation pads that hull by the arc's sagitta, and a shorter step bounds it without.
 */
class CoalCastBVHManager : public ContinuousContactManager
{
public:
  using Ptr = std::shared_ptr<CoalCastBVHManager>;
  using ConstPtr = std::shared_ptr<const CoalCastBVHManager>;
  using UPtr = std::unique_ptr<CoalCastBVHManager>;
  using ConstUPtr = std::unique_ptr<const CoalCastBVHManager>;

  explicit CoalCastBVHManager(std::string name = "CoalCastBVHManager",
                              bool d_arc_compensation = kDefaultDArcCompensation,
                              bool relative_cast = kDefaultRelativeCast);
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

  bool addCollisionObjects(const std::vector<CollisionObjectSpec>& objects) override final;

  const CollisionShapesConst& getCollisionObjectGeometries(const tesseract::common::LinkId& id) const override final;

  const tesseract::common::VectorIsometry3d&
  getCollisionObjectGeometriesTransforms(const tesseract::common::LinkId& id) const override final;

  bool hasCollisionObject(const tesseract::common::LinkId& id) const override final;

  bool removeCollisionObject(const tesseract::common::LinkId& id) override final;

  bool removeCollisionObjects(const std::vector<tesseract::common::LinkId>& ids) override final;

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

  /** @brief Get a link's cast wrapper, or nullptr when the link has none
   *
   *  @warning The returned pointer is invalidated by a subsequent addCollisionObject,
   *  addCollisionObjects, removeCollisionObject or removeCollisionObjects on the same link, and by
   *  setActiveCollisionObjects when it promotes a link whose cast shapes were still deferred, since
   *  building them replaces the wrapper. Constness stops at the wrapper: its collisionGeometryPtr()
   *  still yields a mutable, shared coal::CollisionGeometry*. */
  const CastCollisionObjectWrapper* getCastCollisionObject(const tesseract::common::LinkId& id) const;

  /** @brief Get the number of entries in the narrowphase collision cache, otherwise unobservable
   *  from outside the manager; useful as a test/diagnostic hook */
  std::size_t getCollisionCacheSize() const;

private:
  /**
   * @brief Add a Coal collision object to the manager
   * @param cow The tesseract Coal collision object
   * @pre The link named by @p cow is not already present in the manager. Use the named
   *      addCollisionObject(id, mask_id, shapes, poses, enabled) overload instead when the link
   *      might already exist — it removes any existing entry for that link first.
   * @warning Adding an already-present link overwrites its map entry, destroying the previous
   *          wrapper while its collision objects may still be registered in a broadphase manager,
   *          and appends a duplicate to the collision-objects list that removeCollisionObject
   *          will not fully remove.
   */
  void addCollisionObject(const COW::Ptr& cow);

  /**
   * @brief Bulk-add collision objects using balanced tree construction.
   *
   * This is the backend-side primitive; it assumes fresh, deduplicated ids. The public
   * addCollisionObjects(const std::vector<CollisionObjectSpec>&) overload is the one that honours the
   * single-object contract.
   *
   * @param cows Collision objects to add. The manager adopts this order: it is the order the
   *             objects are registered with the broadphase and the order getCollisionObjects()
   *             reports, so a caller reproducing another manager's contents must supply them in
   *             that manager's order.
   * @param defer_update When true, skips update()/filter/cache operations — caller is responsible
   *                     for calling setActiveCollisionObjects or similar before querying.
   * @pre None of the links in @p cows are already present in the manager. Overwriting an existing
   *      link destroys its previous wrapper while that wrapper's collision objects may still be
   *      registered in a broadphase manager, and leaves a stale duplicate in the collision-objects
   *      list.
   */
  void addCollisionObjects(const std::vector<COW::Ptr>& cows, bool defer_update = false);

  std::string name_;

  /** @brief Broad-phase Collision Manager for static collision objects */
  std::unique_ptr<coal::BroadPhaseCollisionManager> static_manager_;

  /** @brief Broad-phase Collision Manager for active (kinematic) collision objects */
  std::unique_ptr<coal::BroadPhaseCollisionManager> dynamic_manager_;

  /** @brief Cache for collision functors and collision requests */
  CollisionCacheMap collision_cache;

  Link2COW link2cow_;                                    /** @brief A map of all collision objects being managed */
  Link2CastCOW link2castcow_;                            /** @brief A map of cast collision objects being managed. */
  std::unordered_set<tesseract::common::LinkId> active_; /** @brief A list of the active collision objects */
  std::vector<tesseract::common::LinkId> collision_objects_; /** @brief A list of the collision objects */
  ContactTestDataWrapper contact_test_data_; /**< @brief Persistent contact test data (Bullet pattern) */
  std::size_t coal_co_count_{ 0 };           /**< @brief The number of coal collision objects */
  /** @brief When true, pad every swept hull by an arc sagitta, as its swept-sphere radius: a link's own hull
   *  by that of the link's turn in the world, on every transform update, and under relative cast the hull of
   *  a pair of two moving links by that of their relative turn, on every narrowphase query of the pair. */
  bool d_arc_compensation_;

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
  void appendCastBroadphaseUpdate(CastCOW& cast_cow);

  /** @brief Publish a link's new pose to its regular wrapper, and to the broadphase if it holds it.
   *
   *  The regular wrapper carries no sweep, so it has nothing to publish when the link has not moved. Its
   *  stored pose is also the comparison, so writing a change too small to publish would keep sub-epsilon
   *  moves from ever accumulating into one.
   *
   *  @return Whether anything was rewritten, which is also what the caller's cast wrapper needs to know:
   *          whether the link moved is a property of the link, not of either wrapper. */
  bool collectRegularTransformUpdate(COW& reg_cow, const Eigen::Isometry3d& pose);

  /** @brief Collect a single link's transform update into the batch update vectors.
   *  Bumps the COW's GJK generation counter whenever anything was rewritten. */
  void collectTransformUpdate(Link2COW::iterator it, const Eigen::Isometry3d& pose);

  /** @brief Collect a single link's cast transform update into the batch update vectors.
   *  Updates cast shape swept volumes, world transforms, and appends to broadphase
   *  update vectors without flushing. Bumps GJK generation counters whenever anything
   *  was rewritten.
   *  @param cast_it Iterator into link2castcow_ for the link to update
   *  @param reg_it Iterator into link2cow_ for the same link (may be link2cow_.end()) */
  void collectCastTransformUpdate(Link2CastCOW::iterator cast_it,
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
