/**
 * @file coal_discrete_managers.h
 * @brief Tesseract Coal contact checker implementation.
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

#ifndef TESSERACT_COLLISION_COAL_DISCRETE_MANAGERS_H
#define TESSERACT_COLLISION_COAL_DISCRETE_MANAGERS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <unordered_set>
#include <coal/broadphase/broadphase_collision_manager.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/coal/coal_utils.h>

namespace tesseract::collision::tesseract_collision_coal
{
/** @brief A Coal implementation of the discrete contact manager */
class CoalDiscreteBVHManager : public DiscreteContactManager
{
public:
  using Ptr = std::shared_ptr<CoalDiscreteBVHManager>;
  using ConstPtr = std::shared_ptr<const CoalDiscreteBVHManager>;
  using UPtr = std::unique_ptr<CoalDiscreteBVHManager>;
  using ConstUPtr = std::unique_ptr<const CoalDiscreteBVHManager>;

  explicit CoalDiscreteBVHManager(std::string name = "CoalDiscreteBVHManager");
  ~CoalDiscreteBVHManager() override = default;
  CoalDiscreteBVHManager(const CoalDiscreteBVHManager&) = delete;
  CoalDiscreteBVHManager& operator=(const CoalDiscreteBVHManager&) = delete;
  CoalDiscreteBVHManager(CoalDiscreteBVHManager&&) = delete;
  CoalDiscreteBVHManager& operator=(CoalDiscreteBVHManager&&) = delete;

  // Bring base class overloads into scope (prevents name hiding by the derived overrides)
  using DiscreteContactManager::addCollisionObject;
  using DiscreteContactManager::disableCollisionObject;
  using DiscreteContactManager::enableCollisionObject;
  using DiscreteContactManager::getCollisionObjectGeometries;
  using DiscreteContactManager::getCollisionObjectGeometriesTransforms;
  using DiscreteContactManager::hasCollisionObject;
  using DiscreteContactManager::isCollisionObjectEnabled;
  using DiscreteContactManager::removeCollisionObject;
  using DiscreteContactManager::setActiveCollisionObjects;
  using DiscreteContactManager::setCollisionObjectsTransform;

  std::string getName() const override final;

  DiscreteContactManager::UPtr clone() const override final;

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

  void setCollisionObjectsTransform(const std::vector<tesseract::common::LinkId>& ids,
                                    const tesseract::common::VectorIsometry3d& poses) override final;

  const std::vector<tesseract::common::LinkId>& getCollisionObjects() const override final;

  void setActiveCollisionObjects(const std::unordered_set<tesseract::common::LinkId>& ids) override final;

  const std::unordered_set<tesseract::common::LinkId>& getActiveCollisionObjects() const override final;

  void setCollisionMarginData(CollisionMarginData collision_margin_data) override final;

  const CollisionMarginData& getCollisionMarginData() const override final;

  void setCollisionMarginPairData(
      const CollisionMarginPairData& pair_margin_data,
      CollisionMarginPairOverrideType override_type = CollisionMarginPairOverrideType::REPLACE) override final;

  void setDefaultCollisionMargin(double default_collision_margin) override final;

  void setCollisionMarginPair(const tesseract::common::LinkId& id1,
                              const tesseract::common::LinkId& id2,
                              double collision_margin) override final;

  void incrementCollisionMargin(double increment) override final;

  void setContactAllowedValidator(
      std::shared_ptr<const tesseract::common::ContactAllowedValidator> validator) override final;

  std::shared_ptr<const tesseract::common::ContactAllowedValidator> getContactAllowedValidator() const override final;

  void contactTest(ContactResultMap& collisions, const ContactRequest& request) override final;

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

  Link2COW link2cow_; /**< @brief A map of all (static and active) collision objects being managed */
  std::unordered_set<tesseract::common::LinkId> active_;     /**< @brief A list of the active collision objects */
  std::vector<tesseract::common::LinkId> collision_objects_; /**< @brief A list of the collision objects */
  ContactTestDataWrapper contact_test_data_; /**< @brief Persistent contact test data (Bullet pattern) */
  std::size_t coal_co_count_{ 0 };           /**< @brief The number of Coal collision objects */

  /** @brief This is used to store static collision objects to update */
  std::vector<CollisionObjectRawPtr> static_update_;

  /** @brief This is used to store dynamic collision objects to update */
  std::vector<CollisionObjectRawPtr> dynamic_update_;

  /** @brief Collect a single link's transform update into the batch update vectors */
  void collectTransformUpdate(Link2COW::iterator it, const Eigen::Isometry3d& pose);

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
#endif  // TESSERACT_COLLISION_COAL_DISCRETE_MANAGERS_H
