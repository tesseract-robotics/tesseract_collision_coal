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
 * Static octrees use the raw OcTree directly in the static broadphase.
 * Coal provides native GEOM_CUSTOM (CastHullShape) vs GEOM_OCTREE collision
 * support, so voxel expansion is only needed for active (kinematic) octrees.
 */
class CoalCastBVHManager : public ContinuousContactManager
{
public:
  using Ptr = std::shared_ptr<CoalCastBVHManager>;
  using ConstPtr = std::shared_ptr<const CoalCastBVHManager>;
  using UPtr = std::unique_ptr<CoalCastBVHManager>;
  using ConstUPtr = std::unique_ptr<const CoalCastBVHManager>;

  CoalCastBVHManager(std::string name = "CoalCastBVHManager",
                     double gjk_guess_threshold = kDefaultGJKGuessThreshold,
                     bool d_arc_compensation = kDefaultDArcCompensation);
  ~CoalCastBVHManager() override = default;
  CoalCastBVHManager(const CoalCastBVHManager&) = delete;
  CoalCastBVHManager& operator=(const CoalCastBVHManager&) = delete;
  CoalCastBVHManager(CoalCastBVHManager&&) = delete;
  CoalCastBVHManager& operator=(CoalCastBVHManager&&) = delete;

  std::string getName() const override final;

  ContinuousContactManager::UPtr clone() const override final;

  bool addCollisionObject(const std::string& name,
                          const int& mask_id,
                          const CollisionShapesConst& shapes,
                          const tesseract::common::VectorIsometry3d& shape_poses,
                          bool enabled = true) override final;

  const CollisionShapesConst& getCollisionObjectGeometries(const std::string& name) const override final;

  const tesseract::common::VectorIsometry3d&
  getCollisionObjectGeometriesTransforms(const std::string& name) const override final;

  bool hasCollisionObject(const std::string& name) const override final;

  bool removeCollisionObject(const std::string& name) override final;

  bool enableCollisionObject(const std::string& name) override final;

  bool disableCollisionObject(const std::string& name) override final;

  bool isCollisionObjectEnabled(const std::string& name) const override final;

  void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose) override final;

  void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                    const tesseract::common::VectorIsometry3d& poses) override final;

  void setCollisionObjectsTransform(const tesseract::common::TransformMap& transforms) override final;

  void setCollisionObjectsTransform(const std::string& name,
                                    const Eigen::Isometry3d& pose1,
                                    const Eigen::Isometry3d& pose2) override final;

  void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                    const tesseract::common::VectorIsometry3d& pose1,
                                    const tesseract::common::VectorIsometry3d& pose2) override final;

  void setCollisionObjectsTransform(const tesseract::common::TransformMap& pose1,
                                    const tesseract::common::TransformMap& pose2) override final;

  const std::vector<std::string>& getCollisionObjects() const override final;

  void setActiveCollisionObjects(const std::vector<std::string>& names) override final;

  const std::vector<std::string>& getActiveCollisionObjects() const override final;

  void setCollisionMarginData(CollisionMarginData collision_margin_data) override final;

  const CollisionMarginData& getCollisionMarginData() const override final;

  void setCollisionMarginPairData(
      const CollisionMarginPairData& pair_margin_data,
      CollisionMarginPairOverrideType override_type = CollisionMarginPairOverrideType::REPLACE) override final;

  void setDefaultCollisionMargin(double default_collision_margin) override final;

  void incrementCollisionMargin(double increment) override final;

  void setCollisionMarginPair(const std::string& name1,
                              const std::string& name2,
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

  /** @brief Get the cast collision object map (for testing deferred octree expansion) */
  const Link2COW& getCastCollisionObjectMap() const { return link2castcow_; }

private:
  std::string name_;

  /** @brief Broad-phase Collision Manager for static collision objects */
  std::unique_ptr<coal::BroadPhaseCollisionManager> static_manager_;

  /** @brief Broad-phase Collision Manager for active (kinematic) collision objects */
  std::unique_ptr<coal::BroadPhaseCollisionManager> dynamic_manager_;

  /** @brief Cache for collision functors and collision requests */
  CollisionCacheMap collision_cache;

  Link2COW link2cow_;                          /** @brief A map of all collision objects being managed */
  Link2COW link2castcow_;                      /** @brief A map of cast collision objects being managed. */
  std::vector<std::string> active_;            /** @brief A list of the active collision objects */
  std::vector<std::string> collision_objects_; /** @brief A list of the collision objects */
  CollisionMarginData collision_margin_data_;  /** @brief The contact distance threshold */
  std::shared_ptr<const tesseract::common::ContactAllowedValidator> validator_; /**< @brief The is allowed collision
                                                                                  function */
  std::size_t coal_co_count_{ 0 }; /**< @brief The number of coal collision objects */
  double gjk_guess_threshold_;     /**< @brief GJK guess validity threshold (meters) */
  double gjk_guess_threshold_sq_;  /**< @brief Squared GJK guess validity threshold */
  bool d_arc_compensation_;        /**< @brief When true, set CastHullShape swept-sphere radius to arc-sagitta */

  /** @brief This is used to store static collision objects to update */
  std::vector<CollisionObjectRawPtr> static_update_;

  /** @brief This is used to store dynamic collision objects to update */
  std::vector<CollisionObjectRawPtr> dynamic_update_;

  /** @brief Collect a single link's transform update into the batch update vectors.
   *  Bumps the COW's GJK generation counter if the change exceeds the validity threshold. */
  void collectTransformUpdate(Link2COW::iterator it, const Eigen::Isometry3d& pose);

  /** @brief Collect a single link's cast transform update into the batch update vectors.
   *  Updates cast shape swept volumes, world transforms, and appends to broadphase
   *  update vectors without flushing. Bumps GJK generation counters if the change
   *  exceeds the validity threshold.
   *  @param cast_it Iterator into link2castcow_ for the link to update
   *  @param reg_it Iterator into link2cow_ for the same link (may be link2cow_.end()) */
  void collectCastTransformUpdate(Link2COW::iterator cast_it,
                                  Link2COW::iterator reg_it,
                                  const Eigen::Isometry3d& pose1,
                                  const Eigen::Isometry3d& pose2);

  /** @brief Shared implementation for enableCollisionObject / disableCollisionObject */
  bool setCollisionObjectEnabled(const std::string& name, bool enabled);

  /** @brief Unregister objects from a broadphase manager and invalidate cache */
  void removeObjects(const std::vector<CollisionObjectPtr>& objects, coal::BroadPhaseCollisionManager& manager);

  /** @brief Flush accumulated batch updates to the broadphase managers */
  void flushBatchUpdate();

  /** @brief This function will update internal data when margin data has changed */
  void onCollisionMarginDataChanged();
};

}  // namespace tesseract::collision::tesseract_collision_coal

#endif  // TESSERACT_COLLISION_COAL_CAST_MANAGERS_H
