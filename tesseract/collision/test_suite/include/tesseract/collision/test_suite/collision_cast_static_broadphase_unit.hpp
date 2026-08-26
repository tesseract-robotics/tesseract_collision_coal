#ifndef TESSERACT_COLLISION_COLLISION_CAST_STATIC_BROADPHASE_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_CAST_STATIC_BROADPHASE_UNIT_HPP

#include <string>
#include <vector>

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_link_fixtures.hpp>
#include <tesseract/collision/continuous_contact_manager.h>

namespace tesseract::collision::test_suite
{
namespace detail
{
/** @brief Obstacle count for the parked-obstacle scenarios.
 *  More than one, so the static broadphase holds a tree with real structure rather than a single node that a
 *  traversal reaches regardless of its bounds. */
constexpr int kParkedObstacleCount = 7;

inline tesseract::common::LinkId parkedObstacleId(int i) { return { "obstacle_" + std::to_string(i) }; }

/** @brief Where obstacle @p i is parked. The scenarios aim at an obstacle through this function rather than
 *  repeating its arithmetic, so a change to the layout cannot move the obstacles without moving the target. */
inline Eigen::Isometry3d parkedObstaclePose(int i)
{
  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  pose.translation() = Eigen::Vector3d(0, 10.0 * i, 0);
  return pose;
}

/** @brief Park kParkedObstacleCount static boxes clear of one kinematic probe, and return the probe's pose.
 *
 *  Only the probe is active, so every obstacle is a static link and the regular - not the cast - collision
 *  object is what the static broadphase holds for it.
 *
 *  The probe is placed with a zero-length sweep rather than the single-pose setter: a cast manager is not
 *  required to collide an active link whose cast hull has never been given a sweep, and the Bullet BVH manager
 *  does not. */
inline Eigen::Isometry3d buildParkedObstacleScene(ContinuousContactManager& checker,
                                                  const tesseract::common::LinkId& probe)
{
  const Eigen::Vector3d half_extents(0.5, 0.5, 0.5);

  for (int i = 0; i < kParkedObstacleCount; ++i)
    addBoxLink(checker, parkedObstacleId(i).name(), half_extents);
  addBoxLink(checker, probe.name(), half_extents);

  checker.setActiveCollisionObjects({ probe });
  checker.setDefaultCollisionMargin(0.0);

  for (int i = 0; i < kParkedObstacleCount; ++i)
    checker.setCollisionObjectsTransform(parkedObstacleId(i), parkedObstaclePose(i));

  Eigen::Isometry3d probe_pose{ Eigen::Isometry3d::Identity() };
  probe_pose.translation() = Eigen::Vector3d(3, 0, 0);
  checker.setCollisionObjectsTransform(probe, probe_pose, probe_pose);
  return probe_pose;
}
}  // namespace detail

/** @brief Move one link to @p pose through a single spelling of setCollisionObjectsTransform */
using CastMoveFn = void (*)(ContinuousContactManager&, const tesseract::common::LinkId&, const Eigen::Isometry3d&);

inline void moveSinglePose(ContinuousContactManager& checker,
                           const tesseract::common::LinkId& id,
                           const Eigen::Isometry3d& pose)
{
  checker.setCollisionObjectsTransform(id, pose);
}

inline void moveSinglePoseMap(ContinuousContactManager& checker,
                              const tesseract::common::LinkId& id,
                              const Eigen::Isometry3d& pose)
{
  checker.setCollisionObjectsTransform(tesseract::common::LinkIdTransformMap{ { id, pose } });
}

inline void moveSinglePoseArray(ContinuousContactManager& checker,
                                const tesseract::common::LinkId& id,
                                const Eigen::Isometry3d& pose)
{
  checker.setCollisionObjectsTransform(std::vector<tesseract::common::LinkId>{ id },
                                       tesseract::common::VectorIsometry3d{ pose });
}

inline void moveTwoPose(ContinuousContactManager& checker,
                        const tesseract::common::LinkId& id,
                        const Eigen::Isometry3d& pose)
{
  checker.setCollisionObjectsTransform(id, pose, pose);
}

inline void moveTwoPoseMap(ContinuousContactManager& checker,
                           const tesseract::common::LinkId& id,
                           const Eigen::Isometry3d& pose)
{
  const tesseract::common::LinkIdTransformMap transforms{ { id, pose } };
  checker.setCollisionObjectsTransform(transforms, transforms);
}

inline void moveTwoPoseArray(ContinuousContactManager& checker,
                             const tesseract::common::LinkId& id,
                             const Eigen::Isometry3d& pose)
{
  const std::vector<tesseract::common::LinkId> ids{ id };
  const tesseract::common::VectorIsometry3d poses{ pose };
  checker.setCollisionObjectsTransform(ids, poses, poses);
}

/**
 * @brief Park static boxes clear of a kinematic probe, then move one onto the probe with @p move_obstacle.
 *
 * A static link carries no sweep, but every setter still changes its pose and must flush that change to the
 * broadphase. The first contactTest builds the trees and contactTest never refits them itself, so an obstacle
 * whose new pose does not reach the broadphase stays culled at the pose it was parked at, even though
 * getCollisionObjectsTransform reports the new one. The single-pose setter is the control: every spelling moves
 * the same obstacle onto the same probe and must agree on the contact.
 *
 * With @p disabled_during_move the obstacle is disabled across the move and enabled again before the query.
 * Disabling only suspends narrowphase reporting and enabling refits nothing, so the pose set while disabled
 * must have reached the broadphase all the same.
 */
inline void runTestStaticObstacleMoveUpdatesBroadphase(ContinuousContactManager& checker,
                                                       CastMoveFn move_obstacle,
                                                       bool disabled_during_move = false)
{
  const tesseract::common::LinkId probe("probe");
  const Eigen::Isometry3d probe_pose = detail::buildParkedObstacleScene(checker, probe);

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  ASSERT_TRUE(result.empty());

  const tesseract::common::LinkId moved = detail::parkedObstacleId(detail::kParkedObstacleCount / 2);
  if (disabled_during_move)
    checker.disableCollisionObject(moved);
  move_obstacle(checker, moved, probe_pose);
  if (disabled_during_move)
    checker.enableCollisionObject(moved);

  EXPECT_TRUE(checker.getCollisionObjectsTransform(moved).isApprox(probe_pose, 1e-8));

  result.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty());
}

/**
 * @brief Park static boxes clear of a kinematic probe, then move the probe onto one with the single-pose setter.
 *
 * The mirror of runTestStaticObstacleMoveUpdatesBroadphase: there the moved link is static and the broadphase
 * holds its regular collision object, here it is active and the broadphase holds its cast one. A setter that
 * refreshes a fixed wrapper rather than the registered one refreshes nothing in exactly one of the two cases,
 * because the wrapper it picks has no broadphase handle and the refresh is guarded on that handle.
 */
inline void runTestActiveProbeMoveUpdatesBroadphase(ContinuousContactManager& checker,
                                                    bool disabled_during_move = false)
{
  const tesseract::common::LinkId probe("probe");
  detail::buildParkedObstacleScene(checker, probe);

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  ASSERT_TRUE(result.empty());

  const Eigen::Isometry3d target = detail::parkedObstaclePose(detail::kParkedObstacleCount / 2);
  if (disabled_during_move)
    checker.disableCollisionObject(probe);
  checker.setCollisionObjectsTransform(probe, target);
  if (disabled_during_move)
    checker.enableCollisionObject(probe);

  EXPECT_TRUE(checker.getCollisionObjectsTransform(probe).isApprox(target, 1e-8));

  result.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty());
}

/**
 * @brief Sweep a probe, then reposition it with the single-pose setter and confirm the sweep does not follow it.
 *
 * The single-pose setter carries no sweep, so it must leave none behind. A cast hull that keeps the delta of
 * the previous dual-pose call is swept again from the new pose, through space nothing crossed - a contact
 * reported where there is no object, rather than the missed contact the other scenarios cover.
 *
 * The obstacle is parked one sweep-length from where the probe is about to be put, so it can only be reached
 * by a stale delta.
 *
 * With @p disabled_during_move the probe is disabled across the repositioning and enabled again before the
 * query. A sweep written while disabled is ignored, but clearing one is not writing one: enabling refits
 * nothing, so a sweep left in place here is still applied at the next query.
 */
inline void runTestSinglePoseClearsPreviousSweep(ContinuousContactManager& checker, bool disabled_during_move = false)
{
  const tesseract::common::LinkId probe("probe");
  const tesseract::common::LinkId obstacle("obstacle");
  const Eigen::Vector3d half_extents(0.5, 0.5, 0.5);

  detail::addBoxLink(checker, probe.name(), half_extents);
  detail::addBoxLink(checker, obstacle.name(), half_extents);
  checker.setActiveCollisionObjects({ probe });
  checker.setDefaultCollisionMargin(0.0);

  const Eigen::Vector3d sweep_start(0, 0, 0);
  const Eigen::Vector3d sweep_end(0, 0, 20);
  const Eigen::Vector3d parked(50, 0, 0);

  Eigen::Isometry3d obstacle_pose{ Eigen::Isometry3d::Identity() };
  obstacle_pose.translation() = parked + (sweep_end - sweep_start);
  checker.setCollisionObjectsTransform(obstacle, obstacle_pose);

  Eigen::Isometry3d p0{ Eigen::Isometry3d::Identity() };
  p0.translation() = sweep_start;
  Eigen::Isometry3d p1{ Eigen::Isometry3d::Identity() };
  p1.translation() = sweep_end;
  checker.setCollisionObjectsTransform(probe, p0, p1);

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  ASSERT_TRUE(result.empty());

  Eigen::Isometry3d parked_pose{ Eigen::Isometry3d::Identity() };
  parked_pose.translation() = parked;
  if (disabled_during_move)
    checker.disableCollisionObject(probe);
  checker.setCollisionObjectsTransform(probe, parked_pose);
  if (disabled_during_move)
    checker.enableCollisionObject(probe);

  result.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(result.empty()) << "the probe still carries the previous sweep and reaches an obstacle "
                              << (sweep_end - sweep_start).norm() << " m away";

  // Both assertions above pass on a scene that never collides at all, so sweep the probe onto the obstacle
  // deliberately: the absence of contact is only evidence once presence has been shown on the same fixture.
  Eigen::Isometry3d reach{ Eigen::Isometry3d::Identity() };
  reach.translation() = obstacle_pose.translation();
  checker.setCollisionObjectsTransform(probe, parked_pose, reach);

  result.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty()) << "the fixture never reports contact, so the checks above prove nothing";
}

/**
 * @brief Sweep a probe onto an obstacle, then set it back to the sweep's own start pose.
 *
 * The companion of runTestSinglePoseClearsPreviousSweep, aimed at the one pose the world-transform comparison
 * cannot see: the setter is handed the pose the wrapper already stores, so a manager that decides whether there
 * is anything to do by comparing world poses does nothing at all and the sweep survives. The probe is then a
 * stationary box one sweep-length from the obstacle that must no longer be reported against it.
 */
inline void runTestSinglePoseToSweepStartClearsPreviousSweep(ContinuousContactManager& checker)
{
  const tesseract::common::LinkId probe("probe");
  const tesseract::common::LinkId obstacle("obstacle");
  const Eigen::Vector3d half_extents(0.5, 0.5, 0.5);

  detail::addBoxLink(checker, probe.name(), half_extents);
  detail::addBoxLink(checker, obstacle.name(), half_extents);
  checker.setActiveCollisionObjects({ probe });
  checker.setDefaultCollisionMargin(0.0);

  Eigen::Isometry3d obstacle_pose{ Eigen::Isometry3d::Identity() };
  obstacle_pose.translation() = Eigen::Vector3d(0, 0, 20);
  checker.setCollisionObjectsTransform(obstacle, obstacle_pose);

  Eigen::Isometry3d p0{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d p1{ Eigen::Isometry3d::Identity() };
  p1.translation() = obstacle_pose.translation();

  checker.setCollisionObjectsTransform(probe, p0, p1);

  // The sweep reaches the obstacle, which is also this fixture's proof that it can report contact at all.
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  ASSERT_FALSE(result.empty());

  checker.setCollisionObjectsTransform(probe, p0);

  result.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(result.empty()) << "the probe is parked at the start of a sweep it no longer carries";
}

/**
 * @brief Move a static link onto an active one, then make the moved link active and query.
 *
 * A cast manager holds one wrapper per link in the broadphase and swaps which one when a link changes between
 * static and active. The wrapper that is not held still has to carry the link's current pose, because the swap
 * registers it as it stands and nothing repositions it afterwards - so a setter that only refreshes the held
 * wrapper hands the broadphase a link at wherever it used to be.
 *
 * The single-pose spelling is the control: both wrappers are the same box at the same pose, so whichever one
 * the promotion registers, the answer must not depend on how the link was moved.
 */
inline void runTestPromotedLinkEntersBroadphaseAtCurrentPose(ContinuousContactManager& checker, CastMoveFn move_link)
{
  const tesseract::common::LinkId mover("mover");
  const tesseract::common::LinkId anchor("anchor");
  const Eigen::Vector3d half_extents(0.5, 0.5, 0.5);

  detail::addBoxLink(checker, mover.name(), half_extents);
  detail::addBoxLink(checker, anchor.name(), half_extents);
  checker.setActiveCollisionObjects({ anchor });
  checker.setDefaultCollisionMargin(0.0);

  // Deliberately not the identity: a cast wrapper is built at the identity, so a scenario that promotes a
  // link onto the identity cannot tell a synced wrapper from an untouched one.
  Eigen::Isometry3d anchor_pose{ Eigen::Isometry3d::Identity() };
  anchor_pose.translation() = Eigen::Vector3d(5, 0, 0);
  checker.setCollisionObjectsTransform(anchor, anchor_pose, anchor_pose);

  Eigen::Isometry3d parked{ Eigen::Isometry3d::Identity() };
  parked.translation() = Eigen::Vector3d(0, 10, 0);
  checker.setCollisionObjectsTransform(mover, parked);

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  ASSERT_TRUE(result.empty());

  move_link(checker, mover, anchor_pose);
  EXPECT_TRUE(checker.getCollisionObjectsTransform(mover).isApprox(anchor_pose, 1e-8));

  checker.setActiveCollisionObjects({ mover });

  result.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty()) << "the promoted link entered the broadphase at the pose it was moved from";
}

/**
 * @brief Sweep an active link, make it static, move it, then make it active again.
 *
 * The other half of the state a promotion registers, and the companion of
 * runTestPromotedLinkEntersBroadphaseAtCurrentPose. A link cannot be swept while it is static, so whatever
 * sweep it carried on the way down is still on its hulls on the way back up - and applying that sweep from
 * wherever the link has since been moved to sends it through space nothing crossed. A link that has been
 * static has not been swept, so it must come back carrying none.
 *
 * The far obstacle sits one sweep-length beyond where the link is parked, reachable only by a sweep the link
 * should no longer have. The near one is where the original sweep legitimately ended.
 */
inline void runTestPromotedLinkCarriesNoSweep(ContinuousContactManager& checker)
{
  const tesseract::common::LinkId mover("mover");
  const tesseract::common::LinkId anchor("anchor");
  const tesseract::common::LinkId near_obstacle("near_obstacle");
  const tesseract::common::LinkId far_obstacle("far_obstacle");
  const Eigen::Vector3d half_extents(0.5, 0.5, 0.5);

  for (const auto& id : { mover, anchor, near_obstacle, far_obstacle })
    detail::addBoxLink(checker, id.name(), half_extents);
  checker.setDefaultCollisionMargin(0.0);

  const Eigen::Vector3d sweep_end(0, 0, 20);
  const Eigen::Vector3d parked(100, 0, 0);

  // The anchor only exists to hold the active set while the mover is static; it is parked out of reach.
  checker.setCollisionObjectsTransform(anchor, Eigen::Isometry3d(Eigen::Translation3d(0, -100, 0)));
  checker.setCollisionObjectsTransform(near_obstacle, Eigen::Isometry3d(Eigen::Translation3d(sweep_end)));
  checker.setCollisionObjectsTransform(far_obstacle, Eigen::Isometry3d(Eigen::Translation3d(parked + sweep_end)));

  checker.setActiveCollisionObjects({ mover });
  checker.setCollisionObjectsTransform(
      mover, Eigen::Isometry3d(Eigen::Translation3d(0, 0, 0)), Eigen::Isometry3d(Eigen::Translation3d(sweep_end)));

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  ASSERT_FALSE(result.empty());

  checker.setActiveCollisionObjects({ anchor });
  checker.setCollisionObjectsTransform(mover, Eigen::Isometry3d(Eigen::Translation3d(parked)));
  checker.setActiveCollisionObjects({ mover });

  result.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(result.empty()) << "the promoted link still carries the sweep it had before it went static";

  // The check above passes on a scene that cannot collide at all, so sweep the mover onto the far obstacle
  // deliberately: absence of contact is only evidence once presence has been shown after the promotion.
  checker.setCollisionObjectsTransform(mover,
                                       Eigen::Isometry3d(Eigen::Translation3d(parked)),
                                       Eigen::Isometry3d(Eigen::Translation3d(parked + sweep_end)));

  result.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty()) << "the fixture reports no contact after promotion, so the check above proves "
                                  "nothing";
}

}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_CAST_STATIC_BROADPHASE_UNIT_HPP
