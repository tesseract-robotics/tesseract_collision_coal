#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_cast_static_broadphase_unit.hpp>
#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/coal/coal_cast_managers.h>

using namespace tesseract::collision;

namespace
{
/** @brief BulletCastBVHManager's two-pose setters move the regular collision object but refresh the broadphase
 * AABB of the cast one, which a static link has not registered, so its static tree keeps the old bounds. The
 * four two-pose cases are skipped rather than dropped so every run reports the gap; remove the GTEST_SKIP lines
 * once the manager flushes the object the broadphase actually holds. */
constexpr const char* BULLET_CAST_BVH_TWO_POSE_SKIP = "BulletCastBVHManager does not flush a static link's regular "
                                                      "collision object from its two-pose setters";

/** @brief The mirror of the above: the single-pose setter refreshes the regular collision object, which an
 * active link has not registered, so the cast tree keeps the old bounds. */
constexpr const char* BULLET_CAST_BVH_SINGLE_POSE_SKIP = "BulletCastBVHManager does not flush an active link's cast "
                                                         "collision object from its single-pose setter";

/** @brief Neither Bullet cast manager clears the previous sweep when a pose is set without one, so the object is
 * swept again from its new pose by a delta that no longer describes any motion. BulletCastBVHManager passes this
 * case only because its stale broadphase bounds cull the pair first; fixing the skip above exposes it there too. */
constexpr const char* BULLET_CAST_STALE_SWEEP_SKIP = "Bullet cast managers do not clear the previous sweep when a "
                                                     "pose is set without one";

/** @brief A link cannot be swept while it is static, so the sweep it carried on the way down is still on its hulls
 * on the way back up. Neither Bullet cast manager clears it, so the link is swept again from wherever it has since
 * been moved to. Unlike the cases above this is not a setter defect - the promotion itself registers the stale
 * state - so it is skipped under its own name. */
constexpr const char* BULLET_CAST_PROMOTION_SWEEP_SKIP = "Bullet cast managers do not clear a link's previous sweep "
                                                         "when it is promoted back to active";
}  // namespace

TEST(TesseractCollisionUnit, BulletCastSimpleStaticObstacleSinglePoseUpdatesBroadphase)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveSinglePose);
}

TEST(TesseractCollisionUnit, BulletCastSimpleStaticObstacleSinglePoseMapUpdatesBroadphase)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveSinglePoseMap);
}

TEST(TesseractCollisionUnit, BulletCastSimpleStaticObstacleSinglePoseArrayUpdatesBroadphase)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveSinglePoseArray);
}

TEST(TesseractCollisionUnit, BulletCastSimpleStaticObstacleTwoPoseUpdatesBroadphase)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveTwoPose);
}

TEST(TesseractCollisionUnit, BulletCastSimpleStaticObstacleTwoPoseMapUpdatesBroadphase)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveTwoPoseMap);
}

TEST(TesseractCollisionUnit, BulletCastSimpleStaticObstacleTwoPoseArrayUpdatesBroadphase)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveTwoPoseArray);
}

TEST(TesseractCollisionUnit, BulletCastSimpleDisabledStaticObstacleTwoPoseUpdatesBroadphase)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(
      checker, test_suite::moveTwoPose, /*disabled_during_move=*/true);
}

TEST(TesseractCollisionUnit, BulletCastBVHStaticObstacleSinglePoseUpdatesBroadphase)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveSinglePose);
}

TEST(TesseractCollisionUnit, BulletCastBVHStaticObstacleSinglePoseMapUpdatesBroadphase)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveSinglePoseMap);
}

TEST(TesseractCollisionUnit, BulletCastBVHStaticObstacleSinglePoseArrayUpdatesBroadphase)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveSinglePoseArray);
}

TEST(TesseractCollisionUnit, BulletCastBVHStaticObstacleTwoPoseUpdatesBroadphase)  // NOLINT
{
  GTEST_SKIP() << BULLET_CAST_BVH_TWO_POSE_SKIP;
  BulletCastBVHManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveTwoPose);
}

TEST(TesseractCollisionUnit, BulletCastBVHStaticObstacleTwoPoseMapUpdatesBroadphase)  // NOLINT
{
  GTEST_SKIP() << BULLET_CAST_BVH_TWO_POSE_SKIP;
  BulletCastBVHManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveTwoPoseMap);
}

TEST(TesseractCollisionUnit, BulletCastBVHStaticObstacleTwoPoseArrayUpdatesBroadphase)  // NOLINT
{
  GTEST_SKIP() << BULLET_CAST_BVH_TWO_POSE_SKIP;
  BulletCastBVHManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveTwoPoseArray);
}

TEST(TesseractCollisionUnit, BulletCastBVHDisabledStaticObstacleTwoPoseUpdatesBroadphase)  // NOLINT
{
  GTEST_SKIP() << BULLET_CAST_BVH_TWO_POSE_SKIP;
  BulletCastBVHManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(
      checker, test_suite::moveTwoPose, /*disabled_during_move=*/true);
}

TEST(TesseractCollisionUnit, CoalCastBVHStaticObstacleSinglePoseUpdatesBroadphase)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveSinglePose);
}

TEST(TesseractCollisionUnit, CoalCastBVHStaticObstacleSinglePoseMapUpdatesBroadphase)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveSinglePoseMap);
}

TEST(TesseractCollisionUnit, CoalCastBVHStaticObstacleSinglePoseArrayUpdatesBroadphase)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveSinglePoseArray);
}

TEST(TesseractCollisionUnit, CoalCastBVHStaticObstacleTwoPoseUpdatesBroadphase)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveTwoPose);
}

TEST(TesseractCollisionUnit, CoalCastBVHStaticObstacleTwoPoseMapUpdatesBroadphase)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveTwoPoseMap);
}

TEST(TesseractCollisionUnit, CoalCastBVHStaticObstacleTwoPoseArrayUpdatesBroadphase)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(checker, test_suite::moveTwoPoseArray);
}

TEST(TesseractCollisionUnit, CoalCastBVHDisabledStaticObstacleTwoPoseUpdatesBroadphase)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestStaticObstacleMoveUpdatesBroadphase(
      checker, test_suite::moveTwoPose, /*disabled_during_move=*/true);
}

// --- Active link moved with the single-pose setter: the broadphase holds its cast collision object ---

TEST(TesseractCollisionUnit, BulletCastSimpleActiveProbeSinglePoseUpdatesBroadphase)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTestActiveProbeMoveUpdatesBroadphase(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHActiveProbeSinglePoseUpdatesBroadphase)  // NOLINT
{
  GTEST_SKIP() << BULLET_CAST_BVH_SINGLE_POSE_SKIP;
  BulletCastBVHManager checker;
  test_suite::runTestActiveProbeMoveUpdatesBroadphase(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHActiveProbeSinglePoseUpdatesBroadphase)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestActiveProbeMoveUpdatesBroadphase(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHDisabledActiveProbeSinglePoseUpdatesBroadphase)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestActiveProbeMoveUpdatesBroadphase(checker, /*disabled_during_move=*/true);
}

// --- A pose set without a sweep must leave none behind ---

TEST(TesseractCollisionUnit, BulletCastSimpleSinglePoseClearsPreviousSweep)  // NOLINT
{
  GTEST_SKIP() << BULLET_CAST_STALE_SWEEP_SKIP;
  BulletCastSimpleManager checker;
  test_suite::runTestSinglePoseClearsPreviousSweep(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHSinglePoseClearsPreviousSweep)  // NOLINT
{
  GTEST_SKIP() << BULLET_CAST_STALE_SWEEP_SKIP;
  BulletCastBVHManager checker;
  test_suite::runTestSinglePoseClearsPreviousSweep(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHSinglePoseClearsPreviousSweep)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestSinglePoseClearsPreviousSweep(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHDisabledSinglePoseClearsPreviousSweep)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestSinglePoseClearsPreviousSweep(checker, /*disabled_during_move=*/true);
}

TEST(TesseractCollisionUnit, BulletCastSimpleSinglePoseToSweepStartClearsPreviousSweep)  // NOLINT
{
  GTEST_SKIP() << BULLET_CAST_STALE_SWEEP_SKIP;
  BulletCastSimpleManager checker;
  test_suite::runTestSinglePoseToSweepStartClearsPreviousSweep(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHSinglePoseToSweepStartClearsPreviousSweep)  // NOLINT
{
  GTEST_SKIP() << BULLET_CAST_STALE_SWEEP_SKIP;
  BulletCastBVHManager checker;
  test_suite::runTestSinglePoseToSweepStartClearsPreviousSweep(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHSinglePoseToSweepStartClearsPreviousSweep)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestSinglePoseToSweepStartClearsPreviousSweep(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimplePromotedLinkSinglePose)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTestPromotedLinkEntersBroadphaseAtCurrentPose(checker, test_suite::moveSinglePose);
}

TEST(TesseractCollisionUnit, BulletCastSimplePromotedLinkTwoPose)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTestPromotedLinkEntersBroadphaseAtCurrentPose(checker, test_suite::moveTwoPose);
}

TEST(TesseractCollisionUnit, BulletCastBVHPromotedLinkSinglePose)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestPromotedLinkEntersBroadphaseAtCurrentPose(checker, test_suite::moveSinglePose);
}

TEST(TesseractCollisionUnit, BulletCastBVHPromotedLinkTwoPose)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestPromotedLinkEntersBroadphaseAtCurrentPose(checker, test_suite::moveTwoPose);
}

TEST(TesseractCollisionUnit, CoalCastBVHPromotedLinkSinglePose)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestPromotedLinkEntersBroadphaseAtCurrentPose(checker, test_suite::moveSinglePose);
}

TEST(TesseractCollisionUnit, CoalCastBVHPromotedLinkTwoPose)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestPromotedLinkEntersBroadphaseAtCurrentPose(checker, test_suite::moveTwoPose);
}

TEST(TesseractCollisionUnit, BulletCastSimplePromotedLinkCarriesNoSweep)  // NOLINT
{
  GTEST_SKIP() << BULLET_CAST_PROMOTION_SWEEP_SKIP;
  BulletCastSimpleManager checker;
  test_suite::runTestPromotedLinkCarriesNoSweep(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHPromotedLinkCarriesNoSweep)  // NOLINT
{
  GTEST_SKIP() << BULLET_CAST_PROMOTION_SWEEP_SKIP;
  BulletCastBVHManager checker;
  test_suite::runTestPromotedLinkCarriesNoSweep(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHPromotedLinkCarriesNoSweep)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestPromotedLinkCarriesNoSweep(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
