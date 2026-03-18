#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_cast_scenario_unit.hpp>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/coal/coal_cast_managers.h>

using namespace tesseract::collision;

// ============================================================================
// Bullet — self-consistency tests
// ============================================================================

TEST(TesseractCollisionUnit, BulletCastBVHScenarioA_RotationalSweep)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestScenarioA_RotationalSweep(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHScenarioB_MultiShapeSweep)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestScenarioB_MultiShapeSweep(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHScenarioC_LargeMargin)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestScenarioC_LargeMargin(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHScenarioD_ArticulatedArm)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestScenarioD_ArticulatedArm(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHScenarioE_SubdivisionLoop)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestScenarioE_SubdivisionLoop(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHScenarioF_NearMiss)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestScenarioF_NearMiss(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHScenarioG_RepeatedStability)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestScenarioG_RepeatedStability(checker);
}

// ============================================================================
// Coal — self-consistency tests
// ============================================================================

#if defined(TESSERACT_COLLISION_COAL_ENABLE_COAL_CAST_TESTS)

TEST(TesseractCollisionUnit, CoalCastBVHScenarioA_RotationalSweep)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestScenarioA_RotationalSweep(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHScenarioB_MultiShapeSweep)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestScenarioB_MultiShapeSweep(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHScenarioC_LargeMargin)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestScenarioC_LargeMargin(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHScenarioD_ArticulatedArm)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestScenarioD_ArticulatedArm(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHScenarioE_SubdivisionLoop)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestScenarioE_SubdivisionLoop(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHScenarioF_NearMiss)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestScenarioF_NearMiss(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHScenarioG_RepeatedStability)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestScenarioG_RepeatedStability(checker);
}

// ============================================================================
// Cross-backend comparison tests
// ============================================================================

TEST(TesseractCollisionUnit, CoalVsBulletScenarioD_ArticulatedArm)  // NOLINT
{
  BulletCastBVHManager bullet;
  tesseract_collision_coal::CoalCastBVHManager coal;
  test_suite::runTestScenarioD_Comparison(bullet, coal, "Bullet", "Coal");
}

TEST(TesseractCollisionUnit, CoalVsBulletScenarioE_SubdivisionLoop)  // NOLINT
{
  BulletCastBVHManager bullet;
  tesseract_collision_coal::CoalCastBVHManager coal;
  test_suite::runTestScenarioE_Comparison(bullet, coal, "Bullet", "Coal");
}

#endif  // TESSERACT_COLLISION_COAL_ENABLE_COAL_CAST_TESTS

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
