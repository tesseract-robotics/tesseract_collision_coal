#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/test_suite/collision_cast_cctype_unit.hpp>

using namespace tesseract::collision;

// ============================================================================
// Bullet — Simple cast manager
// ============================================================================

TEST(TesseractCollisionUnit, BulletCastSimpleCCTypeTime1)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTestCCTypeTime1(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleCCTypeTime0)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTestCCTypeTime0(checker);
}

// ============================================================================
// Bullet — BVH cast manager
// ============================================================================

TEST(TesseractCollisionUnit, BulletCastBVHCCTypeTime1)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestCCTypeTime1(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCCTypeTime0)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestCCTypeTime0(checker);
}

// ============================================================================
// Coal — BVH cast manager (gated on the cast-test flag)
// ============================================================================

#if defined(TESSERACT_COLLISION_COAL_ENABLE_COAL_CAST_TESTS)
TEST(TesseractCollisionUnit, CoalCastBVHCCTypeTime1)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestCCTypeTime1(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHCCTypeTime0)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestCCTypeTime0(checker);
}
#endif

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
