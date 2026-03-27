#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_cast_disable_during_sweep_unit.hpp>
#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/coal/coal_cast_managers.h>

using namespace tesseract::collision;

TEST(TesseractCollisionUnit, BulletCastSimpleDisabledObjectSweep)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTestDisabledObjectSweepDoesNotUpdateCastState(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHDisabledObjectSweep)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTestDisabledObjectSweepDoesNotUpdateCastState(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHDisabledObjectSweep)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestDisabledObjectSweepDoesNotUpdateCastState(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
