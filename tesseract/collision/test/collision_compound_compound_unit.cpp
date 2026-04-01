#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_compound_compound_unit.hpp>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>

using namespace tesseract::collision;

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionCompoundCompoundUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionCompoundCompoundUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionCompoundCompoundUnit)  // NOLINT
{
  tesseract::collision::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, CoalDiscreteBVHCollisionCompoundCompoundUnit)  // NOLINT
{
  tesseract_collision_coal::CoalDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, BulletContinuousSimpleCollisionCompoundCompoundUnit)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, BulletContinuousBVHCollisionCompoundCompoundUnit)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, CoalContinuousBVHCollisionCompoundCompoundUnit)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
