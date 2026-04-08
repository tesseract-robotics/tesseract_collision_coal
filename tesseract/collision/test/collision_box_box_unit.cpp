#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_box_box_unit.hpp>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>

using namespace tesseract::collision;

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionBoxBoxUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionBoxBoxConvexHullUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionBoxBoxUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionBoxBoxConvexHullUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionBoxBoxUnit)  // NOLINT
{
  tesseract::collision::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionBoxBoxConvexHullUnit)  // NOLINT
{
  tesseract::collision::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionUnit, CoalDiscreteBVHCollisionBoxBoxUnit)  // NOLINT
{
  tesseract_collision_coal::CoalDiscreteBVHManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, CoalDiscreteBVHCollisionBoxBoxConvexHullUnit)  // NOLINT
{
  tesseract_collision_coal::CoalDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
