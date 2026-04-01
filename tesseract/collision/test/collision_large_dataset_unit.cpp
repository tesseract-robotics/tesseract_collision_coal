#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_large_dataset_unit.hpp>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>

using namespace tesseract::collision;

TEST(TesseractCollisionLargeDataSetUnit, BulletDiscreteSimpleCollisionLargeDataSetConvexHullUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionLargeDataSetUnit, BulletDiscreteSimpleCollisionLargeDataSetUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionLargeDataSetUnit, BulletDiscreteBVHCollisionLargeDataSetConvexHullUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionLargeDataSetUnit, BulletDiscreteBVHCollisionLargeDataSetUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionLargeDataSetUnit, FCLDiscreteBVHCollisionLargeDataSetConvexHullUnit)  // NOLINT
{
  tesseract::collision::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionLargeDataSetUnit, FCLDiscreteBVHCollisionLargeDataSetUnit)  // NOLINT
{
  tesseract::collision::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionLargeDataSetUnit, CoalDiscreteBVHCollisionLargeDataSetConvexHullUnit)  // NOLINT
{
  tesseract_collision_coal::CoalDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionLargeDataSetUnit, CoalDiscreteBVHCollisionLargeDataSetUnit)  // NOLINT
{
  tesseract_collision_coal::CoalDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
