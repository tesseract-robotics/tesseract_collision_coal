#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_multi_threaded_unit.hpp>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>

using namespace tesseract::collision;

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteSimpleCollisionMultiThreadedConvexHullUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteSimpleCollisionMultiThreadedUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteBVHCollisionMultiThreadedConvexHullUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteBVHCollisionMultiThreadedUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionMultiThreadedUnit, FCLDiscreteBVHCollisionMultiThreadedConvexHullUnit)  // NOLINT
{
  tesseract::collision::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionMultiThreadedUnit, FCLDiscreteBVHCollisionMultiThreadedUnit)  // NOLINT
{
  tesseract::collision::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionMultiThreadedUnit, CoalDiscreteBVHCollisionMultiThreadedConvexHullUnit)  // NOLINT
{
  tesseract_collision_coal::CoalDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionMultiThreadedUnit, CoalDiscreteBVHCollisionMultiThreadedUnit)  // NOLINT
{
  tesseract_collision_coal::CoalDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
