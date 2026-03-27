#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_multi_shape_cast_unit.hpp>
#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>

using namespace tesseract::collision;

// Discrete multi-shape tests (verify basic multi-shape handling)

TEST(TesseractCollisionUnit, BulletDiscreteSimpleMultiShapeUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteSimpleManager checker;
  test_suite::runTestDiscrete(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHMultiShapeUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteBVHManager checker;
  test_suite::runTestDiscrete(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHMultiShapeUnit)  // NOLINT
{
  tesseract::collision::FCLDiscreteBVHManager checker;
  test_suite::runTestDiscrete(checker);
}

TEST(TesseractCollisionUnit, CoalDiscreteBVHMultiShapeUnit)  // NOLINT
{
  tesseract_collision_coal::CoalDiscreteBVHManager checker;
  test_suite::runTestDiscrete(checker);
}

// Continuous multi-shape cast tests (verify per-child relative transform)

TEST(TesseractCollisionUnit, BulletCastSimpleMultiShapeCastUnit)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTestContinuous(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHMultiShapeCastUnit)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTestContinuous(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHMultiShapeCastUnit)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestContinuous(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
