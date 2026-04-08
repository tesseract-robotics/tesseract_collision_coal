#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_octomap_usage_unit.hpp>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>
#include <tesseract/collision/coal/coal_cast_managers.h>

using namespace tesseract::collision;

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionOctomapUsageUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteSimpleManager checker;
  test_suite::runDiscreteOctomapTransformOverloadUsageTest(checker, tesseract::geometry::OctreeSubType::BOX);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionOctomapUsageUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteBVHManager checker;
  test_suite::runDiscreteOctomapTransformOverloadUsageTest(checker, tesseract::geometry::OctreeSubType::BOX);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionOctomapUsageUnit)  // NOLINT
{
  tesseract::collision::FCLDiscreteBVHManager checker;
  test_suite::runDiscreteOctomapTransformOverloadUsageTest(checker, tesseract::geometry::OctreeSubType::BOX);
}

TEST(TesseractCollisionUnit, CoalDiscreteBVHCollisionOctomapUsageUnit)  // NOLINT
{
  tesseract_collision_coal::CoalDiscreteBVHManager checker;
  test_suite::runDiscreteOctomapTransformOverloadUsageTest(checker, tesseract::geometry::OctreeSubType::BOX);
}

TEST(TesseractCollisionUnit, BulletCastSimpleCollisionOctomapUsageUnit)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runContinuousOctomapTransformOverloadUsageTest(checker, tesseract::geometry::OctreeSubType::BOX);
}

TEST(TesseractCollisionUnit, BulletCastBVHCollisionOctomapUsageUnit)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runContinuousOctomapTransformOverloadUsageTest(checker, tesseract::geometry::OctreeSubType::BOX);
}

TEST(TesseractCollisionUnit, CoalCastBVHCollisionOctomapUsageUnit)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runContinuousOctomapTransformOverloadUsageTest(checker, tesseract::geometry::OctreeSubType::BOX);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
