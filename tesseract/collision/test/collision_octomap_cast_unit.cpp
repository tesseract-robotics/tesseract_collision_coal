#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_octomap_cast_unit.hpp>
#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/coal/coal_cast_managers.h>

using namespace tesseract::collision;

// ---- Bullet: static octree vs active cylinder ----

TEST(TesseractCollisionUnit, BulletCastSimpleCollisionOctomapCylinderUnit)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTestOctomapCylinder(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCollisionOctomapCylinderUnit)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTestOctomapCylinder(checker);
}

// ---- Bullet: static octree vs active sphere ----

TEST(TesseractCollisionUnit, BulletCastSimpleCollisionOctomapSphereUnit)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTestOctomapSphere(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCollisionOctomapSphereUnit)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTestOctomapSphere(checker);
}

// ---- Bullet: static octree vs active convex hull ----

TEST(TesseractCollisionUnit, BulletCastSimpleCollisionOctomapConvexHullUnit)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTestOctomapConvexHull(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCollisionOctomapConvexHullUnit)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTestOctomapConvexHull(checker);
}

// ---- Coal: static octree vs active shapes ----

TEST(TesseractCollisionUnit, CoalCastBVHCollisionOctomapCylinderUnit)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestOctomapCylinder(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHCollisionOctomapSphereUnit)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestOctomapSphere(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHCollisionOctomapConvexHullUnit)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestOctomapConvexHull(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
