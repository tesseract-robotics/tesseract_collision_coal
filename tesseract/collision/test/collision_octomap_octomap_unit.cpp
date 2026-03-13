#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_octomap_octomap_unit.hpp>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>
#include <tesseract/collision/coal/coal_cast_managers.h>

using namespace tesseract::collision;

// Expected distances for each OctreeSubType combination with two 2m octrees at x=±1.1:
//   SPHERE_OUTSIDE vs SPHERE_INSIDE: -0.0071 (slight penetration from sphere extensions)
//   BOX vs BOX:                       0.2    (exact voxel boundaries, 0.2m gap)

// ---- Bullet: SPHERE_OUTSIDE vs SPHERE_INSIDE ----

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionOctomapOctomapUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker,
                      -0.0071,
                      tesseract::geometry::OctreeSubType::SPHERE_OUTSIDE,
                      tesseract::geometry::OctreeSubType::SPHERE_INSIDE);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionOctomapOctomapUnit)  // NOLINT
{
  tesseract::collision::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker,
                      -0.0071,
                      tesseract::geometry::OctreeSubType::SPHERE_OUTSIDE,
                      tesseract::geometry::OctreeSubType::SPHERE_INSIDE);
}

TEST(TesseractCollisionUnit, BulletContinuousSimpleCollisionOctomapOctomapUnit)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTest(checker,
                      -0.0071,
                      tesseract::geometry::OctreeSubType::SPHERE_OUTSIDE,
                      tesseract::geometry::OctreeSubType::SPHERE_INSIDE);
}

TEST(TesseractCollisionUnit, BulletContinuousBVHCollisionOctomapOctomapUnit)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTest(checker,
                      -0.0071,
                      tesseract::geometry::OctreeSubType::SPHERE_OUTSIDE,
                      tesseract::geometry::OctreeSubType::SPHERE_INSIDE);
}

// ---- Coal: BOX vs BOX (only subtype supported by Coal) ----

TEST(TesseractCollisionUnit, CoalDiscreteBVHCollisionOctomapOctomapUnit)  // NOLINT
{
  tesseract_collision_coal::CoalDiscreteBVHManager checker;
  test_suite::runTest(checker,
                      0.2,
                      tesseract::geometry::OctreeSubType::BOX,
                      tesseract::geometry::OctreeSubType::BOX);
}

#if defined(TESSERACT_COLLISION_COAL_ENABLE_COAL_CAST_TESTS)
TEST(TesseractCollisionUnit, CoalContinuousBVHCollisionOctomapOctomapUnit)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTest(checker,
                      0.2,
                      tesseract::geometry::OctreeSubType::BOX,
                      tesseract::geometry::OctreeSubType::BOX);
}
#endif

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
