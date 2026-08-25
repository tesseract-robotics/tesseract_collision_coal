#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_octomap_cast_static_update_unit.hpp>
#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/coal/coal_cast_managers.h>

using namespace tesseract::collision;

TEST(TesseractCollisionUnit, BulletCastSimpleStaticOctreeCylinderContinuousTransformUpdatesBroadphase)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTestStaticOctreeCylinderContinuousTransformUpdatesBroadphase(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHStaticOctreeCylinderContinuousTransformUpdatesBroadphase)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTestStaticOctreeCylinderContinuousTransformUpdatesBroadphase(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleStaticOctreeCylinderActiveToggleStillCollides)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTestStaticOctreeCylinderActiveToggleStillCollides(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHStaticOctreeCylinderActiveToggleStillCollides)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTestStaticOctreeCylinderActiveToggleStillCollides(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleStaticOctreeCylinderShapeIdUsesOriginalGeometryIndex)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTestStaticOctreeCylinderShapeIdUsesOriginalGeometryIndex(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHStaticOctreeCylinderShapeIdUsesOriginalGeometryIndex)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTestStaticOctreeCylinderShapeIdUsesOriginalGeometryIndex(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleStaticOctreeSubshapeIdNamesPrimitive)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTestStaticOctreeSubshapeIdNamesPrimitive(checker, test_suite::OctreeSubshapeIdKind::LeafOrdinal);
}

TEST(TesseractCollisionUnit, BulletCastBVHStaticOctreeSubshapeIdNamesPrimitive)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTestStaticOctreeSubshapeIdNamesPrimitive(checker, test_suite::OctreeSubshapeIdKind::LeafOrdinal);
}

TEST(TesseractCollisionUnit, BulletCastSimpleActiveOctreeDemotionClearsSweepState)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTestActiveOctreeDemotionClearsSweepState(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHActiveOctreeDemotionClearsSweepState)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTestActiveOctreeDemotionClearsSweepState(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleActiveOctreeDisabledSweepDoesNotUpdateCastState)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTestActiveOctreeDisabledSweepDoesNotUpdateCastState(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHActiveOctreeDisabledSweepDoesNotUpdateCastState)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTestActiveOctreeDisabledSweepDoesNotUpdateCastState(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleActiveOctreeRoundTripActiveSetTransitions)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTestActiveOctreeRoundTripActiveSetTransitions(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHActiveOctreeRoundTripActiveSetTransitions)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTestActiveOctreeRoundTripActiveSetTransitions(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleActiveOctreeSubshapeIdNamesPrimitive)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTestActiveOctreeSubshapeIdNamesPrimitive(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHActiveOctreeSubshapeIdNamesPrimitive)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTestActiveOctreeSubshapeIdNamesPrimitive(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHStaticOctreeCylinderContinuousTransformUpdatesBroadphase)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestStaticOctreeCylinderContinuousTransformUpdatesBroadphase(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHStaticOctreeCylinderActiveToggleStillCollides)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestStaticOctreeCylinderActiveToggleStillCollides(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHStaticOctreeCylinderShapeIdUsesOriginalGeometryIndex)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestStaticOctreeCylinderShapeIdUsesOriginalGeometryIndex(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHStaticOctreeSubshapeIdNamesPrimitive)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestStaticOctreeSubshapeIdNamesPrimitive(checker, test_suite::OctreeSubshapeIdKind::NodeHandle);
}

TEST(TesseractCollisionUnit, CoalCastBVHActiveOctreeDemotionClearsSweepState)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestActiveOctreeDemotionClearsSweepState(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHActiveOctreeDisabledSweepDoesNotUpdateCastState)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestActiveOctreeDisabledSweepDoesNotUpdateCastState(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHActiveOctreeRoundTripActiveSetTransitions)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestActiveOctreeRoundTripActiveSetTransitions(checker);
}

TEST(TesseractCollisionUnit, CoalCastBVHActiveOctreeSubshapeIdNamesPrimitive)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestActiveOctreeSubshapeIdNamesPrimitive(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
