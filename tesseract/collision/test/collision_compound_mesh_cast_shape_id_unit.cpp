#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_compound_mesh_cast_shape_id_unit.hpp>
#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/coal/coal_cast_managers.h>

using namespace tesseract::collision;

TEST(TesseractCollisionUnit, BulletCastSimpleCompoundMeshShapeIdUsesOriginalGeometryIndex)  // NOLINT
{
  tesseract::collision::BulletCastSimpleManager checker;
  test_suite::runTestCompoundMeshCastShapeIdUsesOriginalGeometryIndex(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCompoundMeshShapeIdUsesOriginalGeometryIndex)  // NOLINT
{
  tesseract::collision::BulletCastBVHManager checker;
  test_suite::runTestCompoundMeshCastShapeIdUsesOriginalGeometryIndex(checker);
}

#if defined(TESSERACT_COLLISION_COAL_ENABLE_COAL_CAST_TESTS)
TEST(TesseractCollisionUnit, CoalCastBVHCompoundMeshShapeIdUsesOriginalGeometryIndex)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::runTestCompoundMeshCastShapeIdUsesOriginalGeometryIndex(checker);
}
#endif

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
