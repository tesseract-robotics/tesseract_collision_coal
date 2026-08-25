#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_octomap_subshape_id_unit.hpp>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>

using namespace tesseract::collision;

TEST(TesseractCollisionUnit, BulletDiscreteSimpleOctreeSubshapeIdNamesPrimitive)  // NOLINT
{
  tesseract::collision::BulletDiscreteSimpleManager checker;
  test_suite::runTestDiscreteOctreeSubshapeIdNamesPrimitive(checker, test_suite::OctreeSubshapeIdKind::LeafOrdinal);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHOctreeSubshapeIdNamesPrimitive)  // NOLINT
{
  tesseract::collision::BulletDiscreteBVHManager checker;
  test_suite::runTestDiscreteOctreeSubshapeIdNamesPrimitive(checker, test_suite::OctreeSubshapeIdKind::LeafOrdinal);
}

TEST(TesseractCollisionUnit, CoalDiscreteBVHOctreeSubshapeIdNamesPrimitive)  // NOLINT
{
  tesseract_collision_coal::CoalDiscreteBVHManager checker;
  test_suite::runTestDiscreteOctreeSubshapeIdNamesPrimitive(checker, test_suite::OctreeSubshapeIdKind::NodeHandle);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
