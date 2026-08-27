#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <coal/shape/geometric_shapes.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_collision_object_wrapper.h>
#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/geometry/impl/sphere.h>

namespace tesseract::collision::tesseract_collision_coal
{
TEST(CoalCollisionObjectWrapperUnit, SourceShapeIndexFallsBackToShapeIndex)  // NOLINT
{
  auto sphere = std::make_shared<coal::Sphere>(0.1);
  CoalCollisionObjectWrapper wrapper(sphere);

  wrapper.setShapeIndex(7);
  EXPECT_EQ(wrapper.getSourceShapeIndex(), 7);

  wrapper.setSourceShapeIndex(3);
  EXPECT_EQ(wrapper.getSourceShapeIndex(), 3);
  EXPECT_EQ(wrapper.getShapeIndex(), 7);
}

TEST(CoalCollisionObjectWrapperUnit, ClonePreservesContactDistanceThreshold)  // NOLINT
{
  CollisionShapePtr sphere = std::make_shared<tesseract::geometry::Sphere>(0.25);
  CollisionShapesConst shapes{ sphere };
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  CollisionObjectWrapper cow(tesseract::common::LinkId("link"), 0, shapes, poses);
  cow.setContactDistanceThreshold(0.05);

  auto clone_cow = cow.clone();
  EXPECT_DOUBLE_EQ(clone_cow->getContactDistanceThreshold(), 0.05);
}

}  // namespace tesseract::collision::tesseract_collision_coal

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
