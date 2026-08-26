#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <coal/shape/geometric_shapes.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_collision_object_wrapper.h>
#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/geometry/impl/box.h>
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

TEST(CoalCollisionObjectWrapperUnit, CastRebuildKeepsGeometryOwnershipAligned)  // NOLINT
{
  CollisionShapePtr box = std::make_shared<tesseract::geometry::Box>(1, 1, 1);
  CollisionShapesConst shapes{ box };
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  auto cow = std::make_shared<CollisionObjectWrapper>(tesseract::common::LinkId("link"), 0, shapes, poses);

  COW::Ptr cast_cow = makeCastCollisionObject(cow);
  const auto& objects = cast_cow->getCollisionObjects();
  const auto& geometries = cast_cow->getCoalCollisionGeometries();
  ASSERT_EQ(geometries.size(), objects.size());
  for (std::size_t i = 0; i < objects.size(); ++i)
    EXPECT_EQ(geometries[i].get(), objects[i]->collisionGeometryPtr());
}
}  // namespace tesseract::collision::tesseract_collision_coal

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
