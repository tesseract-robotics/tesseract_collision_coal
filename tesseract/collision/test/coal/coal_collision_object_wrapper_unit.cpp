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

TEST(CoalCollisionObjectWrapperUnit, ConstructionDoesNotWriteThroughCachedGeometry)  // NOLINT
{
  CollisionShapePtr sphere = std::make_shared<tesseract::geometry::Sphere>(0.25);
  CollisionShapesConst shapes{ sphere };
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };

  CollisionObjectWrapper first(tesseract::common::LinkId("link_a"), 0, shapes, poses);
  ASSERT_EQ(first.getCollisionObjects().size(), 1U);

  // CoalCollisionGeometryCache is keyed on the tesseract geometry, so a second wrapper over the
  // same shape gets the identical coal geometry. Without that, this test cannot fail.
  CollisionGeometryPtr cached = createShapePrimitive(sphere);
  ASSERT_NE(cached, nullptr);
  ASSERT_EQ(cached.get(), first.getCollisionObjects().front()->collisionGeometryPtr());

  // Asymmetric, so the centre (1, 2, 3) is a value no recomputation could land on either. A bound
  // symmetric about the origin would have centre (0, 0, 0), which is also the sphere's own.
  const coal::AABB poison(coal::Vec3s(-9.0, -8.0, -7.0), coal::Vec3s(11.0, 12.0, 13.0));
  cached->aabb_local = poison;
  cached->aabb_center = poison.center();
  cached->aabb_radius = 42.0;

  CollisionObjectWrapper second(tesseract::common::LinkId("link_b"), 0, shapes, poses);
  ASSERT_EQ(second.getCollisionObjects().size(), 1U);

  for (int i = 0; i < 3; ++i)
  {
    EXPECT_DOUBLE_EQ(cached->aabb_local.min_[i], poison.min_[i]);
    EXPECT_DOUBLE_EQ(cached->aabb_local.max_[i], poison.max_[i]);
    EXPECT_DOUBLE_EQ(cached->aabb_center[i], poison.center()[i]);
  }
  EXPECT_DOUBLE_EQ(cached->aabb_radius, 42.0);
}

}  // namespace tesseract::collision::tesseract_collision_coal

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
