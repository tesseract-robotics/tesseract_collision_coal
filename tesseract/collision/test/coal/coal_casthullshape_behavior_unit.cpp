#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <coal/shape/geometric_shapes.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <memory>

#include <tesseract/collision/coal/coal_casthullshape.h>

namespace tesseract::collision::tesseract_collision_coal
{
TEST(CoalCastHullShapeBehaviorUnit, IsEqualAndCloneSemantics)  // NOLINT
{
  auto base_shape = std::make_shared<coal::Sphere>(0.5);
  coal::Transform3s identity = coal::Transform3s::Identity();

  CastHullShape reference(base_shape, identity);
  CastHullShape same(base_shape, identity);
  EXPECT_TRUE(reference.isEqual(same));

  coal::Transform3s translated = coal::Transform3s::Identity();
  translated.translation() = coal::Vec3s(0.25, 0.0, 0.0);
  CastHullShape different_tf(base_shape, translated);
  EXPECT_FALSE(reference.isEqual(different_tf));

  auto different_shape_ptr = std::make_shared<coal::Sphere>(0.5);
  CastHullShape different_shape(different_shape_ptr, identity);
  EXPECT_FALSE(reference.isEqual(different_shape));

  auto plain_sphere = std::make_shared<coal::Sphere>(0.5);
  EXPECT_FALSE(reference.isEqual(*plain_sphere));

  std::unique_ptr<CastHullShape> cloned(reference.clone());
  ASSERT_NE(cloned, nullptr);
  EXPECT_TRUE(cloned->isEqual(reference));
}

TEST(CoalCastHullShapeBehaviorUnit, UpdateCastTransformRefreshesStateAndAABB)  // NOLINT
{
  auto box = std::make_shared<coal::Box>(1.0, 1.0, 1.0);
  CastHullShape cast_hull(box, coal::Transform3s::Identity());

  cast_hull.computeLocalAABB();
  const coal::Scalar original_x_max = cast_hull.aabb_local.max_[0];

  coal::Transform3s moved = coal::Transform3s::Identity();
  moved.translation() = coal::Vec3s(1.0, 0.0, 0.0);
  cast_hull.updateCastTransform(moved);

  EXPECT_NEAR(cast_hull.getCastTransform().getTranslation().x(), 1.0, 1e-9);
  EXPECT_NEAR(cast_hull.getCastTransformInverse().getTranslation().x(), -1.0, 1e-9);
  EXPECT_GT(cast_hull.aabb_local.max_[0], original_x_max + 0.25);
}

TEST(CoalCastHullShapeBehaviorUnit, SupportTieBreakPrefersPoseOne)  // NOLINT
{
  auto sphere = std::make_shared<coal::Sphere>(0.5);
  coal::Transform3s cast_tf = coal::Transform3s::Identity();
  cast_tf.translation() = coal::Vec3s(0.0, 1.0, 0.0);
  CastHullShape cast_hull(sphere, cast_tf);

  coal::Vec3s dir(1.0, 0.0, 0.0);
  coal::Vec3s support;
  int hint = 0;
  coal::details::ShapeSupportData data;
  cast_hull.computeShapeSupport(dir, support, hint, data);

  // For this direction, both poses have equal projection on x, so tie-break
  // should choose pose 1 and preserve the y translation.
  EXPECT_NEAR(support.x(), 0.5, 1e-9);
  EXPECT_NEAR(support.y(), 1.0, 1e-9);
  EXPECT_NEAR(support.z(), 0.0, 1e-9);
}
}  // namespace tesseract::collision::tesseract_collision_coal

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
