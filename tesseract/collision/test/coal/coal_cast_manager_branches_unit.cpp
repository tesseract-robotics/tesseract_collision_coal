#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <algorithm>

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/geometry/geometries.h>

using namespace tesseract::collision;
using namespace tesseract::collision::tesseract_collision_coal;

#if defined(TESSERACT_COLLISION_COAL_ENABLE_COAL_CAST_TESTS)
namespace
{
void addStaticBoxAndActiveSphere(CoalCastBVHManager& checker)
{
  CollisionShapesConst static_shapes;
  tesseract::common::VectorIsometry3d static_shape_poses;
  static_shapes.push_back(std::make_shared<tesseract::geometry::Box>(0.5, 0.5, 0.5));
  static_shape_poses.push_back(Eigen::Isometry3d::Identity());
  ASSERT_TRUE(checker.addCollisionObject("static_box", 0, static_shapes, static_shape_poses, true));

  CollisionShapesConst moving_shapes;
  tesseract::common::VectorIsometry3d moving_shape_poses;
  moving_shapes.push_back(std::make_shared<tesseract::geometry::Sphere>(0.2));
  moving_shape_poses.push_back(Eigen::Isometry3d::Identity());
  ASSERT_TRUE(checker.addCollisionObject("moving_sphere", 0, moving_shapes, moving_shape_poses, true));
}

bool hasPair(const ContactResultVector& results, const std::string& a, const std::string& b)
{
  return std::any_of(results.begin(), results.end(), [&a, &b](const ContactResult& cr) {
    return ((cr.link_names[0] == a && cr.link_names[1] == b) || (cr.link_names[0] == b && cr.link_names[1] == a));
  });
}
}  // namespace

TEST(CoalCastManagerBranchesUnit, DisabledObjectSweepDoesNotUpdateCastState)  // NOLINT
{
  CoalCastBVHManager checker;
  addStaticBoxAndActiveSphere(checker);

  checker.setActiveCollisionObjects({ "moving_sphere" });
  checker.setDefaultCollisionMargin(0.0);

  const Eigen::Isometry3d start = Eigen::Isometry3d(Eigen::Translation3d(-2.0, 0.0, 0.0));
  const Eigen::Isometry3d end = Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.0));

  checker.disableCollisionObject("moving_sphere");
  checker.setCollisionObjectsTransform("moving_sphere", start, end);
  checker.enableCollisionObject("moving_sphere");

  // Force a broadphase update at the start pose while leaving cast sweep state untouched.
  checker.setCollisionObjectsTransform("moving_sphere", start);

  ContactResultMap no_sweep_result;
  checker.contactTest(no_sweep_result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector no_sweep_vector;
  no_sweep_result.flattenMoveResults(no_sweep_vector);

  EXPECT_FALSE(hasPair(no_sweep_vector, "static_box", "moving_sphere"));

  checker.setCollisionObjectsTransform("moving_sphere", start, end);

  ContactResultMap sweep_result;
  checker.contactTest(sweep_result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector sweep_vector;
  sweep_result.flattenMoveResults(sweep_vector);

  EXPECT_TRUE(hasPair(sweep_vector, "static_box", "moving_sphere"));
}
#endif

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
