/**
 * @brief Validate Coal continuous collision results against Bullet for convex hull swept spheres.
 *
 * This test runs the same convex-hull sphere-sphere swept scenario on both Bullet and Coal,
 * then compares all contact fields. It was created to validate GJK variant choices:
 * NesterovAcceleration matches Bullet exactly, while PolyakAcceleration produces valid
 * but different contact points on tessellated geometry.
 */
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_sphere_sphere_cast_unit.hpp>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/coal/coal_cast_managers.h>

using namespace tesseract::collision;

TEST(CoalVsBulletUnit, ConvexHullSphereSphereSweptComparison)  // NOLINT
{
  auto run_scenario = [](ContinuousContactManager& checker) -> ContactResult {
    test_suite::detail::addCollisionObjects(checker, true);

    checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
    checker.setCollisionMarginData(CollisionMarginData(0.1));

    tesseract::common::TransformMap start, end;
    start["sphere_link"] = Eigen::Isometry3d::Identity();
    start["sphere_link"].translation() = Eigen::Vector3d(-0.2, -1.0, 0);
    start["sphere1_link"] = Eigen::Isometry3d::Identity();
    start["sphere1_link"].translation() = Eigen::Vector3d(0.2, 0, -1.0);
    end["sphere_link"] = Eigen::Isometry3d::Identity();
    end["sphere_link"].translation() = Eigen::Vector3d(-0.2, 1.0, 0);
    end["sphere1_link"] = Eigen::Isometry3d::Identity();
    end["sphere1_link"].translation() = Eigen::Vector3d(0.2, 0, 1.0);
    checker.setCollisionObjectsTransform(start, end);

    ContactResultMap result;
    checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
    ContactResultVector rv;
    result.flattenMoveResults(rv);
    EXPECT_FALSE(rv.empty());
    return rv.empty() ? ContactResult() : rv[0];
  };

  BulletCastBVHManager bullet;
  tesseract_collision_coal::CoalCastBVHManager coal;

  auto bullet_cr = run_scenario(bullet);
  auto coal_cr = run_scenario(coal);

  std::printf("\n=== Convex Hull Sphere-Sphere Swept Collision: Bullet vs Coal ===\n");
  std::printf("%-15s: %s\n", "Bullet", test_suite::detail::formatContactResult(bullet_cr).c_str());
  std::printf("%-15s: %s\n", "Coal", test_suite::detail::formatContactResult(coal_cr).c_str());

  // Find matching link indices
  int bi0 = (bullet_cr.link_names[0] == "sphere_link") ? 0 : 1;
  int ci0 = (coal_cr.link_names[0] == "sphere_link") ? 0 : 1;

  EXPECT_NEAR(coal_cr.distance, bullet_cr.distance, 0.01) << "Penetration depth";
  EXPECT_NEAR(coal_cr.cc_time[ci0], bullet_cr.cc_time[bi0], 0.05) << "sphere_link cc_time";
  EXPECT_NEAR(coal_cr.cc_time[1 - ci0], bullet_cr.cc_time[1 - bi0], 0.05) << "sphere1_link cc_time";
  EXPECT_EQ(coal_cr.cc_type[ci0], bullet_cr.cc_type[bi0]) << "sphere_link cc_type";
  EXPECT_EQ(coal_cr.cc_type[1 - ci0], bullet_cr.cc_type[1 - bi0]) << "sphere1_link cc_type";

  for (int d = 0; d < 3; ++d)
  {
    EXPECT_NEAR(coal_cr.nearest_points[ci0][d], bullet_cr.nearest_points[bi0][d], 0.1)
        << "sphere_link nearest_point[" << d << "]";
    EXPECT_NEAR(coal_cr.nearest_points[1 - ci0][d], bullet_cr.nearest_points[1 - bi0][d], 0.1)
        << "sphere1_link nearest_point[" << d << "]";
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
