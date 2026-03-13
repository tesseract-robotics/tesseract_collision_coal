#ifndef TESSERACT_COLLISION_COLLISION_BOX_BOX_CAST_UNIT_H
#define TESSERACT_COLLISION_COLLISION_BOX_BOX_CAST_UNIT_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/geometry/geometries.h>

namespace tesseract::collision::test_suite
{
namespace detail
{
inline void addCollisionObjects(ContinuousContactManager& checker)
{
  ////////////////////////////
  // Add static box to checker
  ////////////////////////////
  CollisionShapePtr static_box = std::make_shared<tesseract::geometry::Box>(1, 1, 1);
  Eigen::Isometry3d static_box_pose;
  static_box_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract::common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(static_box);
  obj1_poses.push_back(static_box_pose);

  checker.addCollisionObject("static_box_link", 0, obj1_shapes, obj1_poses, false);
  checker.enableCollisionObject("static_box_link");

  /////////////////////////////////////////////
  // Add thin box to checker which is disabled
  /////////////////////////////////////////////
  CollisionShapePtr thin_box = std::make_shared<tesseract::geometry::Box>(0.1, 1, 1);
  Eigen::Isometry3d thin_box_pose;
  thin_box_pose.setIdentity();

  CollisionShapesConst obj2_shapes;
  tesseract::common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses);
  checker.disableCollisionObject("thin_box_link");

  ////////////////////////////
  // Add static box to checker
  ////////////////////////////
  CollisionShapePtr moving_box = std::make_shared<tesseract::geometry::Box>(0.25, 0.25, 0.25);
  Eigen::Isometry3d moving_box_pose;
  moving_box_pose.setIdentity();
  moving_box_pose.translation() = Eigen::Vector3d(0.5, -0.5, 0);

  CollisionShapesConst obj3_shapes;
  tesseract::common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(moving_box);
  obj3_poses.push_back(moving_box_pose);

  checker.addCollisionObject("moving_box_link", 0, obj3_shapes, obj3_poses);

  /////////////////////////////////////////////
  // Add box and remove
  /////////////////////////////////////////////
  CollisionShapePtr remove_box = std::make_shared<tesseract::geometry::Box>(0.1, 1, 1);
  Eigen::Isometry3d remove_box_pose;
  remove_box_pose.setIdentity();

  CollisionShapesConst obj4_shapes;
  tesseract::common::VectorIsometry3d obj4_poses;
  obj4_shapes.push_back(remove_box);
  obj4_poses.push_back(remove_box_pose);

  checker.addCollisionObject("remove_box_link", 0, obj4_shapes, obj4_poses);
  EXPECT_TRUE(checker.getCollisionObjects().size() == 4);
  EXPECT_TRUE(checker.hasCollisionObject("remove_box_link"));
  checker.removeCollisionObject("remove_box_link");
  EXPECT_FALSE(checker.hasCollisionObject("remove_box_link"));

  /////////////////////////////////////////////
  // Try functions on a link that does not exist
  /////////////////////////////////////////////
  EXPECT_FALSE(checker.removeCollisionObject("link_does_not_exist"));
  EXPECT_FALSE(checker.enableCollisionObject("link_does_not_exist"));
  EXPECT_FALSE(checker.disableCollisionObject("link_does_not_exist"));

  /////////////////////////////////////////////
  // Try to add empty Collision Object
  /////////////////////////////////////////////
  EXPECT_FALSE(
      checker.addCollisionObject("empty_link", 0, CollisionShapesConst(), tesseract::common::VectorIsometry3d()));

  /////////////////////////////////////////////
  // Check sizes
  /////////////////////////////////////////////
  EXPECT_TRUE(checker.getCollisionObjects().size() == 3);
  const auto& co = checker.getCollisionObjects();
  for (std::size_t i = 0; i < co.size(); ++i)
  {
    EXPECT_TRUE(checker.getCollisionObjectGeometries(co[i]).size() == 1);
    EXPECT_TRUE(checker.getCollisionObjectGeometriesTransforms(co[i]).size() == 1);
    const auto& cgt = checker.getCollisionObjectGeometriesTransforms(co[i]);
    if (i != 2)
    {
      EXPECT_TRUE(cgt[0].isApprox(Eigen::Isometry3d::Identity(), 1e-5));
    }
    else
    {
      EXPECT_TRUE(cgt[0].isApprox(moving_box_pose, 1e-5));
    }
  }
}
}  // namespace detail

inline void runTest(ContinuousContactManager& checker)
{
  // Check name which should not be empty
  EXPECT_FALSE(checker.getName().empty());

  // Add collision objects
  detail::addCollisionObjects(checker);

  // Call it again to test adding same object
  detail::addCollisionObjects(checker);

  //////////////////////////////////////
  // Test when object is inside another
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "moving_box_link", "static_box_link" });

  std::vector<std::string> active_links{ "moving_box_link" };
  checker.setActiveCollisionObjects(active_links);
  std::vector<std::string> check_active_links = checker.getActiveCollisionObjects();
  EXPECT_TRUE(tesseract::common::isIdentical<std::string>(active_links, check_active_links, false));

  EXPECT_TRUE(checker.getContactAllowedValidator() == nullptr);

  checker.setDefaultCollisionMargin(0.1);
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);

  // Set the collision object transforms
  // static_box_link: unit box at origin (identity transform)
  // moving_box_link: 0.25^3 box sweeping from (-1.9, 0, 0) to (1.9, 3.8, 0)
  std::vector<std::string> names = { "static_box_link" };
  tesseract::common::VectorIsometry3d transforms = { Eigen::Isometry3d::Identity() };
  checker.setCollisionObjectsTransform(names, transforms);

  tesseract::common::VectorIsometry3d start_poses, end_poses;
  Eigen::Isometry3d start_pos, end_pos;
  start_pos.setIdentity();
  start_pos.translation()(0) = -1.9;
  start_pos.translation()(1) = 0.0;
  end_pos.setIdentity();
  end_pos.translation()(0) = 1.9;
  end_pos.translation()(1) = 3.8;
  start_poses.push_back(start_pos);
  end_poses.push_back(end_pos);
  checker.setCollisionObjectsTransform({ "moving_box_link" }, start_poses, end_poses);

  std::vector<ContactTestType> test_types = { ContactTestType::ALL, ContactTestType::CLOSEST, ContactTestType::FIRST };
  std::vector<std::string> test_type_names = { "ALL", "CLOSEST", "FIRST" };

  // Perform collision check
  for (std::size_t ti = 0; ti < test_types.size(); ++ti)
  {
    SCOPED_TRACE("ContactTestType: " + test_type_names[ti]);

    ContactResultMap result;
    checker.contactTest(result, ContactRequest(test_types[ti]));

    ContactResultVector result_vector;
    result.flattenMoveResults(result_vector);

    ASSERT_FALSE(result_vector.empty()) << "No contacts found for moving_box sweeping from (-1.9,0,0) to (1.9,3.8,0) "
                                        << "vs static unit box at origin. The swept path passes through the static "
                                           "box, "
                                        << "so at least one contact is expected.";

    const auto& cr = result_vector[0];

    // Dump full contact state for context on any failure
    SCOPED_TRACE("Contact[0] state:"
                 "\n  link_names: [" +
                 cr.link_names[0] + ", " + cr.link_names[1] +
                 "]"
                 "\n  distance: " +
                 std::to_string(cr.distance) + "\n  normal: (" + std::to_string(cr.normal[0]) + ", " +
                 std::to_string(cr.normal[1]) + ", " + std::to_string(cr.normal[2]) +
                 ")"
                 "\n  nearest_points[0]: (" +
                 std::to_string(cr.nearest_points[0][0]) + ", " + std::to_string(cr.nearest_points[0][1]) + ", " +
                 std::to_string(cr.nearest_points[0][2]) +
                 ")"
                 "\n  nearest_points[1]: (" +
                 std::to_string(cr.nearest_points[1][0]) + ", " + std::to_string(cr.nearest_points[1][1]) + ", " +
                 std::to_string(cr.nearest_points[1][2]) +
                 ")"
                 "\n  nearest_points_local[0]: (" +
                 std::to_string(cr.nearest_points_local[0][0]) + ", " + std::to_string(cr.nearest_points_local[0][1]) +
                 ", " + std::to_string(cr.nearest_points_local[0][2]) +
                 ")"
                 "\n  nearest_points_local[1]: (" +
                 std::to_string(cr.nearest_points_local[1][0]) + ", " + std::to_string(cr.nearest_points_local[1][1]) +
                 ", " + std::to_string(cr.nearest_points_local[1][2]) +
                 ")"
                 "\n  cc_time: [" +
                 std::to_string(cr.cc_time[0]) + ", " + std::to_string(cr.cc_time[1]) +
                 "]"
                 "\n  cc_type: [" +
                 std::to_string(static_cast<int>(cr.cc_type[0])) + ", " +
                 std::to_string(static_cast<int>(cr.cc_type[1])) + "]");

    // Penetration distance: moving box overlaps static box by ~0.2475
    EXPECT_NEAR(cr.distance, -0.2475, 0.001) << "Penetration distance between static_box (1x1x1 at origin) and "
                                             << "moving_box (0.25^3 sweeping from (-1.9,0,0) to (1.9,3.8,0))";

    // cc_time[0]: static_box is not a cast shape, so cc_time should be -1 (unset)
    EXPECT_NEAR(cr.cc_time[0], -1.0, 0.001) << "static_box_link is not a cast shape, cc_time[0] should be -1 (unset)";

    // cc_time[1]: moving_box collision occurs ~25% along the sweep
    EXPECT_NEAR(cr.cc_time[1], 0.25, 0.001) << "moving_box_link cc_time[1] should be ~0.25 (collision at 25% of sweep)";

    // cc_type[0]: static_box is not cast, so CCType_None
    EXPECT_EQ(cr.cc_type[0], ContinuousCollisionType::CCType_None) << "static_box_link is not a cast shape, cc_type[0] "
                                                                      "should be CCType_None (0), "
                                                                   << "got " << static_cast<int>(cr.cc_type[0]);

    // cc_type[1]: moving_box collision is between start and end poses
    EXPECT_EQ(cr.cc_type[1], ContinuousCollisionType::CCType_Between) << "moving_box_link collision is between start "
                                                                         "and end, cc_type[1] should be CCType_Between "
                                                                         "(3), "
                                                                      << "got " << static_cast<int>(cr.cc_type[1]);

    // nearest_points[0]: contact point on static_box surface
    EXPECT_NEAR(cr.nearest_points[0][0], -0.5, 0.001) << "nearest_points[0].x: static_box surface at x=-0.5";
    EXPECT_NEAR(cr.nearest_points[0][1], 0.5, 0.001) << "nearest_points[0].y: static_box surface at y=0.5";
    EXPECT_NEAR(cr.nearest_points[0][2], 0.0, 0.001) << "nearest_points[0].z: contact at z=0 (2D sweep in XY plane)";

    // nearest_points[1]: contact point on moving_box (swept) surface
    EXPECT_NEAR(cr.nearest_points[1][0], -0.325, 0.001) << "nearest_points[1].x: moving_box swept surface contact x";
    EXPECT_NEAR(cr.nearest_points[1][1], 0.325, 0.001) << "nearest_points[1].y: moving_box swept surface contact y";
    EXPECT_NEAR(cr.nearest_points[1][2], 0.0, 0.001) << "nearest_points[1].z: contact at z=0 (2D sweep in XY plane)";

    // Verify nearest_points_local[1] maps to correct world position via start transform
    Eigen::Vector3d p0 = cr.transform[1] * cr.nearest_points_local[1];
    EXPECT_NEAR(p0[0], -1.275, 0.001) << "transform[1] * nearest_points_local[1]: world x at start pose of moving_box";
    EXPECT_NEAR(p0[1], -0.625, 0.001) << "transform[1] * nearest_points_local[1]: world y at start pose of moving_box";
    EXPECT_NEAR(p0[2], 0.0, 0.001) << "transform[1] * nearest_points_local[1]: world z at start pose of moving_box";

    // Verify nearest_points_local[1] maps to correct world position via end (cc) transform
    Eigen::Vector3d p1 = cr.cc_transform[1] * cr.nearest_points_local[1];
    EXPECT_NEAR(p1[0], 2.525, 0.001) << "cc_transform[1] * nearest_points_local[1]: world x at end pose of moving_box";
    EXPECT_NEAR(p1[1], 3.175, 0.001) << "cc_transform[1] * nearest_points_local[1]: world y at end pose of moving_box";
    EXPECT_NEAR(p1[2], 0.0, 0.001) << "cc_transform[1] * nearest_points_local[1]: world z at end pose of moving_box";
  }
}
}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_BOX_BOX_CAST_UNIT_H
