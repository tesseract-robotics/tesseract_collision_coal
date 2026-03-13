#ifndef TESSERACT_COLLISION_COLLISION_MULTI_SHAPE_CAST_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_MULTI_SHAPE_CAST_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iomanip>
#include <sstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/geometry/geometries.h>

namespace tesseract::collision::test_suite
{
namespace detail
{
/**
 * @brief Test continuous collision with a multi-shape link undergoing rotational motion.
 *
 * This tests the per-child relative transform computation in the cast managers.
 * A link ("arm_link") has two small boxes at different local offsets (+1,0,0) and (-1,0,0).
 * The arm rotates 90 degrees around the Z axis. A static obstacle is placed so that
 * only the swept path of one sub-shape intersects it.
 *
 * With the correct per-child relative transform, the collision is detected.
 * With a link-level relative transform (the bug), the swept volumes are incorrect
 * for sub-shapes with non-identity local offsets under rotation.
 */
inline void addMultiShapeCollisionObjects(ContinuousContactManager& checker)
{
  ////////////////////////////
  // Add static obstacle box
  ////////////////////////////
  CollisionShapePtr static_box = std::make_shared<tesseract::geometry::Box>(0.5, 0.5, 0.5);
  Eigen::Isometry3d static_box_pose;
  static_box_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract::common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(static_box);
  obj1_poses.push_back(static_box_pose);

  checker.addCollisionObject("obstacle_link", 0, obj1_shapes, obj1_poses, false);
  checker.enableCollisionObject("obstacle_link");

  ////////////////////////////////////////////////////////////////
  // Add arm link with TWO sub-shapes at different local offsets
  ////////////////////////////////////////////////////////////////
  CollisionShapePtr arm_box_a = std::make_shared<tesseract::geometry::Box>(0.2, 0.2, 0.2);
  CollisionShapePtr arm_box_b = std::make_shared<tesseract::geometry::Box>(0.2, 0.2, 0.2);

  // Sub-shape A at local offset (+1, 0, 0)
  Eigen::Isometry3d shape_pose_a;
  shape_pose_a.setIdentity();
  shape_pose_a.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  // Sub-shape B at local offset (-1, 0, 0)
  Eigen::Isometry3d shape_pose_b;
  shape_pose_b.setIdentity();
  shape_pose_b.translation() = Eigen::Vector3d(-1.0, 0.0, 0.0);

  CollisionShapesConst arm_shapes;
  tesseract::common::VectorIsometry3d arm_poses;
  arm_shapes.push_back(arm_box_a);
  arm_shapes.push_back(arm_box_b);
  arm_poses.push_back(shape_pose_a);
  arm_poses.push_back(shape_pose_b);

  checker.addCollisionObject("arm_link", 0, arm_shapes, arm_poses);
}

inline void runTestMultiShapeCast(ContinuousContactManager& checker)
{
  addMultiShapeCollisionObjects(checker);

  checker.setActiveCollisionObjects({ "arm_link" });
  checker.setDefaultCollisionMargin(0.05);

  // Place obstacle at (0.5, 0.5, 0) — along the MID-SWEEP path of sub-shape A.
  //
  // arm_link at pose1: identity (sub-shape A at (1,0,0), sub-shape B at (-1,0,0))
  // arm_link at pose2: 90 deg CCW around Z (sub-shape A at (0,1,0), sub-shape B at (0,-1,0))
  //
  // Sub-shape A sweeps linearly from (1,0,0) to (0,1,0) — midpoint is (0.5, 0.5, 0).
  // The obstacle is centered on this midpoint, so the collision happens mid-sweep (not at
  // an endpoint). This ensures cc_type is CCType_Between and cc_time is near 0.5.
  //
  // Sub-shape B sweeps from (-1,0,0) to (0,-1,0) — nowhere near obstacle.
  checker.setCollisionObjectsTransform("obstacle_link", Eigen::Isometry3d(Eigen::Translation3d(0.5, 0.5, 0.0)));

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  checker.setCollisionObjectsTransform("arm_link", pose1, pose2);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty()) << "Multi-shape rotational cast: expected collision between sub-shape A "
                                      << "(sweeping from (1,0,0) to (0,1,0)) and static obstacle at (0.5,0.5,0). "
                                      << "If no collision was found, the per-child relative transform may be "
                                         "incorrect.";

  // Verify cc_transform equals the end-pose of the link (pose2)
  // This confirms the cc_transform recovery correctly accounts for
  // non-identity local offsets.
  const auto& cr = result_vector[0];
  std::size_t arm_idx = (cr.link_names[0] == "arm_link") ? 0 : 1;

  // cc_transform should be the link transform at t=1 (pose2)
  EXPECT_TRUE(cr.cc_transform[arm_idx].isApprox(pose2, 0.01)) << "arm_link cc_transform should match end pose (90 deg "
                                                                 "rotation around Z). "
                                                              << "If it doesn't, the cc_transform recovery may not "
                                                                 "account for local shape offsets.\n"
                                                              << "Expected:\n"
                                                              << pose2.matrix() << "\nGot:\n"
                                                              << cr.cc_transform[arm_idx].matrix();

  // transform should be the link transform at t=0 (pose1)
  EXPECT_TRUE(cr.transform[arm_idx].isApprox(pose1, 0.01)) << "arm_link transform should match start pose (identity).\n"
                                                           << "Expected:\n"
                                                           << pose1.matrix() << "\nGot:\n"
                                                           << cr.transform[arm_idx].matrix();

  // cc_type should be Between (collision happens during the sweep, not at endpoints)
  EXPECT_EQ(cr.cc_type[arm_idx], ContinuousCollisionType::CCType_Between)
      << "arm_link cc_type should be CCType_Between, got " << static_cast<int>(cr.cc_type[arm_idx]);

  // cc_time should be between 0 and 1 (sub-shape A passes nearest the obstacle mid-sweep)
  EXPECT_GT(cr.cc_time[arm_idx], 0.0) << "cc_time should be > 0 (not at start pose)";
  EXPECT_LT(cr.cc_time[arm_idx], 1.0) << "cc_time should be < 1 (not at end pose)";
}

/**
 * @brief Test discrete collision with a multi-shape link to verify basic multi-shape handling.
 */
inline void runTestMultiShapeDiscrete(DiscreteContactManager& checker)
{
  ////////////////////////////
  // Add static obstacle box
  ////////////////////////////
  CollisionShapePtr static_box = std::make_shared<tesseract::geometry::Box>(0.5, 0.5, 0.5);
  Eigen::Isometry3d static_box_pose;
  static_box_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract::common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(static_box);
  obj1_poses.push_back(static_box_pose);

  checker.addCollisionObject("obstacle_link", 0, obj1_shapes, obj1_poses, false);
  checker.enableCollisionObject("obstacle_link");

  ////////////////////////////////////////////////////////////////
  // Add arm link with TWO sub-shapes at different local offsets
  ////////////////////////////////////////////////////////////////
  CollisionShapePtr arm_box_a = std::make_shared<tesseract::geometry::Box>(0.2, 0.2, 0.2);
  CollisionShapePtr arm_box_b = std::make_shared<tesseract::geometry::Box>(0.2, 0.2, 0.2);

  Eigen::Isometry3d shape_pose_a;
  shape_pose_a.setIdentity();
  shape_pose_a.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  Eigen::Isometry3d shape_pose_b;
  shape_pose_b.setIdentity();
  shape_pose_b.translation() = Eigen::Vector3d(-1.0, 0.0, 0.0);

  CollisionShapesConst arm_shapes;
  tesseract::common::VectorIsometry3d arm_poses;
  arm_shapes.push_back(arm_box_a);
  arm_shapes.push_back(arm_box_b);
  arm_poses.push_back(shape_pose_a);
  arm_poses.push_back(shape_pose_b);

  checker.addCollisionObject("arm_link", 0, arm_shapes, arm_poses);

  checker.setActiveCollisionObjects({ "arm_link" });
  checker.setDefaultCollisionMargin(0.05);

  // Place obstacle at (0, 1.0, 0)
  checker.setCollisionObjectsTransform("obstacle_link", Eigen::Isometry3d(Eigen::Translation3d(0.0, 1.0, 0.0)));

  // After 90 deg rotation, sub-shape A is at (0, 1, 0) — overlapping obstacle
  Eigen::Isometry3d arm_pose = Eigen::Isometry3d::Identity();
  arm_pose.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  checker.setCollisionObjectsTransform("arm_link", arm_pose);

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty()) << "Multi-shape discrete: expected collision between sub-shape A (at (0,1,0) "
                                         "after 90 deg "
                                      << "rotation) and static obstacle at (0,1,0).";
}

}  // namespace detail

inline void runTestContinuous(ContinuousContactManager& checker) { detail::runTestMultiShapeCast(checker); }

inline void runTestDiscrete(DiscreteContactManager& checker) { detail::runTestMultiShapeDiscrete(checker); }

}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_MULTI_SHAPE_CAST_UNIT_HPP
