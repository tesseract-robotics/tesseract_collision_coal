#ifndef TESSERACT_COLLISION_COLLISION_CAST_DISABLE_DURING_SWEEP_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_CAST_DISABLE_DURING_SWEEP_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <algorithm>

#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/common.h>
#include <tesseract/geometry/geometries.h>

namespace tesseract::collision::test_suite
{
namespace detail
{
inline void addStaticBoxAndActiveSphere(ContinuousContactManager& checker)
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

inline bool hasPairInResults(const ContactResultVector& results, const std::string& a, const std::string& b)
{
  return std::any_of(results.begin(), results.end(), [&a, &b](const ContactResult& cr) {
    return ((cr.link_names[0] == a && cr.link_names[1] == b) || (cr.link_names[0] == b && cr.link_names[1] == a));
  });
}

/// Verifies that disabling an object, setting its sweep transform, re-enabling
/// it, then setting only the start pose does NOT retain stale cast sweep state.
inline void runTestDisabledObjectSweepDoesNotUpdateCastState(ContinuousContactManager& checker)
{
  addStaticBoxAndActiveSphere(checker);

  checker.setActiveCollisionObjects({ "moving_sphere" });
  checker.setDefaultCollisionMargin(0.0);

  const Eigen::Isometry3d start = Eigen::Isometry3d(Eigen::Translation3d(-2.0, 0.0, 0.0));
  const Eigen::Isometry3d end = Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.0));

  // Disable the sphere, set a sweep transform (should be ignored), re-enable it
  checker.disableCollisionObject("moving_sphere");
  checker.setCollisionObjectsTransform("moving_sphere", start, end);
  checker.enableCollisionObject("moving_sphere");

  // Force a broadphase update at the start pose while leaving cast sweep state untouched
  checker.setCollisionObjectsTransform("moving_sphere", start);

  ContactResultMap no_sweep_result;
  checker.contactTest(no_sweep_result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector no_sweep_vector;
  no_sweep_result.flattenMoveResults(no_sweep_vector);

  // At the start pose (-2, 0, 0) the sphere is far from the box — no contact expected
  EXPECT_FALSE(hasPairInResults(no_sweep_vector, "static_box", "moving_sphere"));

  // Now set the sweep properly (object is enabled)
  checker.setCollisionObjectsTransform("moving_sphere", start, end);

  ContactResultMap sweep_result;
  checker.contactTest(sweep_result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector sweep_vector;
  sweep_result.flattenMoveResults(sweep_vector);

  // The sweep from (-2,0,0) to (0,0,0) passes through the box — contact expected
  EXPECT_TRUE(hasPairInResults(sweep_vector, "static_box", "moving_sphere"));
}
}  // namespace detail

inline void runTestDisabledObjectSweepDoesNotUpdateCastState(ContinuousContactManager& checker)
{
  detail::runTestDisabledObjectSweepDoesNotUpdateCastState(checker);
}

}  // namespace tesseract::collision::test_suite
#endif  // TESSERACT_COLLISION_COLLISION_CAST_DISABLE_DURING_SWEEP_UNIT_HPP
