#ifndef TESSERACT_COLLISION_COLLISION_CAST_CCTYPE_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_CAST_CCTYPE_UNIT_HPP

/**
 * @file collision_cast_cctype_unit.hpp
 * @brief Tests for CCType_Time0, CCType_Time1, and type_id propagation in cast contacts.
 *
 * These tests cover gaps identified in the 2026-03-18 ContactResult audit:
 *
 *   CCType_Time1: A moving sphere translates in +x and is deepest inside the static
 *                 sphere at t=1 (end pose). The contact normal aligns with the sweep
 *                 direction, so the backend must classify cc_type = CCType_Time1 and
 *                 set cc_time = 1.0.  Prior to the "link-frame world supports" fix in
 *                 Coal, Coal always produced CCType_Between for pure translations.
 *
 *   CCType_Time0: A moving sphere starts inside the static sphere and translates away.
 *                 The contact is worst at t=0 (start pose). The backend must classify
 *                 cc_type = CCType_Time0 and set cc_time = 0.0.
 *
 *   type_id:      addCollisionObject receives a non-zero type_id (7 for the moving
 *                 sphere).  The contact result must propagate it correctly.
 *
 * Geometry (shared by all three scenarios):
 *   - "static_sphere":  r=0.25, type_id=0, at (0, 0, 0), static
 *   - "moving_sphere":  r=0.25, type_id=7, kinematic
 *
 * For CCType_Time1:  moving_sphere sweeps from (-2, 0, 0) → (-0.1, 0, 0)
 *   The CastHullShape is a capsule from x=-2 to x=-0.1, radius 0.25.
 *   The end pose sphere (center -0.1) overlaps static_sphere by 0.5-0.1=0.4.
 *   Contact normal: +x (moving sphere approaches from −x).
 *   normal · sweep = (+x)·(+1.9x) = 1.9 > 0 → CCType_Time1.
 *
 * For CCType_Time0:  moving_sphere sweeps from (-0.1, 0, 0) → (-2, 0, 0)
 *   Identical geometry (same convex hull), but now the START pose is deepest.
 *   normal · sweep = (+x)·(−1.9x) = −1.9 < 0 → CCType_Time0.
 *
 * Analytical values for nearest_points_local:
 *   For a sphere (support point = r * normal_local):
 *     nearest_points_local[ki] = link_tf_inv * shape_tf0 * (r * normal_local)
 *   Since pure translation ⇒ normal_local = normal_world = +x:
 *     = link_tf_inv * (start_pose_translation + r*x_hat)
 *     = (0.25, 0, 0)  [independent of start pose for a sphere]
 *   Round-trips:
 *     transform[ki]    * (0.25, 0, 0) = start_pos + (0.25, 0, 0)
 *     cc_transform[ki] * (0.25, 0, 0) = end_pos   + (0.25, 0, 0)
 */

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/geometry/geometries.h>

namespace tesseract::collision::test_suite
{
namespace detail
{
// ---------------------------------------------------------------------------
// Setup helpers
// ---------------------------------------------------------------------------

/// Add static_sphere (type_id=0) and moving_sphere (type_id=7) to the checker.
/// The shapes have identity local poses (no per-shape translation offset).
inline void addCCTypeObjects(ContinuousContactManager& checker)
{
  // Static sphere at origin (stays fixed, not active)
  CollisionShapePtr ss = std::make_shared<tesseract::geometry::Sphere>(0.25);
  Eigen::Isometry3d ss_pose = Eigen::Isometry3d::Identity();

  CollisionShapesConst s1_shapes{ ss };
  tesseract::common::VectorIsometry3d s1_poses{ ss_pose };
  ASSERT_TRUE(checker.addCollisionObject("static_sphere", 0, s1_shapes, s1_poses, true));

  // Moving sphere (kinematic)
  CollisionShapePtr ms = std::make_shared<tesseract::geometry::Sphere>(0.25);
  Eigen::Isometry3d ms_pose = Eigen::Isometry3d::Identity();

  CollisionShapesConst s2_shapes{ ms };
  tesseract::common::VectorIsometry3d s2_poses{ ms_pose };
  ASSERT_TRUE(checker.addCollisionObject("moving_sphere", 7, s2_shapes, s2_poses, true));
}

/// Derive contact-result slot indices from link_names.
/// Returns {moving_idx, static_idx, normal_sign}
/// where normal_sign = +1 if link_names[0]=="moving_sphere", else -1.
inline std::array<int, 3> getSlots(const ContactResult& cr)
{
  if (cr.link_names[0] == "moving_sphere")
    return { 0, 1, 1 };
  return { 1, 0, -1 };
}

// ---------------------------------------------------------------------------
// CCType_Time1
// ---------------------------------------------------------------------------

/**
 * Moving sphere sweeps from (−2, 0, 0) → (−0.1, 0, 0) (pure +x translation).
 * Static sphere at origin.
 *
 * The contact at t=1 (end pose) is the deepest (sphere centre only 0.1 from
 * static sphere centre, penetration = 0.5 − 0.1 = 0.4).
 * contact.normal · sweep > 0  →  CCType_Time1, cc_time = 1.0.
 */
inline void runTestCCTypeTime1(ContinuousContactManager& checker)
{
  addCCTypeObjects(checker);
  checker.setActiveCollisionObjects({ "moving_sphere" });
  checker.setDefaultCollisionMargin(0.0);

  // Static sphere fixed at origin
  checker.setCollisionObjectsTransform("static_sphere", Eigen::Isometry3d::Identity());

  const Eigen::Isometry3d start_pose =
      Eigen::Isometry3d(Eigen::Translation3d(-2.0, 0.0, 0.0));
  const Eigen::Isometry3d end_pose =
      Eigen::Isometry3d(Eigen::Translation3d(-0.1, 0.0, 0.0));

  checker.setCollisionObjectsTransform("moving_sphere", start_pose, end_pose);

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty())
      << "CCType_Time1: moving_sphere (r=0.25) sweeps from x=-2 to x=-0.1. "
      << "At t=1 its centre is 0.1 from static_sphere centre → penetration 0.4. "
      << "A contact is expected.";

  const auto& cr = result_vector[0];
  const auto slots = getSlots(cr);
  const std::size_t ki = static_cast<std::size_t>(slots[0]);  // moving_sphere slot
  const std::size_t si = static_cast<std::size_t>(slots[1]);  // static_sphere slot

  SCOPED_TRACE("CCType_Time1 contact: link_names=[" + cr.link_names[0] + ", " +
               cr.link_names[1] + "] ki=" + std::to_string(ki) +
               " si=" + std::to_string(si));

  // -----------------------------------------------------------------------
  // distance
  // -----------------------------------------------------------------------
  EXPECT_NEAR(cr.distance, -0.4, 0.001)
      << "Penetration depth: sphere centres 0.1 apart, combined radii 0.5 → −0.4";

  // -----------------------------------------------------------------------
  // type_id propagation
  // -----------------------------------------------------------------------
  EXPECT_EQ(cr.type_id[ki], 7)
      << "moving_sphere was registered with type_id=7; result must reflect that";
  EXPECT_EQ(cr.type_id[si], 0)
      << "static_sphere was registered with type_id=0; result must reflect that";

  // -----------------------------------------------------------------------
  // cc_type / cc_time for the moving (kinematic) sphere
  // -----------------------------------------------------------------------
  EXPECT_EQ(cr.cc_type[ki], ContinuousCollisionType::CCType_Time1)
      << "moving_sphere translates in +x; contact normal is in +x direction; "
      << "normal·sweep > 0 → CCType_Time1. Got: " << static_cast<int>(cr.cc_type[ki]);

  EXPECT_NEAR(cr.cc_time[ki], 1.0, 0.001)
      << "CCType_Time1 → cc_time must be 1.0 (collision worst at end pose)";

  // -----------------------------------------------------------------------
  // cc_type / cc_time for the static sphere
  // -----------------------------------------------------------------------
  EXPECT_EQ(cr.cc_type[si], ContinuousCollisionType::CCType_None)
      << "static_sphere has no CastHullShape → cc_type must be CCType_None";

  EXPECT_NEAR(cr.cc_time[si], -1.0, 0.001)
      << "static_sphere has no CastHullShape → cc_time must be −1 (unset)";

  // -----------------------------------------------------------------------
  // transform / cc_transform for the moving sphere
  // -----------------------------------------------------------------------
  EXPECT_TRUE(cr.transform[ki].isApprox(start_pose, 1e-5))
      << "transform[ki] must equal the start pose of moving_sphere";

  EXPECT_TRUE(cr.cc_transform[ki].isApprox(end_pose, 1e-5))
      << "cc_transform[ki] must equal the end pose of moving_sphere";

  // -----------------------------------------------------------------------
  // normal is a unit vector
  // -----------------------------------------------------------------------
  EXPECT_NEAR(cr.normal.norm(), 1.0, 1e-4)
      << "Contact normal must be a unit vector";

  // Contact normal must have a component along +x (sphere-sphere along X axis)
  const int ns = slots[2];  // +1 if moving_sphere is link_names[0]
  EXPECT_GT(ns * cr.normal[0], 0.5)
      << "Contact normal x-component should point from moving_sphere toward static_sphere (+x)";

  // -----------------------------------------------------------------------
  // nearest_points (world frame)
  // -----------------------------------------------------------------------
  // At t=1: moving_sphere surface toward static_sphere is at (−0.1+0.25, 0, 0)=(0.15, 0, 0)
  // static_sphere surface toward moving_sphere is at (−0.25, 0, 0)
  EXPECT_NEAR(cr.nearest_points[ki][0], 0.15, 0.001)
      << "nearest_points[ki].x: moving_sphere surface at end pose toward static_sphere";
  EXPECT_NEAR(cr.nearest_points[ki][1], 0.0, 0.001)
      << "nearest_points[ki].y: no offset in Y";
  EXPECT_NEAR(cr.nearest_points[ki][2], 0.0, 0.001)
      << "nearest_points[ki].z: no offset in Z";

  EXPECT_NEAR(cr.nearest_points[si][0], -0.25, 0.001)
      << "nearest_points[si].x: static_sphere surface in −x direction";
  EXPECT_NEAR(cr.nearest_points[si][1], 0.0, 0.001)
      << "nearest_points[si].y: no offset in Y";
  EXPECT_NEAR(cr.nearest_points[si][2], 0.0, 0.001)
      << "nearest_points[si].z: no offset in Z";

  // -----------------------------------------------------------------------
  // nearest_points_local via round-trip transforms
  //
  // For CCType_Time1, nearest_points_local[ki] = sphere surface at r=0.25
  // in the contact-normal direction, expressed in link-local coords.
  // For a sphere with identity local pose: (0.25, 0, 0).
  //
  // transform[ki]    * nearest_points_local[ki] = start_pos + (0.25,0,0) = (-1.75,0,0)
  // cc_transform[ki] * nearest_points_local[ki] = end_pos   + (0.25,0,0) = ( 0.15,0,0)
  // -----------------------------------------------------------------------
  EXPECT_GT(cr.nearest_points_local[ki].norm(), 1e-6)
      << "nearest_points_local[ki] must be non-zero (was previously left at zero for CCType_Time1)";

  const Eigen::Vector3d p_at_start = cr.transform[ki] * cr.nearest_points_local[ki];
  EXPECT_NEAR(p_at_start[0], -1.75, 0.001)
      << "transform[ki] * nearest_points_local[ki]: sphere surface at start pose in +x direction";
  EXPECT_NEAR(p_at_start[1], 0.0, 0.001);
  EXPECT_NEAR(p_at_start[2], 0.0, 0.001);

  const Eigen::Vector3d p_at_end = cr.cc_transform[ki] * cr.nearest_points_local[ki];
  EXPECT_NEAR(p_at_end[0], 0.15, 0.001)
      << "cc_transform[ki] * nearest_points_local[ki]: sphere surface at end pose in +x direction "
      << "(matches nearest_points[ki])";
  EXPECT_NEAR(p_at_end[1], 0.0, 0.001);
  EXPECT_NEAR(p_at_end[2], 0.0, 0.001);
}

// ---------------------------------------------------------------------------
// CCType_Time0
// ---------------------------------------------------------------------------

/**
 * Moving sphere sweeps from (−0.1, 0, 0) → (−2, 0, 0) (pure −x translation).
 * Static sphere at origin.
 *
 * The contact at t=0 (start pose) is the deepest (centre 0.1 from static sphere,
 * penetration 0.4).  As the moving sphere retreats the overlap reduces.
 * contact.normal · sweep < 0  →  CCType_Time0, cc_time = 0.0.
 */
inline void runTestCCTypeTime0(ContinuousContactManager& checker)
{
  addCCTypeObjects(checker);
  checker.setActiveCollisionObjects({ "moving_sphere" });
  checker.setDefaultCollisionMargin(0.0);

  // Static sphere fixed at origin
  checker.setCollisionObjectsTransform("static_sphere", Eigen::Isometry3d::Identity());

  const Eigen::Isometry3d start_pose =
      Eigen::Isometry3d(Eigen::Translation3d(-0.1, 0.0, 0.0));
  const Eigen::Isometry3d end_pose =
      Eigen::Isometry3d(Eigen::Translation3d(-2.0, 0.0, 0.0));

  checker.setCollisionObjectsTransform("moving_sphere", start_pose, end_pose);

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty())
      << "CCType_Time0: moving_sphere starts at x=-0.1 overlapping static_sphere. "
      << "The swept hull includes this start position, so a contact is expected.";

  const auto& cr = result_vector[0];
  const auto slots = getSlots(cr);
  const std::size_t ki = static_cast<std::size_t>(slots[0]);
  const std::size_t si = static_cast<std::size_t>(slots[1]);

  SCOPED_TRACE("CCType_Time0 contact: link_names=[" + cr.link_names[0] + ", " +
               cr.link_names[1] + "] ki=" + std::to_string(ki) +
               " si=" + std::to_string(si));

  // -----------------------------------------------------------------------
  // distance
  // -----------------------------------------------------------------------
  EXPECT_NEAR(cr.distance, -0.4, 0.001)
      << "Penetration depth: same geometry as CCType_Time1 scenario";

  // -----------------------------------------------------------------------
  // type_id propagation
  // -----------------------------------------------------------------------
  EXPECT_EQ(cr.type_id[ki], 7)
      << "moving_sphere type_id must be 7 in both CCType_Time0 and CCType_Time1";
  EXPECT_EQ(cr.type_id[si], 0)
      << "static_sphere type_id must be 0";

  // -----------------------------------------------------------------------
  // cc_type / cc_time for the moving sphere
  // -----------------------------------------------------------------------
  EXPECT_EQ(cr.cc_type[ki], ContinuousCollisionType::CCType_Time0)
      << "moving_sphere retreats in −x; contact normal is in +x direction; "
      << "normal·sweep < 0 → CCType_Time0. Got: " << static_cast<int>(cr.cc_type[ki]);

  EXPECT_NEAR(cr.cc_time[ki], 0.0, 0.001)
      << "CCType_Time0 → cc_time must be 0.0 (collision worst at start pose)";

  // -----------------------------------------------------------------------
  // cc_type / cc_time for the static sphere
  // -----------------------------------------------------------------------
  EXPECT_EQ(cr.cc_type[si], ContinuousCollisionType::CCType_None)
      << "static_sphere → CCType_None";

  EXPECT_NEAR(cr.cc_time[si], -1.0, 0.001)
      << "static_sphere → cc_time = −1 (unset)";

  // -----------------------------------------------------------------------
  // transform / cc_transform for the moving sphere
  // -----------------------------------------------------------------------
  EXPECT_TRUE(cr.transform[ki].isApprox(start_pose, 1e-5))
      << "transform[ki] must equal the start pose (−0.1, 0, 0)";

  EXPECT_TRUE(cr.cc_transform[ki].isApprox(end_pose, 1e-5))
      << "cc_transform[ki] must equal the end pose (−2, 0, 0)";

  // -----------------------------------------------------------------------
  // normal is a unit vector
  // -----------------------------------------------------------------------
  EXPECT_NEAR(cr.normal.norm(), 1.0, 1e-4)
      << "Contact normal must be a unit vector";

  // Contact normal still points from moving_sphere toward static_sphere (+x)
  const int ns = slots[2];
  EXPECT_GT(ns * cr.normal[0], 0.5)
      << "Contact normal x must point from moving_sphere toward static_sphere (+x)";

  // -----------------------------------------------------------------------
  // nearest_points (world frame)
  // -----------------------------------------------------------------------
  // At t=0: moving_sphere surface toward static_sphere is at (−0.1+0.25, 0, 0) = (0.15, 0, 0)
  // static_sphere surface toward moving_sphere is at (−0.25, 0, 0)
  EXPECT_NEAR(cr.nearest_points[ki][0], 0.15, 0.001)
      << "nearest_points[ki].x: moving_sphere surface at start pose toward static_sphere";
  EXPECT_NEAR(cr.nearest_points[ki][1], 0.0, 0.001);
  EXPECT_NEAR(cr.nearest_points[ki][2], 0.0, 0.001);

  EXPECT_NEAR(cr.nearest_points[si][0], -0.25, 0.001)
      << "nearest_points[si].x: static_sphere surface in −x direction";
  EXPECT_NEAR(cr.nearest_points[si][1], 0.0, 0.001);
  EXPECT_NEAR(cr.nearest_points[si][2], 0.0, 0.001);

  // -----------------------------------------------------------------------
  // nearest_points_local via round-trip transforms
  //
  // For CCType_Time0, nearest_points_local[ki] = sphere surface at r=0.25 in
  // the contact-normal direction (link-local). For identity local pose: (0.25, 0, 0).
  //
  // transform[ki]    * (0.25,0,0) = (−0.1+0.25, 0, 0) = ( 0.15, 0, 0)  ← actual contact
  // cc_transform[ki] * (0.25,0,0) = (−2.0+0.25, 0, 0) = (−1.75, 0, 0)
  // -----------------------------------------------------------------------
  EXPECT_GT(cr.nearest_points_local[ki].norm(), 1e-6)
      << "nearest_points_local[ki] must be non-zero (was previously left at zero for CCType_Time0)";

  const Eigen::Vector3d p_at_start = cr.transform[ki] * cr.nearest_points_local[ki];
  EXPECT_NEAR(p_at_start[0], 0.15, 0.001)
      << "transform[ki] * nearest_points_local[ki]: sphere surface at start pose (+x dir) = "
         "contact point at t=0";
  EXPECT_NEAR(p_at_start[1], 0.0, 0.001);
  EXPECT_NEAR(p_at_start[2], 0.0, 0.001);

  const Eigen::Vector3d p_at_end = cr.cc_transform[ki] * cr.nearest_points_local[ki];
  EXPECT_NEAR(p_at_end[0], -1.75, 0.001)
      << "cc_transform[ki] * nearest_points_local[ki]: sphere surface at end pose (+x dir)";
  EXPECT_NEAR(p_at_end[1], 0.0, 0.001);
  EXPECT_NEAR(p_at_end[2], 0.0, 0.001);
}

}  // namespace detail

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

/// Run the CCType_Time1 test on the provided cast manager.
inline void runTestCCTypeTime1(ContinuousContactManager& checker)
{
  detail::runTestCCTypeTime1(checker);
}

/// Run the CCType_Time0 test on the provided cast manager.
inline void runTestCCTypeTime0(ContinuousContactManager& checker)
{
  detail::runTestCCTypeTime0(checker);
}

}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_CAST_CCTYPE_UNIT_HPP
