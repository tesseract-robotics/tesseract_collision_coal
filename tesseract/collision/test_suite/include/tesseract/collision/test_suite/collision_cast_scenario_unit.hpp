/**
 * @file collision_cast_scenario_unit.hpp
 * @brief Continuous collision scenario tests exercising Trajopt-relevant code paths.
 *
 * Seven scenarios covering rotational sweeps, multi-shape links, large margins,
 * articulated arms, subdivision loops, near-miss contacts, and repeated-query
 * stability.  Each scenario validates ContactResult field self-consistency
 * (cc_type, cc_time, transforms, nearest_points_local, normal) independently
 * per backend.
 *
 * Pairwise comparison helpers are also provided for scenarios where cross-backend
 * equivalence is specifically valuable.
 */
#ifndef TESSERACT_COLLISION_COLLISION_CAST_SCENARIO_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_CAST_SCENARIO_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <octomap/octomap.h>
#include <iomanip>
#include <sstream>
#include <cmath>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/common.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>

namespace tesseract::collision::test_suite
{
namespace detail
{
// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Pretty-print a ContactResult for diagnostic output.
inline std::string formatCR(const std::string& label, const ContactResult& cr)
{
  std::ostringstream os;
  os << std::setprecision(6) << std::fixed;
  os << "[" << label << "]"
     << "\n  link_names: [" << cr.link_ids[0].name() << ", " << cr.link_ids[1].name() << "]"
     << "\n  distance: " << cr.distance << "\n  normal: (" << cr.normal.transpose() << ")"
     << "\n  nearest_points[0]: (" << cr.nearest_points[0].transpose() << ")"
     << "\n  nearest_points[1]: (" << cr.nearest_points[1].transpose() << ")"
     << "\n  nearest_points_local[0]: (" << cr.nearest_points_local[0].transpose() << ")"
     << "\n  nearest_points_local[1]: (" << cr.nearest_points_local[1].transpose() << ")"
     << "\n  cc_time: [" << cr.cc_time[0] << ", " << cr.cc_time[1] << "]"
     << "\n  cc_type: [" << static_cast<int>(cr.cc_type[0]) << ", " << static_cast<int>(cr.cc_type[1]) << "]"
     << "\n  shape_id: [" << cr.shape_id[0] << ", " << cr.shape_id[1] << "]"
     << "\n  transform[0].t: (" << cr.transform[0].translation().transpose() << ")"
     << "\n  transform[1].t: (" << cr.transform[1].translation().transpose() << ")"
     << "\n  cc_transform[0].t: (" << cr.cc_transform[0].translation().transpose() << ")"
     << "\n  cc_transform[1].t: (" << cr.cc_transform[1].translation().transpose() << ")";
  return os.str();
}

/// Add a BOX octree at the origin to a cast manager.
inline void addOctree(ContinuousContactManager& checker, const std::string& link_name)
{
  tesseract::common::GeneralResourceLocator locator;
  std::string path = locator.locateResource("package://tesseract/support/meshes/box_2m.bt")->getFilePath();
  auto ot = std::make_shared<octomap::OcTree>(path);
  CollisionShapePtr dense_octomap =
      std::make_shared<tesseract::geometry::Octree>(ot, tesseract::geometry::OctreeSubType::BOX);

  CollisionShapesConst shapes;
  tesseract::common::VectorIsometry3d poses;
  shapes.push_back(dense_octomap);
  poses.push_back(Eigen::Isometry3d::Identity());
  checker.addCollisionObject(link_name, 0, shapes, poses);
}

/// Add a single box shape on a link.
inline void addBoxLink(ContinuousContactManager& checker,
                       const std::string& link_name,
                       const Eigen::Vector3d& half_extents,
                       const Eigen::Isometry3d& local_pose = Eigen::Isometry3d::Identity())
{
  CollisionShapePtr box =
      std::make_shared<tesseract::geometry::Box>(half_extents.x() * 2, half_extents.y() * 2, half_extents.z() * 2);
  CollisionShapesConst shapes;
  tesseract::common::VectorIsometry3d poses;
  shapes.push_back(box);
  poses.push_back(local_pose);
  checker.addCollisionObject(link_name, 0, shapes, poses);
}

/// Add a link with two box sub-shapes at different local offsets.
inline void addMultiShapeLink(ContinuousContactManager& checker,
                              const std::string& link_name,
                              const Eigen::Vector3d& box_half,
                              const Eigen::Vector3d& offset_a,
                              const Eigen::Vector3d& offset_b)
{
  CollisionShapePtr box_a =
      std::make_shared<tesseract::geometry::Box>(box_half.x() * 2, box_half.y() * 2, box_half.z() * 2);
  CollisionShapePtr box_b =
      std::make_shared<tesseract::geometry::Box>(box_half.x() * 2, box_half.y() * 2, box_half.z() * 2);

  Eigen::Isometry3d pose_a = Eigen::Isometry3d::Identity();
  pose_a.translation() = offset_a;
  Eigen::Isometry3d pose_b = Eigen::Isometry3d::Identity();
  pose_b.translation() = offset_b;

  CollisionShapesConst shapes;
  tesseract::common::VectorIsometry3d poses;
  shapes.push_back(box_a);
  shapes.push_back(box_b);
  poses.push_back(pose_a);
  poses.push_back(pose_b);
  checker.addCollisionObject(link_name, 0, shapes, poses);
}

/// Find first contact result involving the given kinematic link, return its index
/// in the ContactResultVector (-1 if not found).  Also determines which slot holds
/// the kinematic link.
/// @param env_link If non-empty, only match contacts where the OTHER link is env_link.
///                 This prevents matching kinematic-vs-kinematic contacts.
inline int findKinContact(const ContactResultVector& results,
                          const std::string& kin_link,
                          std::size_t& kin_idx,
                          const std::string& env_link = "")
{
  for (std::size_t i = 0; i < results.size(); ++i)
  {
    if (results[i].link_ids[0].name() == kin_link)
    {
      if (!env_link.empty() && results[i].link_ids[1].name() != env_link)
        continue;
      kin_idx = 0;
      return static_cast<int>(i);
    }
    if (results[i].link_ids[1].name() == kin_link)
    {
      if (!env_link.empty() && results[i].link_ids[0].name() != env_link)
        continue;
      kin_idx = 1;
      return static_cast<int>(i);
    }
  }
  return -1;
}

/// Compute a simplified Trajopt-style contact point using cc_type, cc_time,
/// nearest_points_local, transform, and cc_transform.
inline Eigen::Vector3d trajoptContactPoint(const ContactResult& cr, std::size_t ki)
{
  switch (cr.cc_type[ki])
  {
    case ContinuousCollisionType::CCType_Time0:
      return cr.transform[ki] * cr.nearest_points_local[ki];
    case ContinuousCollisionType::CCType_Time1:
      return cr.cc_transform[ki] * cr.nearest_points_local[ki];
    case ContinuousCollisionType::CCType_Between:
    {
      Eigen::Isometry3d interp = Eigen::Isometry3d::Identity();
      interp.translation() =
          (1.0 - cr.cc_time[ki]) * cr.transform[ki].translation() + cr.cc_time[ki] * cr.cc_transform[ki].translation();
      Eigen::Quaterniond q0(cr.transform[ki].rotation());
      Eigen::Quaterniond q1(cr.cc_transform[ki].rotation());
      interp.linear() = q0.slerp(cr.cc_time[ki], q1).toRotationMatrix();
      return interp * cr.nearest_points_local[ki];
    }
    default:
      return Eigen::Vector3d::Zero();
  }
}

/// Run contactTest on a manager and flatten results.
inline ContactResultVector runContact(ContinuousContactManager& checker, ContactTestType type = ContactTestType::ALL)
{
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(type));
  ContactResultVector vec;
  result.flattenMoveResults(vec);
  return vec;
}

/// Validate self-consistency of continuous collision fields for a kinematic-vs-octree
/// contact.  Checks that cc_type, cc_time, transforms, and nearest_points_local are
/// internally consistent, independent of what any other backend reports.
inline void validateOctreeContactSelfConsistency(const ContactResult& cr,
                                                 const std::string& kin_link,
                                                 const Eigen::Isometry3d& pose1,
                                                 const Eigen::Isometry3d& pose2,
                                                 const std::string& label)
{
  SCOPED_TRACE(label + ": " + formatCR(label, cr));

  std::size_t ki = std::numeric_limits<std::size_t>::max();
  if (cr.link_ids[0].name() == kin_link)
    ki = 0;
  else if (cr.link_ids[1].name() == kin_link)
    ki = 1;
  ASSERT_NE(ki, std::numeric_limits<std::size_t>::max()) << label << ": kin_link not found in contact";

  EXPECT_TRUE(cr.transform[ki].isApprox(pose1, 0.01)) << label << ": transform should match pose1";
  EXPECT_TRUE(cr.cc_transform[ki].isApprox(pose2, 0.01)) << label << ": cc_transform should match pose2";

  EXPECT_NE(cr.cc_type[ki], ContinuousCollisionType::CCType_None)
      << label << ": kinematic link should not have CCType_None";

  if (cr.cc_type[ki] == ContinuousCollisionType::CCType_Time0)
  {
    EXPECT_NEAR(cr.cc_time[ki], 0.0, 1e-3) << label << ": CCType_Time0 should have cc_time=0";
  }
  else if (cr.cc_type[ki] == ContinuousCollisionType::CCType_Time1)
  {
    EXPECT_NEAR(cr.cc_time[ki], 1.0, 1e-3) << label << ": CCType_Time1 should have cc_time=1";
  }
  else
  {
    EXPECT_GE(cr.cc_time[ki], 0.0) << label << ": CCType_Between cc_time should be >= 0";
    EXPECT_LE(cr.cc_time[ki], 1.0) << label << ": CCType_Between cc_time should be <= 1";
  }

  EXPECT_NEAR(cr.normal.norm(), 1.0, 1e-3) << label << ": normal must be unit vector";

  EXPECT_GT(cr.nearest_points_local[ki].norm(), 1e-6)
      << label << ": nearest_points_local should be non-zero for Trajopt Jacobian";

  Eigen::Vector3d tpt = trajoptContactPoint(cr, ki);
  EXPECT_TRUE(tpt.allFinite()) << label << ": trajopt contact point should be finite";
}

// ---------------------------------------------------------------------------
// Pairwise comparison helpers
// ---------------------------------------------------------------------------

/// For a given reference contact involving kin_link, find the best-matching contact
/// in the other backend's results by closest outward normal from the kinematic link.
/// @param env_link If non-empty, only consider contacts where the other link is env_link.
inline int findMatchingContact(const ContactResult& ref_cr,
                               std::size_t ref_ki,
                               const ContactResultVector& other_results,
                               const std::string& kin_link,
                               std::size_t& other_ki,
                               const std::string& env_link = "")
{
  const Eigen::Vector3d ref_outward = (ref_ki == 0 ? 1.0 : -1.0) * ref_cr.normal;

  int best_idx = -1;
  double best_dot = -2.0;
  for (std::size_t i = 0; i < other_results.size(); ++i)
  {
    std::size_t ki = std::numeric_limits<std::size_t>::max();
    if (other_results[i].link_ids[0].name() == kin_link)
    {
      if (!env_link.empty() && other_results[i].link_ids[1].name() != env_link)
        continue;
      ki = 0;
    }
    else if (other_results[i].link_ids[1].name() == kin_link)
    {
      if (!env_link.empty() && other_results[i].link_ids[0].name() != env_link)
        continue;
      ki = 1;
    }
    else
      continue;

    const Eigen::Vector3d other_outward = (ki == 0 ? 1.0 : -1.0) * other_results[i].normal;
    double dot = ref_outward.dot(other_outward);
    if (dot > best_dot)
    {
      best_dot = dot;
      best_idx = static_cast<int>(i);
      other_ki = ki;
    }
  }
  return best_idx;
}

/// Compare a scalar between two backends, logging differences.
inline bool
compareScalar(const std::string& field_name, double val_a, double val_b, double tolerance, std::ostringstream& log)
{
  bool match = std::abs(val_a - val_b) <= tolerance;
  if (!match)
    log << "  DIFF " << field_name << ": a=" << val_a << "  b=" << val_b << "  delta=" << std::abs(val_a - val_b)
        << "\n";
  return match;
}

/// Compare an integer between two backends, logging differences.
inline bool compareInt(const std::string& field_name, int val_a, int val_b, std::ostringstream& log)
{
  bool match = (val_a == val_b);
  if (!match)
    log << "  DIFF " << field_name << ": a=" << val_a << "  b=" << val_b << "\n";
  return match;
}

/// Compare a Vector3d between two backends, logging differences.
inline bool compareVector(const std::string& field_name,
                          const Eigen::Vector3d& val_a,
                          const Eigen::Vector3d& val_b,
                          double tolerance,
                          std::ostringstream& log)
{
  bool match = (val_a - val_b).norm() <= tolerance;
  if (!match)
    log << "  DIFF " << field_name << ": a=(" << val_a.transpose() << ")  b=(" << val_b.transpose()
        << ")  delta=" << (val_a - val_b).norm() << "\n";
  return match;
}

// ---------------------------------------------------------------------------
// Scenario A: Rotational sweep through octree
// ---------------------------------------------------------------------------

inline void runTestScenarioA_RotationalSweep(ContinuousContactManager& checker)
{
  addOctree(checker, "octomap_link");
  addBoxLink(checker, "kin_link", Eigen::Vector3d(0.1, 0.1, 0.1));
  checker.setActiveCollisionObjects(std::vector<std::string>{ "kin_link" });
  checker.setDefaultCollisionMargin(0.1);
  checker.setCollisionObjectsTransform("octomap_link", Eigen::Isometry3d::Identity());

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() = Eigen::Vector3d(-2.0, 0, 0);
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(0.0, 0, 0);
  pose2.linear() = Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  checker.setCollisionObjectsTransform("kin_link", pose1, pose2);
  auto results = runContact(checker);

  ASSERT_FALSE(results.empty()) << "No contacts found for rotational sweep through octree";

  for (std::size_t i = 0; i < results.size(); ++i)
    validateOctreeContactSelfConsistency(results[i], "kin_link", pose1, pose2, "Result[" + std::to_string(i) + "]");
}

// ---------------------------------------------------------------------------
// Scenario B: Multi-shape link sweep through octree
// ---------------------------------------------------------------------------

inline void runTestScenarioB_MultiShapeSweep(ContinuousContactManager& checker)
{
  addOctree(checker, "octomap_link");
  addMultiShapeLink(
      checker, "arm_link", Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(0.5, 0, 0), Eigen::Vector3d(-0.5, 0, 0));
  checker.setActiveCollisionObjects(std::vector<std::string>{ "arm_link" });
  checker.setDefaultCollisionMargin(0.1);
  checker.setCollisionObjectsTransform("octomap_link", Eigen::Isometry3d::Identity());

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() = Eigen::Vector3d(-2.0, 0, 0);
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(0.0, 0, 0);

  checker.setCollisionObjectsTransform("arm_link", pose1, pose2);
  auto results = runContact(checker);

  ASSERT_FALSE(results.empty()) << "No contacts found for multi-shape sweep through octree";

  for (std::size_t i = 0; i < results.size(); ++i)
    validateOctreeContactSelfConsistency(results[i], "arm_link", pose1, pose2, "Result[" + std::to_string(i) + "]");
}

// ---------------------------------------------------------------------------
// Scenario C: Large collision margin
// ---------------------------------------------------------------------------

inline void runTestScenarioC_LargeMargin(ContinuousContactManager& checker)
{
  const double margin = 0.5;

  addOctree(checker, "octomap_link");
  addBoxLink(checker, "kin_link", Eigen::Vector3d(0.1, 0.1, 0.1));
  checker.setActiveCollisionObjects(std::vector<std::string>{ "kin_link" });
  checker.setDefaultCollisionMargin(margin);
  checker.setCollisionObjectsTransform("octomap_link", Eigen::Isometry3d::Identity());

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() = Eigen::Vector3d(-2.0, 0, 0);
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(0.0, 0, 0);
  pose2.linear() = Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  checker.setCollisionObjectsTransform("kin_link", pose1, pose2);
  auto results = runContact(checker);

  ASSERT_FALSE(results.empty()) << "No contacts found with margin=" << margin;

  for (std::size_t i = 0; i < results.size(); ++i)
    validateOctreeContactSelfConsistency(results[i], "kin_link", pose1, pose2, "Result[" + std::to_string(i) + "]");
}

// ---------------------------------------------------------------------------
// Scenario D: Articulated arm simulation
// ---------------------------------------------------------------------------

struct ArticulatedArmPoses
{
  Eigen::Isometry3d parent_pose1;
  Eigen::Isometry3d parent_pose2;
  Eigen::Isometry3d child_pose1;
  Eigen::Isometry3d child_pose2;
};

inline ArticulatedArmPoses setupArticulatedArm(ContinuousContactManager& checker)
{
  addOctree(checker, "octomap_link");
  addBoxLink(checker, "parent_link", Eigen::Vector3d(0.15, 0.15, 0.15));
  addBoxLink(checker, "child_link", Eigen::Vector3d(0.1, 0.1, 0.1));
  checker.setActiveCollisionObjects(std::vector<std::string>{ "parent_link", "child_link" });
  checker.setDefaultCollisionMargin(0.1);
  checker.setCollisionObjectsTransform("octomap_link", Eigen::Isometry3d::Identity());

  ArticulatedArmPoses poses;
  poses.parent_pose1 = Eigen::Isometry3d::Identity();
  poses.parent_pose1.translation() = Eigen::Vector3d(-2.0, 0, 0);
  poses.parent_pose2 = Eigen::Isometry3d::Identity();
  poses.parent_pose2.translation() = Eigen::Vector3d(-0.5, 0, 0);
  poses.parent_pose2.linear() = Eigen::AngleAxisd(M_PI / 3.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  checker.setCollisionObjectsTransform("parent_link", poses.parent_pose1, poses.parent_pose2);

  poses.child_pose1 = Eigen::Isometry3d::Identity();
  poses.child_pose1.translation() = Eigen::Vector3d(-1.2, 0, 0);
  poses.child_pose2 = Eigen::Isometry3d::Identity();
  poses.child_pose2.translation() = Eigen::Vector3d(-0.1, 0.693, 0);
  poses.child_pose2.linear() = poses.parent_pose2.linear();
  checker.setCollisionObjectsTransform("child_link", poses.child_pose1, poses.child_pose2);

  return poses;
}

inline void runTestScenarioD_ArticulatedArm(ContinuousContactManager& checker)
{
  auto poses = setupArticulatedArm(checker);
  auto results = runContact(checker);

  // Validate self-consistency for every contact found
  for (std::size_t i = 0; i < results.size(); ++i)
  {
    const auto& cr = results[i];
    const std::string label = "Result[" + std::to_string(i) + "]";

    // Determine which link this contact involves and use the right poses
    if (cr.link_ids[0].name() == "parent_link" || cr.link_ids[1].name() == "parent_link")
      validateOctreeContactSelfConsistency(cr, "parent_link", poses.parent_pose1, poses.parent_pose2, label);
    if (cr.link_ids[0].name() == "child_link" || cr.link_ids[1].name() == "child_link")
      validateOctreeContactSelfConsistency(cr, "child_link", poses.child_pose1, poses.child_pose2, label);
  }
}

/// Pairwise comparison: run both managers on the articulated arm scenario
/// and verify they agree on which links have contacts and produce consistent fields.
inline void runTestScenarioD_Comparison(ContinuousContactManager& checker_a,
                                        ContinuousContactManager& checker_b,
                                        const std::string& label_a,
                                        const std::string& label_b)
{
  auto poses_a = setupArticulatedArm(checker_a);
  auto poses_b = setupArticulatedArm(checker_b);
  (void)poses_a;
  (void)poses_b;

  auto results_a = runContact(checker_a);
  auto results_b = runContact(checker_b);

  // Compare only kinematic-vs-environment contacts (not kinematic-vs-kinematic).
  // Both backends produce identical contacts, but ContactResultMap iteration order
  // is non-deterministic, so findKinContact must filter by the environment link.
  const std::string env_link = "octomap_link";

  // Both must agree on which links have contacts with the environment
  for (const auto& link : { "parent_link", "child_link" })
  {
    std::size_t a_ki = 0, b_ki = 0;
    bool a_has = (findKinContact(results_a, link, a_ki, env_link) >= 0);
    bool b_has = (findKinContact(results_b, link, b_ki, env_link) >= 0);
    if (a_has && !b_has)
      ADD_FAILURE() << label_a << " found contact for " << link << " but " << label_b << " did not";
    if (!a_has && b_has)
      ADD_FAILURE() << label_b << " found contact for " << link << " but " << label_a << " did not";
  }

  // For each link contact found by checker_a, find matching contact in checker_b and compare fields
  for (const auto& link : { "parent_link", "child_link" })
  {
    std::size_t a_ki = 0;
    int ai = findKinContact(results_a, link, a_ki, env_link);
    if (ai < 0)
      continue;

    const auto& acr = results_a[static_cast<std::size_t>(ai)];
    std::size_t b_ki = 0;
    int bi = findMatchingContact(acr, a_ki, results_b, link, b_ki, env_link);
    if (bi < 0)
      continue;

    const auto& bcr = results_b[static_cast<std::size_t>(bi)];

    std::ostringstream diff_log;
    bool all_match = true;
    all_match &= compareInt("cc_type[" + std::string(link) + "]",
                            static_cast<int>(acr.cc_type[a_ki]),
                            static_cast<int>(bcr.cc_type[b_ki]),
                            diff_log);
    all_match &=
        compareScalar("cc_time[" + std::string(link) + "]", acr.cc_time[a_ki], bcr.cc_time[b_ki], 0.15, diff_log);
    all_match &= compareScalar("distance[" + std::string(link) + "]", acr.distance, bcr.distance, 0.1, diff_log);

    Eigen::Vector3d a_pt = trajoptContactPoint(acr, a_ki);
    Eigen::Vector3d b_pt = trajoptContactPoint(bcr, b_ki);
    all_match &= compareVector("trajopt_contact_point[" + std::string(link) + "]", a_pt, b_pt, 0.3, diff_log);

    EXPECT_TRUE(all_match) << "ContactResult field divergence for " << link << " between " << label_a << " and "
                           << label_b << ".\n"
                           << diff_log.str();
  }
}

// ---------------------------------------------------------------------------
// Scenario E: Subdivision loop (Trajopt-style)
// ---------------------------------------------------------------------------

inline void runTestScenarioE_SubdivisionLoop(ContinuousContactManager& checker)
{
  const int num_segments = 6;

  addOctree(checker, "octomap_link");
  addBoxLink(checker, "kin_link", Eigen::Vector3d(0.15, 0.15, 0.15));
  checker.setActiveCollisionObjects(std::vector<std::string>{ "kin_link" });
  checker.setDefaultCollisionMargin(0.25);
  checker.setCollisionObjectsTransform("octomap_link", Eigen::Isometry3d::Identity());

  Eigen::Isometry3d full_start = Eigen::Isometry3d::Identity();
  full_start.translation() = Eigen::Vector3d(-3.0, 0, 0);
  Eigen::Isometry3d full_end = Eigen::Isometry3d::Identity();
  full_end.translation() = Eigen::Vector3d(3.0, 0, 0);
  full_end.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  auto interpolate = [&](double t) -> Eigen::Isometry3d {
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translation() = (1.0 - t) * full_start.translation() + t * full_end.translation();
    Eigen::Quaterniond q0(full_start.rotation());
    Eigen::Quaterniond q1(full_end.rotation());
    result.linear() = q0.slerp(t, q1).toRotationMatrix();
    return result;
  };

  int total_contacts = 0;
  int segments_found = 0;

  for (int seg = 0; seg < num_segments; ++seg)
  {
    double t0 = static_cast<double>(seg) / num_segments;
    double t1 = static_cast<double>(seg + 1) / num_segments;
    Eigen::Isometry3d pose1 = interpolate(t0);
    Eigen::Isometry3d pose2 = interpolate(t1);

    checker.setCollisionObjectsTransform("kin_link", pose1, pose2);
    auto results = runContact(checker);

    total_contacts += static_cast<int>(results.size());
    if (!results.empty())
      segments_found++;

    // Self-consistency for every contact in every segment
    for (std::size_t i = 0; i < results.size(); ++i)
      validateOctreeContactSelfConsistency(
          results[i], "kin_link", pose1, pose2, "Seg" + std::to_string(seg) + "[" + std::to_string(i) + "]");
  }

  // The trajectory passes through a 2m octree centered at origin ([-1,1]^3).
  // At least the middle segments must find contacts.
  EXPECT_GT(segments_found, 0) << "No contacts found in any segment of a sweep through the octree";
  EXPECT_GT(total_contacts, 0) << "Zero total contacts across " << num_segments << " segments";
}

/// Pairwise comparison: both managers must find contacts in the same segments.
inline void runTestScenarioE_Comparison(ContinuousContactManager& checker_a,
                                        ContinuousContactManager& checker_b,
                                        const std::string& label_a,
                                        const std::string& label_b)
{
  const int num_segments = 6;

  // Setup both managers identically
  auto setup = [&](ContinuousContactManager& checker) {
    addOctree(checker, "octomap_link");
    addBoxLink(checker, "kin_link", Eigen::Vector3d(0.15, 0.15, 0.15));
    checker.setActiveCollisionObjects(std::vector<std::string>{ "kin_link" });
    checker.setDefaultCollisionMargin(0.25);
    checker.setCollisionObjectsTransform("octomap_link", Eigen::Isometry3d::Identity());
  };
  setup(checker_a);
  setup(checker_b);

  Eigen::Isometry3d full_start = Eigen::Isometry3d::Identity();
  full_start.translation() = Eigen::Vector3d(-3.0, 0, 0);
  Eigen::Isometry3d full_end = Eigen::Isometry3d::Identity();
  full_end.translation() = Eigen::Vector3d(3.0, 0, 0);
  full_end.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  auto interpolate = [&](double t) -> Eigen::Isometry3d {
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translation() = (1.0 - t) * full_start.translation() + t * full_end.translation();
    Eigen::Quaterniond q0(full_start.rotation());
    Eigen::Quaterniond q1(full_end.rotation());
    result.linear() = q0.slerp(t, q1).toRotationMatrix();
    return result;
  };

  int segments_a_only = 0;

  for (int seg = 0; seg < num_segments; ++seg)
  {
    double t0 = static_cast<double>(seg) / num_segments;
    double t1 = static_cast<double>(seg + 1) / num_segments;
    Eigen::Isometry3d pose1 = interpolate(t0);
    Eigen::Isometry3d pose2 = interpolate(t1);

    checker_a.setCollisionObjectsTransform("kin_link", pose1, pose2);
    checker_b.setCollisionObjectsTransform("kin_link", pose1, pose2);

    auto a_results = runContact(checker_a);
    auto b_results = runContact(checker_b);

    if (!a_results.empty() && b_results.empty())
      segments_a_only++;
  }

  EXPECT_EQ(segments_a_only, 0) << label_b << " missed contacts in " << segments_a_only << " sub-segments where "
                                << label_a << " found them. This would cause the optimizer to see zero penalties "
                                << "for those segments.";
}

// ---------------------------------------------------------------------------
// Scenario F: Near-miss sweep with large margin
// ---------------------------------------------------------------------------

inline void runTestScenarioF_NearMiss(ContinuousContactManager& checker)
{
  const double margin = 0.5;

  addOctree(checker, "octomap_link");
  addBoxLink(checker, "kin_link", Eigen::Vector3d(0.1, 0.1, 0.1));
  checker.setActiveCollisionObjects(std::vector<std::string>{ "kin_link" });
  checker.setDefaultCollisionMargin(margin);
  checker.setCollisionObjectsTransform("octomap_link", Eigen::Isometry3d::Identity());

  // Sweep parallel to the octree face at Y = 1.0 + gap, where gap < margin.
  const double gap = 0.2;
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() = Eigen::Vector3d(-0.5, 1.0 + gap, 0);
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(0.5, 1.0 + gap, 0);

  checker.setCollisionObjectsTransform("kin_link", pose1, pose2);
  auto results = runContact(checker);

  // Near-miss contacts within margin should be detected
  ASSERT_FALSE(results.empty()) << "No contacts found for near-miss sweep (gap=" << gap << " < margin=" << margin
                                << "). Broadphase AABB expansion may be insufficient.";

  for (std::size_t i = 0; i < results.size(); ++i)
    validateOctreeContactSelfConsistency(results[i], "kin_link", pose1, pose2, "Result[" + std::to_string(i) + "]");
}

// ---------------------------------------------------------------------------
// Scenario G: Repeated contactTest stability
// ---------------------------------------------------------------------------

inline void runTestScenarioG_RepeatedStability(ContinuousContactManager& checker)
{
  addOctree(checker, "octomap_link");
  addBoxLink(checker, "kin_link", Eigen::Vector3d(0.1, 0.1, 0.1));
  checker.setActiveCollisionObjects(std::vector<std::string>{ "kin_link" });
  checker.setDefaultCollisionMargin(0.1);
  checker.setCollisionObjectsTransform("octomap_link", Eigen::Isometry3d::Identity());

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() = Eigen::Vector3d(-2.0, 0, 0);
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(0.0, 0, 0);
  checker.setCollisionObjectsTransform("kin_link", pose1, pose2);

  int prev_count = -1;
  for (int call = 0; call < 5; ++call)
  {
    auto results = runContact(checker);
    int count = static_cast<int>(results.size());

    if (prev_count >= 0)
    {
      EXPECT_EQ(count, prev_count) << "Contact count changed between repeated contactTest calls (" << prev_count
                                   << " -> " << count << "). This suggests internal state corruption.";
    }
    prev_count = count;

    EXPECT_GT(count, 0) << "Call " << call << " returned zero contacts for a sweep through the octree";
  }
}

}  // namespace detail

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

inline void runTestScenarioA_RotationalSweep(ContinuousContactManager& checker)
{
  detail::runTestScenarioA_RotationalSweep(checker);
}

inline void runTestScenarioB_MultiShapeSweep(ContinuousContactManager& checker)
{
  detail::runTestScenarioB_MultiShapeSweep(checker);
}

inline void runTestScenarioC_LargeMargin(ContinuousContactManager& checker)
{
  detail::runTestScenarioC_LargeMargin(checker);
}

inline void runTestScenarioD_ArticulatedArm(ContinuousContactManager& checker)
{
  detail::runTestScenarioD_ArticulatedArm(checker);
}

inline void runTestScenarioD_Comparison(ContinuousContactManager& checker_a,
                                        ContinuousContactManager& checker_b,
                                        const std::string& label_a,
                                        const std::string& label_b)
{
  detail::runTestScenarioD_Comparison(checker_a, checker_b, label_a, label_b);
}

inline void runTestScenarioE_SubdivisionLoop(ContinuousContactManager& checker)
{
  detail::runTestScenarioE_SubdivisionLoop(checker);
}

inline void runTestScenarioE_Comparison(ContinuousContactManager& checker_a,
                                        ContinuousContactManager& checker_b,
                                        const std::string& label_a,
                                        const std::string& label_b)
{
  detail::runTestScenarioE_Comparison(checker_a, checker_b, label_a, label_b);
}

inline void runTestScenarioF_NearMiss(ContinuousContactManager& checker) { detail::runTestScenarioF_NearMiss(checker); }

inline void runTestScenarioG_RepeatedStability(ContinuousContactManager& checker)
{
  detail::runTestScenarioG_RepeatedStability(checker);
}

}  // namespace tesseract::collision::test_suite
#endif  // TESSERACT_COLLISION_COLLISION_CAST_SCENARIO_UNIT_HPP
