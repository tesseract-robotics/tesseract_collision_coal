#ifndef TESSERACT_COLLISION_COLLISION_SPHERE_SPHERE_CAST_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_SPHERE_SPHERE_CAST_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iomanip>
#include <sstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/convex_hull_utils.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/common.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/ply_io.h>

namespace tesseract::collision::test_suite
{
namespace detail
{
inline void addCollisionObjects(ContinuousContactManager& checker, bool use_convex_mesh = false)
{
  tesseract::common::GeneralResourceLocator locator;

  ////////////////////////
  // Add sphere to checker
  ////////////////////////
  CollisionShapePtr sphere;
  if (use_convex_mesh)
  {
    auto mesh_vertices = std::make_shared<tesseract::common::VectorVector3d>();
    auto mesh_faces = std::make_shared<Eigen::VectorXi>();
    EXPECT_GT(tesseract::common::loadSimplePlyFile(
                  locator.locateResource("package://tesseract/support/meshes/sphere_p25m.ply")->getFilePath(),
                  *mesh_vertices,
                  *mesh_faces,
                  true),
              0);

    auto mesh = std::make_shared<tesseract::geometry::Mesh>(mesh_vertices, mesh_faces);
    sphere = makeConvexMesh(*mesh);
  }
  else
  {
    sphere = std::make_shared<tesseract::geometry::Sphere>(0.25);
  }

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();
  sphere_pose.translation()[2] = 0.25;

  CollisionShapesConst obj1_shapes;
  tesseract::common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(sphere);
  obj1_poses.push_back(sphere_pose);

  checker.addCollisionObject("sphere_link", 0, obj1_shapes, obj1_poses, false);
  EXPECT_FALSE(checker.isCollisionObjectEnabled("sphere_link"));
  checker.enableCollisionObject("sphere_link");
  EXPECT_TRUE(checker.isCollisionObjectEnabled("sphere_link"));

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
  EXPECT_TRUE(checker.isCollisionObjectEnabled("thin_box_link"));
  checker.disableCollisionObject("thin_box_link");
  EXPECT_FALSE(checker.isCollisionObjectEnabled("thin_box_link"));

  /////////////////////////////////////////////////////////////////
  // Add second sphere to checker. If use_convex_mesh = true
  // then this sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr sphere1;

  if (use_convex_mesh)
  {
    auto mesh_vertices = std::make_shared<tesseract::common::VectorVector3d>();
    auto mesh_faces = std::make_shared<Eigen::VectorXi>();
    EXPECT_GT(tesseract::common::loadSimplePlyFile(
                  locator.locateResource("package://tesseract/support/meshes/sphere_p25m.ply")->getFilePath(),
                  *mesh_vertices,
                  *mesh_faces,
                  true),
              0);

    auto mesh = std::make_shared<tesseract::geometry::Mesh>(mesh_vertices, mesh_faces);
    sphere1 = makeConvexMesh(*mesh);
  }
  else
  {
    sphere1 = std::make_shared<tesseract::geometry::Sphere>(0.25);
  }

  Eigen::Isometry3d sphere1_pose;
  sphere1_pose.setIdentity();
  sphere1_pose.translation()[2] = 0.25;

  CollisionShapesConst obj3_shapes;
  tesseract::common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(sphere1);
  obj3_poses.push_back(sphere1_pose);

  checker.addCollisionObject("sphere1_link", 0, obj3_shapes, obj3_poses);
  EXPECT_TRUE(checker.isCollisionObjectEnabled("sphere1_link"));

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
  EXPECT_TRUE(checker.getCollisionObjects().size() == 3);

  /////////////////////////////////////////////
  // Try functions on a link that does not exist
  /////////////////////////////////////////////
  EXPECT_FALSE(checker.removeCollisionObject("link_does_not_exist"));
  EXPECT_FALSE(checker.enableCollisionObject("link_does_not_exist"));
  EXPECT_FALSE(checker.disableCollisionObject("link_does_not_exist"));
  EXPECT_FALSE(checker.isCollisionObjectEnabled("link_does_not_exist"));

  /////////////////////////////////////////////
  // Try to add empty Collision Object
  /////////////////////////////////////////////
  EXPECT_FALSE(
      checker.addCollisionObject("empty_link", 0, CollisionShapesConst(), tesseract::common::VectorIsometry3d()));
  EXPECT_TRUE(checker.getCollisionObjects().size() == 3);
}

inline std::string formatContactResult(const ContactResult& cr)
{
  std::ostringstream os;
  os << std::setprecision(6) << std::fixed;
  os << "Contact result state:"
     << "\n  link_names: [" << cr.link_names[0] << ", " << cr.link_names[1] << "]"
     << "\n  distance: " << cr.distance << "\n  normal: (" << cr.normal[0] << ", " << cr.normal[1] << ", "
     << cr.normal[2] << ")"
     << "\n  nearest_points[0]: (" << cr.nearest_points[0][0] << ", " << cr.nearest_points[0][1] << ", "
     << cr.nearest_points[0][2] << ")"
     << "\n  nearest_points[1]: (" << cr.nearest_points[1][0] << ", " << cr.nearest_points[1][1] << ", "
     << cr.nearest_points[1][2] << ")"
     << "\n  nearest_points_local[0]: (" << cr.nearest_points_local[0][0] << ", " << cr.nearest_points_local[0][1]
     << ", " << cr.nearest_points_local[0][2] << ")"
     << "\n  nearest_points_local[1]: (" << cr.nearest_points_local[1][0] << ", " << cr.nearest_points_local[1][1]
     << ", " << cr.nearest_points_local[1][2] << ")"
     << "\n  cc_time: [" << cr.cc_time[0] << ", " << cr.cc_time[1] << "]"
     << "\n  cc_type: [" << static_cast<int>(cr.cc_type[0]) << ", " << static_cast<int>(cr.cc_type[1]) << "]";
  return os.str();
}

inline void runTestPrimitive(ContinuousContactManager& checker)
{
  ///////////////////////////////////////////////////
  // Test when object is in collision at cc_time 0.5
  // Both spheres (r=0.25) sweep through each other:
  //   sphere_link:  (-0.2, -1.0, 0) -> (-0.2, 1.0, 0)  (sweeps along Y)
  //   sphere1_link: (0.2, 0, -1.0)  -> (0.2, 0, 1.0)   (sweeps along Z)
  // At t=0.5 both pass through Y=0/Z=0, separated by 0.4 in X.
  // With r=0.25 each, they overlap by 0.1 at t=0.5.
  ///////////////////////////////////////////////////
  std::vector<std::string> active_links{ "sphere_link", "sphere1_link" };
  checker.setActiveCollisionObjects(active_links);
  std::vector<std::string> check_active_links = checker.getActiveCollisionObjects();
  EXPECT_TRUE(tesseract::common::isIdentical<std::string>(active_links, check_active_links, false));

  EXPECT_TRUE(checker.getContactAllowedValidator() == nullptr);

  checker.setCollisionMarginData(CollisionMarginData(0.1));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);

  // Set the start location
  tesseract::common::TransformMap location_start;
  location_start["sphere_link"] = Eigen::Isometry3d::Identity();
  location_start["sphere_link"].translation()(0) = -0.2;
  location_start["sphere_link"].translation()(1) = -1.0;

  location_start["sphere1_link"] = Eigen::Isometry3d::Identity();
  location_start["sphere1_link"].translation()(0) = 0.2;
  location_start["sphere1_link"].translation()(2) = -1.0;

  // Set the end location
  tesseract::common::TransformMap location_end;
  location_end["sphere_link"] = Eigen::Isometry3d::Identity();
  location_end["sphere_link"].translation()(0) = -0.2;
  location_end["sphere_link"].translation()(1) = 1.0;

  location_end["sphere1_link"] = Eigen::Isometry3d::Identity();
  location_end["sphere1_link"].translation()(0) = 0.2;
  location_end["sphere1_link"].translation()(2) = 1.0;

  checker.setCollisionObjectsTransform(location_start, location_end);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty()) << "Scenario 1 (symmetric cc_time=0.5): No contacts found. "
                                      << "sphere_link (r=0.25) sweeps Y=-1->1 at x=-0.2, "
                                      << "sphere1_link (r=0.25) sweeps Z=-1->1 at x=0.2. "
                                      << "At t=0.5, centers are 0.4 apart (< 2*r=0.5), so collision is expected.";

  const auto& cr1 = result_vector[0];
  SCOPED_TRACE("Scenario 1 (cc_time=0.5): " + formatContactResult(cr1));

  EXPECT_NEAR(cr1.distance, -0.1, 0.0001) << "Penetration should be -0.1 (sphere separation 0.4, combined radii 0.5)";

  std::vector<int> idx = { 0, 1, 1 };
  if (cr1.link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  const std::string sphere_slot = (idx[0] == 0) ? "slot 0" : "slot 1";
  const std::string sphere1_slot = (idx[1] == 0) ? "slot 0" : "slot 1";

  EXPECT_NEAR(cr1.cc_time[static_cast<size_t>(idx[0])], 0.5, 0.001)
      << "sphere_link (" << sphere_slot << ") cc_time should be 0.5 (collision at midpoint of sweep)";
  EXPECT_NEAR(cr1.cc_time[static_cast<size_t>(idx[1])], 0.5, 0.001)
      << "sphere1_link (" << sphere1_slot << ") cc_time should be 0.5 (collision at midpoint of sweep)";

  EXPECT_EQ(cr1.cc_type[static_cast<size_t>(idx[0])], ContinuousCollisionType::CCType_Between)
      << "sphere_link (" << sphere_slot << ") cc_type should be CCType_Between (3), "
      << "got " << static_cast<int>(cr1.cc_type[static_cast<size_t>(idx[0])]);
  EXPECT_EQ(cr1.cc_type[static_cast<size_t>(idx[1])], ContinuousCollisionType::CCType_Between)
      << "sphere1_link (" << sphere1_slot << ") cc_type should be CCType_Between (3), "
      << "got " << static_cast<int>(cr1.cc_type[static_cast<size_t>(idx[1])]);

  // World-frame nearest points at collision time
  EXPECT_NEAR(cr1.nearest_points[static_cast<size_t>(idx[0])][0], 0.05, 0.001) << "sphere_link nearest_point.x: "
                                                                                  "midpoint between centers (-0.2 and "
                                                                                  "0.2) minus half penetration";
  EXPECT_NEAR(cr1.nearest_points[static_cast<size_t>(idx[0])][1], 0.0, 0.001) << "sphere_link nearest_point.y: at y=0 "
                                                                                 "(midpoint of Y-sweep at t=0.5)";
  EXPECT_NEAR(cr1.nearest_points[static_cast<size_t>(idx[0])][2], 0.25, 0.001) << "sphere_link nearest_point.z: offset "
                                                                                  "by sphere_pose z=0.25";

  EXPECT_NEAR(cr1.nearest_points[static_cast<size_t>(idx[1])][0], -0.05, 0.001) << "sphere1_link nearest_point.x";
  EXPECT_NEAR(cr1.nearest_points[static_cast<size_t>(idx[1])][1], 0.0, 0.001) << "sphere1_link nearest_point.y: at y=0 "
                                                                                 "(sphere1 doesn't move in Y)";
  EXPECT_NEAR(cr1.nearest_points[static_cast<size_t>(idx[1])][2], 0.25, 0.001) << "sphere1_link nearest_point.z: "
                                                                                  "offset by sphere_pose z=0.25";

  // Local-frame nearest points
  EXPECT_NEAR(cr1.nearest_points_local[static_cast<size_t>(idx[0])][0], 0.25, 0.001) << "sphere_link "
                                                                                        "nearest_point_local.x: sphere "
                                                                                        "surface at +x (toward other "
                                                                                        "sphere)";
  EXPECT_NEAR(cr1.nearest_points_local[static_cast<size_t>(idx[0])][1], 0.0, 0.001) << "sphere_link "
                                                                                       "nearest_point_local.y";
  EXPECT_NEAR(cr1.nearest_points_local[static_cast<size_t>(idx[0])][2], 0.25, 0.001) << "sphere_link "
                                                                                        "nearest_point_local.z: "
                                                                                        "sphere_pose offset";
  EXPECT_NEAR(cr1.nearest_points_local[static_cast<size_t>(idx[1])][0], -0.25, 0.001) << "sphere1_link "
                                                                                         "nearest_point_local.x: "
                                                                                         "sphere surface at -x (toward "
                                                                                         "other sphere)";
  EXPECT_NEAR(cr1.nearest_points_local[static_cast<size_t>(idx[1])][1], 0.0, 0.001) << "sphere1_link "
                                                                                       "nearest_point_local.y";
  EXPECT_NEAR(cr1.nearest_points_local[static_cast<size_t>(idx[1])][2], 0.25, 0.001) << "sphere1_link "
                                                                                        "nearest_point_local.z: "
                                                                                        "sphere_pose offset";

  // Verify start/end transforms are stored correctly
  EXPECT_TRUE(cr1.transform[static_cast<size_t>(idx[0])].isApprox(location_start["sphere_link"], 0.0001)) << "sphere_"
                                                                                                             "link "
                                                                                                             "transform"
                                                                                                             " should "
                                                                                                             "match "
                                                                                                             "start "
                                                                                                             "pose "
                                                                                                             "(-0.2, "
                                                                                                             "-1, 0)";
  EXPECT_TRUE(cr1.transform[static_cast<size_t>(idx[1])].isApprox(location_start["sphere1_link"], 0.0001)) << "sphere1_"
                                                                                                              "link "
                                                                                                              "transfor"
                                                                                                              "m "
                                                                                                              "should "
                                                                                                              "match "
                                                                                                              "start "
                                                                                                              "pose "
                                                                                                              "(0.2, "
                                                                                                              "0, -1)";
  EXPECT_TRUE(cr1.cc_transform[static_cast<size_t>(idx[0])].isApprox(location_end["sphere_link"], 0.0001)) << "sphere_"
                                                                                                              "link "
                                                                                                              "cc_"
                                                                                                              "transfor"
                                                                                                              "m "
                                                                                                              "should "
                                                                                                              "match "
                                                                                                              "end "
                                                                                                              "pose "
                                                                                                              "(-0.2, "
                                                                                                              "1, 0)";
  EXPECT_TRUE(cr1.cc_transform[static_cast<size_t>(idx[1])].isApprox(location_end["sphere1_link"], 0.0001)) << "sphere1"
                                                                                                               "_link "
                                                                                                               "cc_"
                                                                                                               "transfo"
                                                                                                               "rm "
                                                                                                               "should "
                                                                                                               "match "
                                                                                                               "end "
                                                                                                               "pose "
                                                                                                               "(0.2, "
                                                                                                               "0, 1)";

  // Contact normal should point along X (spheres separated in X direction)
  EXPECT_NEAR(cr1.normal[0], idx[2] * 1.0, 0.001) << "Contact normal x: should point along X axis (sphere separation "
                                                     "direction)";
  EXPECT_NEAR(cr1.normal[1], idx[2] * 0.0, 0.001) << "Contact normal y: should be ~0 (no Y separation between sphere "
                                                     "centers)";
  EXPECT_NEAR(cr1.normal[2], idx[2] * 0.0, 0.001) << "Contact normal z: should be ~0 (no Z separation between sphere "
                                                     "centers at contact)";

  /////////////////////////////////////////////////////////////
  // Test when object is in collision at cc_time 0.333 and 0.5
  // sphere_link now starts closer: sweeps Y=-0.5->1.0 (1.5 total)
  // sphere1_link sweeps Z=-1.0->1.0 as before (2.0 total)
  // sphere_link reaches Y=0 at t=0.5/1.5=0.333
  // sphere1_link reaches Z=0 at t=1.0/2.0=0.5
  /////////////////////////////////////////////////////////////

  // Set the start location
  location_start["sphere_link"] = Eigen::Isometry3d::Identity();
  location_start["sphere_link"].translation()(0) = -0.2;
  location_start["sphere_link"].translation()(1) = -0.5;

  location_start["sphere1_link"] = Eigen::Isometry3d::Identity();
  location_start["sphere1_link"].translation()(0) = 0.2;
  location_start["sphere1_link"].translation()(2) = -1.0;

  // Set the end location
  location_end["sphere_link"] = Eigen::Isometry3d::Identity();
  location_end["sphere_link"].translation()(0) = -0.2;
  location_end["sphere_link"].translation()(1) = 1.0;

  location_end["sphere1_link"] = Eigen::Isometry3d::Identity();
  location_end["sphere1_link"].translation()(0) = 0.2;
  location_end["sphere1_link"].translation()(2) = 1.0;

  checker.setCollisionObjectsTransform(location_start, location_end);

  // Perform collision check
  result.clear();
  result_vector.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));

  result.flattenCopyResults(result_vector);

  ASSERT_FALSE(result_vector.empty()) << "Scenario 2 (asymmetric cc_time 0.333/0.5): No contacts found. "
                                      << "sphere_link sweeps Y=-0.5->1.0, sphere1_link sweeps Z=-1.0->1.0. "
                                      << "Collision is expected when sweeps cross.";

  const auto& cr2 = result_vector[0];
  SCOPED_TRACE("Scenario 2 (cc_time 0.333/0.5): " + formatContactResult(cr2));

  EXPECT_NEAR(cr2.distance, -0.1, 0.0001) << "Penetration should be -0.1 (same sphere geometry, same X separation)";

  idx = { 0, 1, 1 };
  if (cr2.link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  const std::string sphere_slot2 = (idx[0] == 0) ? "slot 0" : "slot 1";
  const std::string sphere1_slot2 = (idx[1] == 0) ? "slot 0" : "slot 1";

  EXPECT_NEAR(cr2.cc_time[static_cast<size_t>(idx[0])], 0.3333, 0.001)
      << "sphere_link (" << sphere_slot2 << ") cc_time should be ~0.333 "
      << "(Y=0 reached at 0.5/1.5 of sweep from Y=-0.5 to Y=1.0)";
  EXPECT_NEAR(cr2.cc_time[static_cast<size_t>(idx[1])], 0.5, 0.001)
      << "sphere1_link (" << sphere1_slot2 << ") cc_time should be 0.5 "
      << "(Z=0 reached at 1.0/2.0 of sweep from Z=-1 to Z=1)";

  EXPECT_EQ(cr2.cc_type[static_cast<size_t>(idx[0])], ContinuousCollisionType::CCType_Between)
      << "sphere_link (" << sphere_slot2 << ") cc_type should be CCType_Between (3), "
      << "got " << static_cast<int>(cr2.cc_type[static_cast<size_t>(idx[0])]);
  EXPECT_EQ(cr2.cc_type[static_cast<size_t>(idx[1])], ContinuousCollisionType::CCType_Between)
      << "sphere1_link (" << sphere1_slot2 << ") cc_type should be CCType_Between (3), "
      << "got " << static_cast<int>(cr2.cc_type[static_cast<size_t>(idx[1])]);

  // World-frame nearest points
  EXPECT_NEAR(cr2.nearest_points[static_cast<size_t>(idx[0])][0], 0.05, 0.001) << "sphere_link nearest_point.x";
  EXPECT_NEAR(cr2.nearest_points[static_cast<size_t>(idx[0])][1], 0.0, 0.001) << "sphere_link nearest_point.y: at Y=0 "
                                                                                 "(crossing point)";
  EXPECT_NEAR(cr2.nearest_points[static_cast<size_t>(idx[0])][2], 0.25, 0.001) << "sphere_link nearest_point.z: "
                                                                                  "sphere_pose z=0.25 offset";

  EXPECT_NEAR(cr2.nearest_points[static_cast<size_t>(idx[1])][0], -0.05, 0.001) << "sphere1_link nearest_point.x";
  EXPECT_NEAR(cr2.nearest_points[static_cast<size_t>(idx[1])][1], 0.0, 0.001) << "sphere1_link nearest_point.y";
  EXPECT_NEAR(cr2.nearest_points[static_cast<size_t>(idx[1])][2], 0.25, 0.001) << "sphere1_link nearest_point.z: "
                                                                                  "sphere_pose z=0.25 offset";

  // Local-frame nearest points
  EXPECT_NEAR(cr2.nearest_points_local[static_cast<size_t>(idx[0])][0], 0.25, 0.001) << "sphere_link "
                                                                                        "nearest_point_local.x: sphere "
                                                                                        "surface at +x";
  EXPECT_NEAR(cr2.nearest_points_local[static_cast<size_t>(idx[0])][1], 0.0, 0.001) << "sphere_link "
                                                                                       "nearest_point_local.y";
  EXPECT_NEAR(cr2.nearest_points_local[static_cast<size_t>(idx[0])][2], 0.25, 0.001) << "sphere_link "
                                                                                        "nearest_point_local.z";
  EXPECT_NEAR(cr2.nearest_points_local[static_cast<size_t>(idx[1])][0], -0.25, 0.001) << "sphere1_link "
                                                                                         "nearest_point_local.x: "
                                                                                         "sphere surface at -x";
  EXPECT_NEAR(cr2.nearest_points_local[static_cast<size_t>(idx[1])][1], 0.0, 0.001) << "sphere1_link "
                                                                                       "nearest_point_local.y";
  EXPECT_NEAR(cr2.nearest_points_local[static_cast<size_t>(idx[1])][2], 0.25, 0.001) << "sphere1_link "
                                                                                        "nearest_point_local.z";

  // Verify transforms
  EXPECT_TRUE(cr2.transform[static_cast<size_t>(idx[0])].isApprox(location_start["sphere_link"], 0.0001)) << "sphere_"
                                                                                                             "link "
                                                                                                             "transform"
                                                                                                             " should "
                                                                                                             "match "
                                                                                                             "start "
                                                                                                             "pose "
                                                                                                             "(-0.2, "
                                                                                                             "-0.5, 0)";
  EXPECT_TRUE(cr2.transform[static_cast<size_t>(idx[1])].isApprox(location_start["sphere1_link"], 0.0001)) << "sphere1_"
                                                                                                              "link "
                                                                                                              "transfor"
                                                                                                              "m "
                                                                                                              "should "
                                                                                                              "match "
                                                                                                              "start "
                                                                                                              "pose "
                                                                                                              "(0.2, "
                                                                                                              "0, -1)";
  EXPECT_TRUE(cr2.cc_transform[static_cast<size_t>(idx[0])].isApprox(location_end["sphere_link"], 0.0001)) << "sphere_"
                                                                                                              "link "
                                                                                                              "cc_"
                                                                                                              "transfor"
                                                                                                              "m "
                                                                                                              "should "
                                                                                                              "match "
                                                                                                              "end "
                                                                                                              "pose "
                                                                                                              "(-0.2, "
                                                                                                              "1, 0)";
  EXPECT_TRUE(cr2.cc_transform[static_cast<size_t>(idx[1])].isApprox(location_end["sphere1_link"], 0.0001)) << "sphere1"
                                                                                                               "_link "
                                                                                                               "cc_"
                                                                                                               "transfo"
                                                                                                               "rm "
                                                                                                               "should "
                                                                                                               "match "
                                                                                                               "end "
                                                                                                               "pose "
                                                                                                               "(0.2, "
                                                                                                               "0, 1)";

  // Contact normal
  EXPECT_NEAR(cr2.normal[0], idx[2] * 1.0, 0.001) << "Contact normal x: should point along X (sphere separation "
                                                     "direction)";
  EXPECT_NEAR(cr2.normal[1], idx[2] * 0.0, 0.001) << "Contact normal y: should be ~0";
  EXPECT_NEAR(cr2.normal[2], idx[2] * 0.0, 0.001) << "Contact normal z: should be ~0";
}

inline void runTestConvex(ContinuousContactManager& checker)
{
  ///////////////////////////////////////////////////
  // Test when object is in collision at cc_time 0.5
  // Same geometry as primitive test but using convex hull mesh approximation
  // of spheres. Expected values differ slightly due to tessellation.
  //   sphere_link:  (-0.2, -1.0, 0) -> (-0.2, 1.0, 0)
  //   sphere1_link: (0.2, 0, -1.0)  -> (0.2, 0, 1.0)
  ///////////////////////////////////////////////////
  std::vector<std::string> active_links{ "sphere_link", "sphere1_link" };
  checker.setActiveCollisionObjects(active_links);
  std::vector<std::string> check_active_links = checker.getActiveCollisionObjects();
  EXPECT_TRUE(tesseract::common::isIdentical<std::string>(active_links, check_active_links, false));

  EXPECT_TRUE(checker.getContactAllowedValidator() == nullptr);

  checker.setCollisionMarginData(CollisionMarginData(0.1));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);

  // Set the start location
  tesseract::common::TransformMap location_start;
  location_start["sphere_link"] = Eigen::Isometry3d::Identity();
  location_start["sphere_link"].translation()(0) = -0.2;
  location_start["sphere_link"].translation()(1) = -1.0;

  location_start["sphere1_link"] = Eigen::Isometry3d::Identity();
  location_start["sphere1_link"].translation()(0) = 0.2;
  location_start["sphere1_link"].translation()(2) = -1.0;

  // Set the end location
  tesseract::common::TransformMap location_end;
  location_end["sphere_link"] = Eigen::Isometry3d::Identity();
  location_end["sphere_link"].translation()(0) = -0.2;
  location_end["sphere_link"].translation()(1) = 1.0;

  location_end["sphere1_link"] = Eigen::Isometry3d::Identity();
  location_end["sphere1_link"].translation()(0) = 0.2;
  location_end["sphere1_link"].translation()(2) = 1.0;

  checker.setCollisionObjectsTransform(location_start, location_end);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty()) << "Scenario 1 convex (symmetric cc_time=0.5): No contacts found. "
                                      << "Convex hull sphere_link sweeps Y=-1->1, sphere1_link sweeps Z=-1->1. "
                                      << "At t=0.5 centers are 0.4 apart, combined mesh radii ~0.25 each, collision "
                                         "expected.";

  const auto& cr1 = result_vector[0];
  SCOPED_TRACE("Scenario 1 convex (cc_time=0.5): " + formatContactResult(cr1));

  // Convex mesh approximation gives slightly less penetration than exact sphere
  EXPECT_NEAR(cr1.distance, -0.0754, 0.001) << "Penetration for convex mesh spheres (tessellation reduces effective "
                                               "radius slightly)";

  std::vector<int> idx = { 0, 1, 1 };
  if (cr1.link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  const std::string sphere_slot = (idx[0] == 0) ? "slot 0" : "slot 1";
  const std::string sphere1_slot = (idx[1] == 0) ? "slot 0" : "slot 1";

  EXPECT_NEAR(cr1.cc_time[static_cast<size_t>(idx[0])], 0.5, 0.001)
      << "sphere_link (" << sphere_slot << ") cc_time should be 0.5 (collision at midpoint)";
  EXPECT_NEAR(cr1.cc_time[static_cast<size_t>(idx[1])], 0.5, 0.001)
      << "sphere1_link (" << sphere1_slot << ") cc_time should be 0.5 (collision at midpoint)";

  EXPECT_EQ(cr1.cc_type[static_cast<size_t>(idx[0])], ContinuousCollisionType::CCType_Between)
      << "sphere_link (" << sphere_slot << ") cc_type should be CCType_Between (3), "
      << "got " << static_cast<int>(cr1.cc_type[static_cast<size_t>(idx[0])]);
  EXPECT_EQ(cr1.cc_type[static_cast<size_t>(idx[1])], ContinuousCollisionType::CCType_Between)
      << "sphere1_link (" << sphere1_slot << ") cc_type should be CCType_Between (3), "
      << "got " << static_cast<int>(cr1.cc_type[static_cast<size_t>(idx[1])]);

  // World-frame nearest points (convex mesh values)
  EXPECT_NEAR(cr1.nearest_points[static_cast<size_t>(idx[0])][0], 0.0377, 0.001) << "sphere_link nearest_point.x "
                                                                                    "(convex mesh)";
  EXPECT_NEAR(cr1.nearest_points[static_cast<size_t>(idx[0])][1], 0.0, 0.001) << "sphere_link nearest_point.y: at y=0 "
                                                                                 "(midpoint of sweep at t=0.5)";
  EXPECT_NEAR(cr1.nearest_points[static_cast<size_t>(idx[0])][2], 0.25, 0.001) << "sphere_link nearest_point.z: "
                                                                                  "sphere_pose z=0.25 offset";

  EXPECT_NEAR(cr1.nearest_points[static_cast<size_t>(idx[1])][0], -0.0377, 0.001) << "sphere1_link nearest_point.x "
                                                                                     "(convex mesh)";
  EXPECT_NEAR(cr1.nearest_points[static_cast<size_t>(idx[1])][1], 0.0, 0.001) << "sphere1_link nearest_point.y";
  EXPECT_NEAR(cr1.nearest_points[static_cast<size_t>(idx[1])][2], 0.25, 0.001) << "sphere1_link nearest_point.z: "
                                                                                  "sphere_pose z=0.25 offset";

  // Local-frame nearest points (convex mesh values)
  EXPECT_NEAR(cr1.nearest_points_local[static_cast<size_t>(idx[0])][0], 0.2377, 0.001) << "sphere_link "
                                                                                          "nearest_point_local.x "
                                                                                          "(convex mesh surface at +x)";
  EXPECT_NEAR(cr1.nearest_points_local[static_cast<size_t>(idx[0])][1], 0.0, 0.001) << "sphere_link "
                                                                                       "nearest_point_local.y";
  EXPECT_NEAR(cr1.nearest_points_local[static_cast<size_t>(idx[0])][2], 0.25, 0.001) << "sphere_link "
                                                                                        "nearest_point_local.z";
  EXPECT_NEAR(cr1.nearest_points_local[static_cast<size_t>(idx[1])][0], -0.2377, 0.001) << "sphere1_link "
                                                                                           "nearest_point_local.x "
                                                                                           "(convex mesh surface at "
                                                                                           "-x)";
  EXPECT_NEAR(cr1.nearest_points_local[static_cast<size_t>(idx[1])][1], 0.0, 0.001) << "sphere1_link "
                                                                                       "nearest_point_local.y";
  EXPECT_NEAR(cr1.nearest_points_local[static_cast<size_t>(idx[1])][2], 0.25, 0.001) << "sphere1_link "
                                                                                        "nearest_point_local.z";

  // Verify transforms
  EXPECT_TRUE(cr1.transform[static_cast<size_t>(idx[0])].isApprox(location_start["sphere_link"], 0.0001)) << "sphere_"
                                                                                                             "link "
                                                                                                             "transform"
                                                                                                             " should "
                                                                                                             "match "
                                                                                                             "start "
                                                                                                             "pose "
                                                                                                             "(-0.2, "
                                                                                                             "-1, 0)";
  EXPECT_TRUE(cr1.transform[static_cast<size_t>(idx[1])].isApprox(location_start["sphere1_link"], 0.0001)) << "sphere1_"
                                                                                                              "link "
                                                                                                              "transfor"
                                                                                                              "m "
                                                                                                              "should "
                                                                                                              "match "
                                                                                                              "start "
                                                                                                              "pose "
                                                                                                              "(0.2, "
                                                                                                              "0, -1)";
  EXPECT_TRUE(cr1.cc_transform[static_cast<size_t>(idx[0])].isApprox(location_end["sphere_link"], 0.0001)) << "sphere_"
                                                                                                              "link "
                                                                                                              "cc_"
                                                                                                              "transfor"
                                                                                                              "m "
                                                                                                              "should "
                                                                                                              "match "
                                                                                                              "end "
                                                                                                              "pose "
                                                                                                              "(-0.2, "
                                                                                                              "1, 0)";
  EXPECT_TRUE(cr1.cc_transform[static_cast<size_t>(idx[1])].isApprox(location_end["sphere1_link"], 0.0001)) << "sphere1"
                                                                                                               "_link "
                                                                                                               "cc_"
                                                                                                               "transfo"
                                                                                                               "rm "
                                                                                                               "should "
                                                                                                               "match "
                                                                                                               "end "
                                                                                                               "pose "
                                                                                                               "(0.2, "
                                                                                                               "0, 1)";

  // Contact normal
  EXPECT_NEAR(cr1.normal[0], idx[2] * 1.0, 0.001) << "Contact normal x: should point along X (sphere separation "
                                                     "direction)";
  EXPECT_NEAR(cr1.normal[1], idx[2] * 0.0, 0.001) << "Contact normal y: should be ~0";
  EXPECT_NEAR(cr1.normal[2], idx[2] * 0.0, 0.001) << "Contact normal z: should be ~0";

  /////////////////////////////////////////////////////////////
  // Test when object is in collision at cc_time ~0.385 and 0.5
  // sphere_link now starts closer: sweeps Y=-0.5->1.0
  // sphere1_link sweeps Z=-1.0->1.0 as before
  // Note: convex mesh cc_time differs from primitive (0.3848 vs 0.3333)
  // due to tessellation affecting the support point distances.
  /////////////////////////////////////////////////////////////

  // Set the start location
  location_start["sphere_link"] = Eigen::Isometry3d::Identity();
  location_start["sphere_link"].translation()(0) = -0.2;
  location_start["sphere_link"].translation()(1) = -0.5;

  location_start["sphere1_link"] = Eigen::Isometry3d::Identity();
  location_start["sphere1_link"].translation()(0) = 0.2;
  location_start["sphere1_link"].translation()(2) = -1.0;

  // Set the end location
  location_end["sphere_link"] = Eigen::Isometry3d::Identity();
  location_end["sphere_link"].translation()(0) = -0.2;
  location_end["sphere_link"].translation()(1) = 1.0;

  location_end["sphere1_link"] = Eigen::Isometry3d::Identity();
  location_end["sphere1_link"].translation()(0) = 0.2;
  location_end["sphere1_link"].translation()(2) = 1.0;

  checker.setCollisionObjectsTransform(location_start, location_end);

  // Perform collision check
  result.clear();
  result_vector.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));

  result.flattenCopyResults(result_vector);

  ASSERT_FALSE(result_vector.empty()) << "Scenario 2 convex (asymmetric cc_time 0.385/0.5): No contacts found. "
                                      << "Convex sphere_link sweeps Y=-0.5->1.0, sphere1_link sweeps Z=-1.0->1.0.";

  const auto& cr2 = result_vector[0];
  SCOPED_TRACE("Scenario 2 convex (cc_time 0.385/0.5): " + formatContactResult(cr2));

  EXPECT_NEAR(cr2.distance, -0.0754, 0.001) << "Penetration for convex mesh spheres";

  idx = { 0, 1, 1 };
  if (cr2.link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  const std::string sphere_slot2 = (idx[0] == 0) ? "slot 0" : "slot 1";
  const std::string sphere1_slot2 = (idx[1] == 0) ? "slot 0" : "slot 1";

  EXPECT_NEAR(cr2.cc_time[static_cast<size_t>(idx[0])], 0.3848, 0.001)
      << "sphere_link (" << sphere_slot2 << ") cc_time ~0.385 (convex mesh; differs from "
      << "primitive 0.333 due to tessellation support point distances)";
  EXPECT_NEAR(cr2.cc_time[static_cast<size_t>(idx[1])], 0.5, 0.001)
      << "sphere1_link (" << sphere1_slot2 << ") cc_time should be 0.5";

  EXPECT_EQ(cr2.cc_type[static_cast<size_t>(idx[0])], ContinuousCollisionType::CCType_Between)
      << "sphere_link (" << sphere_slot2 << ") cc_type should be CCType_Between (3), "
      << "got " << static_cast<int>(cr2.cc_type[static_cast<size_t>(idx[0])]);
  EXPECT_EQ(cr2.cc_type[static_cast<size_t>(idx[1])], ContinuousCollisionType::CCType_Between)
      << "sphere1_link (" << sphere1_slot2 << ") cc_type should be CCType_Between (3), "
      << "got " << static_cast<int>(cr2.cc_type[static_cast<size_t>(idx[1])]);

  // World-frame nearest points
  EXPECT_NEAR(cr2.nearest_points[static_cast<size_t>(idx[0])][0], 0.0377, 0.001) << "sphere_link nearest_point.x "
                                                                                    "(convex mesh)";
  EXPECT_NEAR(cr2.nearest_points[static_cast<size_t>(idx[0])][1], 0.0772, 0.001) << "sphere_link nearest_point.y: "
                                                                                    "offset from Y=0 due to asymmetric "
                                                                                    "sweep timing";
  EXPECT_NEAR(cr2.nearest_points[static_cast<size_t>(idx[0])][2], 0.25, 0.001) << "sphere_link nearest_point.z: "
                                                                                  "sphere_pose z=0.25 offset";

  EXPECT_NEAR(cr2.nearest_points[static_cast<size_t>(idx[1])][0], -0.0377, 0.001) << "sphere1_link nearest_point.x "
                                                                                     "(convex mesh)";
  EXPECT_NEAR(cr2.nearest_points[static_cast<size_t>(idx[1])][1], 0.0772, 0.001) << "sphere1_link nearest_point.y";
  EXPECT_NEAR(cr2.nearest_points[static_cast<size_t>(idx[1])][2], 0.25, 0.001) << "sphere1_link nearest_point.z: "
                                                                                  "sphere_pose z=0.25 offset";

  // Local-frame nearest points
  EXPECT_NEAR(cr2.nearest_points_local[static_cast<size_t>(idx[0])][0], 0.2377, 0.001) << "sphere_link "
                                                                                          "nearest_point_local.x "
                                                                                          "(convex mesh)";
  EXPECT_NEAR(cr2.nearest_points_local[static_cast<size_t>(idx[0])][1], 0.0, 0.001) << "sphere_link "
                                                                                       "nearest_point_local.y";
  EXPECT_NEAR(cr2.nearest_points_local[static_cast<size_t>(idx[0])][2], 0.25, 0.001) << "sphere_link "
                                                                                        "nearest_point_local.z";
  EXPECT_NEAR(cr2.nearest_points_local[static_cast<size_t>(idx[1])][0], -0.2377, 0.001) << "sphere1_link "
                                                                                           "nearest_point_local.x "
                                                                                           "(convex mesh)";
  EXPECT_NEAR(cr2.nearest_points_local[static_cast<size_t>(idx[1])][1], 0.0, 0.001) << "sphere1_link "
                                                                                       "nearest_point_local.y";
  EXPECT_NEAR(cr2.nearest_points_local[static_cast<size_t>(idx[1])][2], 0.25, 0.001) << "sphere1_link "
                                                                                        "nearest_point_local.z";

  // Verify transforms
  EXPECT_TRUE(cr2.transform[static_cast<size_t>(idx[0])].isApprox(location_start["sphere_link"], 0.0001)) << "sphere_"
                                                                                                             "link "
                                                                                                             "transform"
                                                                                                             " should "
                                                                                                             "match "
                                                                                                             "start "
                                                                                                             "pose "
                                                                                                             "(-0.2, "
                                                                                                             "-0.5, 0)";
  EXPECT_TRUE(cr2.transform[static_cast<size_t>(idx[1])].isApprox(location_start["sphere1_link"], 0.0001)) << "sphere1_"
                                                                                                              "link "
                                                                                                              "transfor"
                                                                                                              "m "
                                                                                                              "should "
                                                                                                              "match "
                                                                                                              "start "
                                                                                                              "pose "
                                                                                                              "(0.2, "
                                                                                                              "0, -1)";
  EXPECT_TRUE(cr2.cc_transform[static_cast<size_t>(idx[0])].isApprox(location_end["sphere_link"], 0.0001)) << "sphere_"
                                                                                                              "link "
                                                                                                              "cc_"
                                                                                                              "transfor"
                                                                                                              "m "
                                                                                                              "should "
                                                                                                              "match "
                                                                                                              "end "
                                                                                                              "pose "
                                                                                                              "(-0.2, "
                                                                                                              "1, 0)";
  EXPECT_TRUE(cr2.cc_transform[static_cast<size_t>(idx[1])].isApprox(location_end["sphere1_link"], 0.0001)) << "sphere1"
                                                                                                               "_link "
                                                                                                               "cc_"
                                                                                                               "transfo"
                                                                                                               "rm "
                                                                                                               "should "
                                                                                                               "match "
                                                                                                               "end "
                                                                                                               "pose "
                                                                                                               "(0.2, "
                                                                                                               "0, 1)";

  // Contact normal
  EXPECT_NEAR(cr2.normal[0], idx[2] * 1.0, 0.001) << "Contact normal x: should point along X (sphere separation "
                                                     "direction)";
  EXPECT_NEAR(cr2.normal[1], idx[2] * 0.0, 0.001) << "Contact normal y: should be ~0";
  EXPECT_NEAR(cr2.normal[2], idx[2] * 0.0, 0.001) << "Contact normal z: should be ~0";
}
}  // namespace detail

inline void runTest(ContinuousContactManager& checker, bool use_convex_mesh)
{
  // Add collision objects
  detail::addCollisionObjects(checker, use_convex_mesh);

  // Call it again to test adding same object
  detail::addCollisionObjects(checker, use_convex_mesh);

  if (use_convex_mesh)
    detail::runTestConvex(checker);
  else
    detail::runTestPrimitive(checker);
}

}  // namespace tesseract::collision::test_suite
#endif  // TESSERACT_COLLISION_COLLISION_SPHERE_SPHERE_CAST_UNIT_HPP
