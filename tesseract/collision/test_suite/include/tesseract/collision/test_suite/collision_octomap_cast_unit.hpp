#ifndef TESSERACT_COLLISION_COLLISION_OCTOMAP_CAST_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_OCTOMAP_CAST_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <octomap/octomap.h>
#include <gtest/gtest.h>
#include <sstream>
#include <iomanip>
#include <cmath>
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
inline std::string formatOctomapContactResult(const ContactResult& cr)
{
  std::ostringstream os;
  os << std::setprecision(6) << std::fixed;
  os << "Contact result:"
     << "\n  link_names: [" << cr.link_names[0] << ", " << cr.link_names[1] << "]"
     << "\n  distance: " << cr.distance
     << "\n  normal: (" << cr.normal[0] << ", " << cr.normal[1] << ", " << cr.normal[2] << ")"
     << "\n  nearest_points[0]: (" << cr.nearest_points[0][0] << ", " << cr.nearest_points[0][1] << ", "
     << cr.nearest_points[0][2] << ")"
     << "\n  nearest_points[1]: (" << cr.nearest_points[1][0] << ", " << cr.nearest_points[1][1] << ", "
     << cr.nearest_points[1][2] << ")"
     << "\n  nearest_points_local[0]: (" << cr.nearest_points_local[0][0] << ", "
     << cr.nearest_points_local[0][1] << ", " << cr.nearest_points_local[0][2] << ")"
     << "\n  nearest_points_local[1]: (" << cr.nearest_points_local[1][0] << ", "
     << cr.nearest_points_local[1][1] << ", " << cr.nearest_points_local[1][2] << ")"
     << "\n  cc_time: [" << cr.cc_time[0] << ", " << cr.cc_time[1] << "]"
     << "\n  cc_type: [" << static_cast<int>(cr.cc_type[0]) << ", " << static_cast<int>(cr.cc_type[1]) << "]"
     << "\n  transform[0].t: (" << cr.transform[0].translation()[0] << ", " << cr.transform[0].translation()[1] << ", "
     << cr.transform[0].translation()[2] << ")"
     << "\n  transform[1].t: (" << cr.transform[1].translation()[0] << ", " << cr.transform[1].translation()[1] << ", "
     << cr.transform[1].translation()[2] << ")"
     << "\n  cc_transform[0].t: (" << cr.cc_transform[0].translation()[0] << ", "
     << cr.cc_transform[0].translation()[1] << ", " << cr.cc_transform[0].translation()[2] << ")"
     << "\n  cc_transform[1].t: (" << cr.cc_transform[1].translation()[0] << ", "
     << cr.cc_transform[1].translation()[1] << ", " << cr.cc_transform[1].translation()[2] << ")";
  return os.str();
}

/**
 * @brief Check continuous collision fields on an octree-vs-kinematic contact result.
 *
 * @param cr           The contact result to validate.
 * @param kin_link     Name of the kinematic (active) link.
 * @param start_pos    Pose1 (start of sweep) for the kinematic link.
 * @param end_pos      Pose2 (end of sweep) for the kinematic link.
 * @param sweep_dir    Unit vector along the sweep direction in world frame.
 */
/**
 * @brief Check continuous collision fields on an octree-vs-kinematic contact.
 *
 * @param sweep_dir  Unit vector along the sweep direction, used to validate
 *                   the normal direction when cc_type is CCType_Between.
 *
 * Which CCType the kinematic side receives depends on which octree voxel the
 * broadphase selects (non-deterministic across test runs):
 *
 *  - CCType_Time1: the contact normal has a large component along the sweep
 *    direction, so the t=1 world-frame support dominates → cc_time = 1.0.
 *
 *  - CCType_Between: the contact normal is roughly perpendicular to the sweep
 *    (normal · sweep ≈ 0), world supports are equal → cc_time ∈ (0, 1].
 *    In this case the perpendicularity is a consequence of the type, so we
 *    can also assert it.
 *
 * CCType_None is impossible (kinematic object), and CCType_Time0 is
 * impossible because the kinematic shape starts well outside the octree.
 */
inline void checkOctomapCastResult(const ContactResult& cr,
                                   const std::string& kin_link,
                                   const Eigen::Isometry3d& start_pos,
                                   const Eigen::Isometry3d& end_pos,
                                   const Eigen::Vector3d& sweep_dir)
{
  // Determine which slot holds the kinematic shape and which holds the static octree.
  EXPECT_TRUE(cr.link_names[0] == kin_link || cr.link_names[1] == kin_link)
      << "Expected kinematic link '" << kin_link << "' in contact result, "
      << "got [" << cr.link_names[0] << ", " << cr.link_names[1] << "]";
  const std::size_t ki = (cr.link_names[0] == kin_link) ? 0 : 1;
  const std::size_t si = 1 - ki;

  // -----------------------------------------------------------------------
  // Static (octree) side
  // The octree voxels are wrapped in CastHullShape with identity cast, but
  // they are in StaticFilter.  populateContinuousCollisionFields skips them,
  // so their CCD fields keep the default values: CCType_None / cc_time = -1.
  // -----------------------------------------------------------------------
  EXPECT_EQ(cr.cc_type[si], ContinuousCollisionType::CCType_None)
      << "Octree (static) cc_type should be CCType_None; "
      << "got " << static_cast<int>(cr.cc_type[si]);
  EXPECT_NEAR(cr.cc_time[si], -1.0, 1e-3)
      << "Octree (static) cc_time should be -1 (not set for static objects)";

  // -----------------------------------------------------------------------
  // Kinematic side: cc_type must be Time1 or Between.
  //
  // Which one depends on which voxel the broadphase selects (ordering is not
  // guaranteed across test runs).  Both values are correct; what matters is
  // that the static value (None) and the impossible value (Time0) are ruled
  // out.
  // -----------------------------------------------------------------------
  const bool kin_type_valid = (cr.cc_type[ki] == ContinuousCollisionType::CCType_Time1 ||
                               cr.cc_type[ki] == ContinuousCollisionType::CCType_Between);
  EXPECT_TRUE(kin_type_valid)
      << "Kinematic cc_type must be CCType_Time1 (2) or CCType_Between (3); "
      << "got " << static_cast<int>(cr.cc_type[ki])
      << ". CCType_None means static (wrong). "
         "CCType_Time0 is geometrically impossible (shape starts outside octree).";

  // cc_time consistency with cc_type:
  //   CCType_Time1  → cc_time = 1.0 (set explicitly by populateContinuousCollisionFields)
  //   CCType_Between → cc_time = projection of contact midpoint onto sweep,
  //                    always > 0 because the shape starts outside the octree.
  if (cr.cc_type[ki] == ContinuousCollisionType::CCType_Time1)
  {
    EXPECT_NEAR(cr.cc_time[ki], 1.0, 1e-3)
        << "For CCType_Time1, cc_time must be 1.0";
  }
  else
  {
    EXPECT_GT(cr.cc_time[ki], 0.0)
        << "For CCType_Between, cc_time must be > 0 (shape starts outside octree)";
    EXPECT_LE(cr.cc_time[ki], 1.0)
        << "For CCType_Between, cc_time must be <= 1.0";

    // CCType_Between is set precisely when the contact normal is perpendicular
    // to the sweep (normal · sweep ≈ 0), making the world supports equal at
    // t=0 and t=1.  This is an invariant we can verify.
    const double along_sweep = std::abs(cr.normal.dot(sweep_dir));
    EXPECT_LT(along_sweep, 0.5)
        << "For CCType_Between, contact normal should be mostly perpendicular "
        << "to sweep direction (along_sweep = " << along_sweep << "); "
        << "a large component means the world supports should differ and "
        << "CCType_Time1 should have been set instead";
  }

  // transform[ki] = pose1 (start of sweep).
  EXPECT_TRUE(cr.transform[ki].isApprox(start_pos, 1e-5))
      << "Kinematic transform should match start pose (pose1)";

  // cc_transform[ki] = pose2 (end of sweep).
  EXPECT_TRUE(cr.cc_transform[ki].isApprox(end_pos, 1e-5))
      << "Kinematic cc_transform should match end pose (pose2)";

  // Normal must be a unit vector.
  EXPECT_NEAR(cr.normal.norm(), 1.0, 1e-3)
      << "Contact normal must be a unit vector";

  // -----------------------------------------------------------------------
  // distance: the kinematic link ends inside the octree, so the swept hull
  // genuinely penetrates at least one voxel → distance must be negative.
  // -----------------------------------------------------------------------
  EXPECT_LT(cr.distance, 0.0)
      << "Expected penetration (distance < 0); got " << cr.distance
      << ". The kinematic shape ends inside the octree, so the swept hull "
         "must overlap a voxel and distance must be strictly negative.";

  // -----------------------------------------------------------------------
  // nearest_points_local[ki]: the local-frame contact point for the kinematic
  // link must be non-trivial (non-zero).  A zero vector means the field was
  // never populated, which would cause Trajopt to compute a zero-offset
  // Jacobian and produce incorrect gradients.
  // -----------------------------------------------------------------------
  EXPECT_GT(cr.nearest_points_local[ki].norm(), 1e-6)
      << "nearest_points_local for kinematic link '" << kin_link << "' should be non-zero "
         "(non-trivial contact point in the link frame). A zero value means the "
         "field was not set by populateContinuousCollisionFields.";

  // -----------------------------------------------------------------------
  // CCType_Time1 normal-direction invariant:
  //
  // CCType_Time1 is selected when link_sup1 > link_sup0 + tolerance.
  // For a pure-translational sweep:
  //   link_sup1 - link_sup0  =  normal_world · sweep_vector
  // where normal_world is the outward normal FROM the kinematic shape
  // (i.e. normal_world = contact.normal when ki==0, -contact.normal when ki==1).
  //
  // Therefore, if cc_type is CCType_Time1, the outward normal from the
  // kinematic shape must have a strictly positive component along sweep_dir.
  // -----------------------------------------------------------------------
  if (cr.cc_type[ki] == ContinuousCollisionType::CCType_Time1)
  {
    const double along_sweep = (ki == 0 ? 1.0 : -1.0) * cr.normal.dot(sweep_dir);
    EXPECT_GT(along_sweep, 0.001)
        << "For CCType_Time1, the outward normal from the kinematic shape must "
        << "have a positive component along sweep_dir "
        << "(along_sweep = " << along_sweep << "). "
        << "This is the invariant that selects CCType_Time1 over CCType_Between.";
  }
}

/**
 * @brief Add a static octree (BOX) and an active cylinder to the cast checker.
 *
 * The octree is a 2m box octree at the origin. The cylinder is a small
 * primitive (r=0.1, h=0.5) that will be swept through the octree.
 */
inline void addOctomapCylinderCollisionObjects(ContinuousContactManager& checker)
{
  /////////////////////////////////////////////////////////////////
  // Add Octomap (static)
  /////////////////////////////////////////////////////////////////
  tesseract::common::GeneralResourceLocator locator;
  std::string path = locator.locateResource("package://tesseract/support/meshes/box_2m.bt")->getFilePath();
  auto ot = std::make_shared<octomap::OcTree>(path);
  CollisionShapePtr dense_octomap =
      std::make_shared<tesseract::geometry::Octree>(ot, tesseract::geometry::OctreeSubType::BOX);
  Eigen::Isometry3d octomap_pose = Eigen::Isometry3d::Identity();

  CollisionShapesConst obj1_shapes;
  tesseract::common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(dense_octomap);
  obj1_poses.push_back(octomap_pose);

  checker.addCollisionObject("octomap_link", 0, obj1_shapes, obj1_poses);

  /////////////////////////////////////////////////////////////////
  // Add cylinder (active, will be swept)
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr cylinder = std::make_shared<tesseract::geometry::Cylinder>(0.1, 0.5);
  Eigen::Isometry3d cylinder_pose = Eigen::Isometry3d::Identity();

  CollisionShapesConst obj2_shapes;
  tesseract::common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(cylinder);
  obj2_poses.push_back(cylinder_pose);

  checker.addCollisionObject("cylinder_link", 0, obj2_shapes, obj2_poses);

  EXPECT_EQ(checker.getCollisionObjects().size(), 2);
}

/**
 * @brief Add a static octree (BOX) and an active sphere to the cast checker.
 */
inline void addOctomapSphereCollisionObjects(ContinuousContactManager& checker)
{
  /////////////////////////////////////////////////////////////////
  // Add Octomap (static)
  /////////////////////////////////////////////////////////////////
  tesseract::common::GeneralResourceLocator locator;
  std::string path = locator.locateResource("package://tesseract/support/meshes/box_2m.bt")->getFilePath();
  auto ot = std::make_shared<octomap::OcTree>(path);
  CollisionShapePtr dense_octomap =
      std::make_shared<tesseract::geometry::Octree>(ot, tesseract::geometry::OctreeSubType::BOX);
  Eigen::Isometry3d octomap_pose = Eigen::Isometry3d::Identity();

  CollisionShapesConst obj1_shapes;
  tesseract::common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(dense_octomap);
  obj1_poses.push_back(octomap_pose);

  checker.addCollisionObject("octomap_link", 0, obj1_shapes, obj1_poses);

  /////////////////////////////////////////////////////////////////
  // Add sphere (active, will be swept)
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr sphere = std::make_shared<tesseract::geometry::Sphere>(0.25);
  Eigen::Isometry3d sphere_pose = Eigen::Isometry3d::Identity();

  CollisionShapesConst obj2_shapes;
  tesseract::common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(sphere);
  obj2_poses.push_back(sphere_pose);

  checker.addCollisionObject("sphere_link", 0, obj2_shapes, obj2_poses);

  EXPECT_EQ(checker.getCollisionObjects().size(), 2);
}

inline void runOctomapCylinderCastTest(ContinuousContactManager& checker, ContactTestType test_type)
{
  // Only the cylinder is active; the octree is static.
  std::vector<std::string> active_links{ "cylinder_link" };
  checker.setActiveCollisionObjects(active_links);

  checker.setDefaultCollisionMargin(0.1);
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);

  // Set the static octree transform
  tesseract::common::TransformMap location;
  location["octomap_link"] = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);

  // Sweep the cylinder from outside (-2, 0, 0) through the octree to (0, 0, 0).
  // The 2m box octree extends from roughly -1 to 1 on each axis, so the cylinder
  // starts outside and ends inside.
  Eigen::Isometry3d start_pos = Eigen::Isometry3d::Identity();
  start_pos.translation() = Eigen::Vector3d(-2.0, 0, 0);
  Eigen::Isometry3d end_pos = Eigen::Isometry3d::Identity();
  end_pos.translation() = Eigen::Vector3d(0.0, 0, 0);
  checker.setCollisionObjectsTransform("cylinder_link", start_pos, end_pos);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(test_type));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty()) << "No contacts found for cylinder sweeping from (-2,0,0) to (0,0,0) "
                                      << "through a 2m box octree at origin. The swept path enters the octree, "
                                      << "so at least one contact is expected.";

  // Verify we got a collision between the expected link pair
  bool found_pair = false;
  for (const auto& cr : result_vector)
  {
    if ((cr.link_names[0] == "octomap_link" && cr.link_names[1] == "cylinder_link") ||
        (cr.link_names[0] == "cylinder_link" && cr.link_names[1] == "octomap_link"))
    {
      found_pair = true;
      SCOPED_TRACE(formatOctomapContactResult(cr));
      EXPECT_LT(cr.distance, 0.11) << "Expected collision/near-contact between octree and cylinder";
      checkOctomapCastResult(cr, "cylinder_link", start_pos, end_pos, Eigen::Vector3d(1, 0, 0));
      break;
    }
  }
  EXPECT_TRUE(found_pair) << "Expected contact between octomap_link and cylinder_link";
}

inline void runOctomapSphereCastTest(ContinuousContactManager& checker, ContactTestType test_type)
{
  // Only the sphere is active; the octree is static.
  std::vector<std::string> active_links{ "sphere_link" };
  checker.setActiveCollisionObjects(active_links);

  checker.setDefaultCollisionMargin(0.1);
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);

  // Set the static octree transform
  tesseract::common::TransformMap location;
  location["octomap_link"] = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);

  // Sweep the sphere from outside (0, 0, 2) down into the octree at (0, 0, 0).
  Eigen::Isometry3d start_pos = Eigen::Isometry3d::Identity();
  start_pos.translation() = Eigen::Vector3d(0, 0, 2.0);
  Eigen::Isometry3d end_pos = Eigen::Isometry3d::Identity();
  end_pos.translation() = Eigen::Vector3d(0, 0, 0.0);
  checker.setCollisionObjectsTransform("sphere_link", start_pos, end_pos);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(test_type));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty()) << "No contacts found for sphere sweeping from (0,0,2) to (0,0,0) "
                                      << "through a 2m box octree at origin. The swept path enters the octree, "
                                      << "so at least one contact is expected.";

  // Verify correct link pair
  bool found_pair = false;
  for (const auto& cr : result_vector)
  {
    if ((cr.link_names[0] == "octomap_link" && cr.link_names[1] == "sphere_link") ||
        (cr.link_names[0] == "sphere_link" && cr.link_names[1] == "octomap_link"))
    {
      found_pair = true;
      SCOPED_TRACE(formatOctomapContactResult(cr));
      EXPECT_LT(cr.distance, 0.11) << "Expected collision/near-contact between octree and sphere";
      checkOctomapCastResult(cr, "sphere_link", start_pos, end_pos, Eigen::Vector3d(0, 0, -1));
      break;
    }
  }
  EXPECT_TRUE(found_pair) << "Expected contact between octomap_link and sphere_link";
}
/**
 * @brief Add a static octree (BOX) and an active convex hull to the cast checker.
 *
 * The convex hull is built from a sphere PLY mesh, giving a polyhedral
 * approximation. This tests the octree cast path with ConvexMesh geometry.
 */
inline void addOctomapConvexHullCollisionObjects(ContinuousContactManager& checker)
{
  /////////////////////////////////////////////////////////////////
  // Add Octomap (static)
  /////////////////////////////////////////////////////////////////
  tesseract::common::GeneralResourceLocator locator;
  std::string path = locator.locateResource("package://tesseract/support/meshes/box_2m.bt")->getFilePath();
  auto ot = std::make_shared<octomap::OcTree>(path);
  CollisionShapePtr dense_octomap =
      std::make_shared<tesseract::geometry::Octree>(ot, tesseract::geometry::OctreeSubType::BOX);
  Eigen::Isometry3d octomap_pose = Eigen::Isometry3d::Identity();

  CollisionShapesConst obj1_shapes;
  tesseract::common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(dense_octomap);
  obj1_poses.push_back(octomap_pose);

  checker.addCollisionObject("octomap_link", 0, obj1_shapes, obj1_poses);

  /////////////////////////////////////////////////////////////////
  // Add convex hull (active, will be swept)
  /////////////////////////////////////////////////////////////////
  auto mesh_vertices = std::make_shared<tesseract::common::VectorVector3d>();
  auto mesh_faces = std::make_shared<Eigen::VectorXi>();
  EXPECT_GT(tesseract::common::loadSimplePlyFile(
                locator.locateResource("package://tesseract/support/meshes/sphere_p25m.ply")->getFilePath(),
                *mesh_vertices,
                *mesh_faces,
                true),
            0);

  auto mesh = std::make_shared<tesseract::geometry::Mesh>(mesh_vertices, mesh_faces);
  CollisionShapePtr convex = makeConvexMesh(*mesh);
  Eigen::Isometry3d convex_pose = Eigen::Isometry3d::Identity();

  CollisionShapesConst obj2_shapes;
  tesseract::common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(convex);
  obj2_poses.push_back(convex_pose);

  checker.addCollisionObject("convex_link", 0, obj2_shapes, obj2_poses);

  EXPECT_EQ(checker.getCollisionObjects().size(), 2);
}

inline void runOctomapConvexHullCastTest(ContinuousContactManager& checker, ContactTestType test_type)
{
  // Only the convex hull is active; the octree is static.
  std::vector<std::string> active_links{ "convex_link" };
  checker.setActiveCollisionObjects(active_links);

  checker.setDefaultCollisionMargin(0.1);
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);

  // Set the static octree transform
  tesseract::common::TransformMap location;
  location["octomap_link"] = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);

  // Sweep the convex hull from outside (-2, 0, 0) through the octree to (0, 0, 0).
  Eigen::Isometry3d start_pos = Eigen::Isometry3d::Identity();
  start_pos.translation() = Eigen::Vector3d(-2.0, 0, 0);
  Eigen::Isometry3d end_pos = Eigen::Isometry3d::Identity();
  end_pos.translation() = Eigen::Vector3d(0.0, 0, 0);
  checker.setCollisionObjectsTransform("convex_link", start_pos, end_pos);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(test_type));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty()) << "No contacts found for convex hull sweeping from (-2,0,0) to (0,0,0) "
                                      << "through a 2m box octree at origin. The swept path enters the octree, "
                                      << "so at least one contact is expected.";

  // Verify correct link pair
  bool found_pair = false;
  for (const auto& cr : result_vector)
  {
    if ((cr.link_names[0] == "octomap_link" && cr.link_names[1] == "convex_link") ||
        (cr.link_names[0] == "convex_link" && cr.link_names[1] == "octomap_link"))
    {
      found_pair = true;
      SCOPED_TRACE(formatOctomapContactResult(cr));
      EXPECT_LT(cr.distance, 0.11) << "Expected collision/near-contact between octree and convex hull";
      checkOctomapCastResult(cr, "convex_link", start_pos, end_pos, Eigen::Vector3d(1, 0, 0));
      break;
    }
  }
  EXPECT_TRUE(found_pair) << "Expected contact between octomap_link and convex_link";
}
}  // namespace detail

/**
 * @brief Run continuous collision test: static octree (BOX) vs active cylinder.
 *
 * Tests the scenario where an octree is static and a primitive shape sweeps
 * through it. This exercises the cast manager's handling of expanded octree
 * voxels in the static broadphase vs CastHullShape in the dynamic broadphase.
 */
inline void runTestOctomapCylinder(ContinuousContactManager& checker)
{
  detail::addOctomapCylinderCollisionObjects(checker);

  // Call again to test re-adding same objects
  detail::addOctomapCylinderCollisionObjects(checker);

  detail::runOctomapCylinderCastTest(checker, ContactTestType::FIRST);
  detail::runOctomapCylinderCastTest(checker, ContactTestType::CLOSEST);
  detail::runOctomapCylinderCastTest(checker, ContactTestType::ALL);

  // Also test after cloning
  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runOctomapCylinderCastTest(*cloned, ContactTestType::FIRST);
}

/**
 * @brief Run continuous collision test: static octree (BOX) vs active sphere.
 */
inline void runTestOctomapSphere(ContinuousContactManager& checker)
{
  detail::addOctomapSphereCollisionObjects(checker);

  // Call again to test re-adding same objects
  detail::addOctomapSphereCollisionObjects(checker);

  detail::runOctomapSphereCastTest(checker, ContactTestType::FIRST);
  detail::runOctomapSphereCastTest(checker, ContactTestType::CLOSEST);
  detail::runOctomapSphereCastTest(checker, ContactTestType::ALL);

  // Also test after cloning
  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runOctomapSphereCastTest(*cloned, ContactTestType::FIRST);
}

/**
 * @brief Run continuous collision test: static octree (BOX) vs active convex hull.
 *
 * Tests the scenario with a polyhedral convex mesh (built from a sphere PLY)
 * sweeping through a static octree.
 */
inline void runTestOctomapConvexHull(ContinuousContactManager& checker)
{
  detail::addOctomapConvexHullCollisionObjects(checker);

  // Call again to test re-adding same objects
  detail::addOctomapConvexHullCollisionObjects(checker);

  detail::runOctomapConvexHullCastTest(checker, ContactTestType::FIRST);
  detail::runOctomapConvexHullCastTest(checker, ContactTestType::CLOSEST);
  detail::runOctomapConvexHullCastTest(checker, ContactTestType::ALL);

  // Also test after cloning
  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runOctomapConvexHullCastTest(*cloned, ContactTestType::FIRST);
}

}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_OCTOMAP_CAST_UNIT_HPP
