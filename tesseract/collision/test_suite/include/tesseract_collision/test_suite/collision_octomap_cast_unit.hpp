#ifndef TESSERACT_COLLISION_COLLISION_OCTOMAP_CAST_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_OCTOMAP_CAST_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <octomap/octomap.h>
#include <gtest/gtest.h>
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
      // The collision should have negative or near-zero distance (penetration or contact)
      EXPECT_LT(cr.distance, 0.11) << "Expected collision/near-contact between octree and cylinder";
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
      EXPECT_LT(cr.distance, 0.11) << "Expected collision/near-contact between octree and sphere";
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
      EXPECT_LT(cr.distance, 0.11) << "Expected collision/near-contact between octree and convex hull";
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
