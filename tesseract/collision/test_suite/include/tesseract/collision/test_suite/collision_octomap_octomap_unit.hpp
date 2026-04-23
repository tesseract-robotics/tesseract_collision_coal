#ifndef TESSERACT_COLLISION_COLLISION_OCTOMAP_OCTOMAP_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_OCTOMAP_OCTOMAP_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <octomap/octomap.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>

namespace tesseract::collision::test_suite
{
namespace detail
{
template <class T>
inline void addCollisionObjects(T& checker,
                                tesseract::geometry::OctreeSubType subtype1,
                                tesseract::geometry::OctreeSubType subtype2)
{
  /////////////////////////////////////////////////////////////////
  // Add Octomap
  /////////////////////////////////////////////////////////////////
  tesseract::common::GeneralResourceLocator locator;
  std::string path = locator.locateResource("package://tesseract/support/meshes/box_2m.bt")->getFilePath();
  auto ot = std::make_shared<octomap::OcTree>(path);
  CollisionShapePtr dense_octomap = std::make_shared<tesseract::geometry::Octree>(ot, subtype1);
  Eigen::Isometry3d octomap_pose;
  octomap_pose.setIdentity();
  octomap_pose.translation() = Eigen::Vector3d(1.1, 0, 0);

  CollisionShapesConst obj1_shapes;
  tesseract::common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(dense_octomap);
  obj1_poses.push_back(octomap_pose);

  checker.addCollisionObject("octomap1_link", 0, obj1_shapes, obj1_poses);

  /////////////////////////////////////////////////////////////////
  // Add second octomap
  /////////////////////////////////////////////////////////////////
  auto ot_b = std::make_shared<octomap::OcTree>(path);
  CollisionShapePtr dense_octomap_b = std::make_shared<tesseract::geometry::Octree>(ot_b, subtype2);
  Eigen::Isometry3d octomap_pose_b;
  octomap_pose_b.setIdentity();
  octomap_pose_b.translation() = Eigen::Vector3d(-1.1, 0, 0);

  CollisionShapesConst obj2_shapes;
  tesseract::common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(dense_octomap_b);
  obj2_poses.push_back(octomap_pose_b);

  checker.addCollisionObject("octomap2_link", 0, obj2_shapes, obj2_poses);

  EXPECT_TRUE(checker.getCollisionObjects().size() == 2);
  const auto& co = checker.getCollisionObjects();
  for (std::size_t i = 0; i < co.size(); ++i)
  {
    EXPECT_TRUE(checker.getCollisionObjectGeometries(co[i].name()).size() == 1);
    EXPECT_TRUE(checker.getCollisionObjectGeometriesTransforms(co[i].name()).size() == 1);
    const auto& cgt = checker.getCollisionObjectGeometriesTransforms(co[i].name());
    if (i == 0)
    {
      EXPECT_TRUE(cgt[0].isApprox(octomap_pose, 1e-5));
    }
    else
    {
      EXPECT_TRUE(cgt[0].isApprox(octomap_pose_b, 1e-5));
    }
  }
}

inline void runTestOctomap(DiscreteContactManager& checker, ContactTestType test_type, double expected_distance)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  std::vector<std::string> active_links{ "octomap1_link", "octomap2_link" };
  checker.setActiveCollisionObjects(active_links);
  std::vector<std::string> check_active_links = checker.getActiveCollisionObjectNames();
  EXPECT_TRUE(tesseract::common::isIdentical<std::string>(active_links, check_active_links, false));

  EXPECT_TRUE(checker.getContactAllowedValidator() == nullptr);

  checker.setCollisionMarginData(CollisionMarginData(0.25));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.25, 1e-5);

  // Set the collision object transforms
  tesseract::common::LinkIdTransformMap location;
  location[tesseract::common::LinkId("octomap1_link")] = Eigen::Isometry3d::Identity();
  location[tesseract::common::LinkId("octomap2_link")] = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(test_type));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  EXPECT_TRUE(!result_vector.empty());
  for (const auto& cr : result_vector)
  {
    EXPECT_NEAR(cr.distance, expected_distance, 0.001);
  }
}

inline void runTestOctomap(ContinuousContactManager& checker, ContactTestType test_type, double expected_distance)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  std::vector<std::string> active_links{ "octomap1_link" };
  checker.setActiveCollisionObjects(active_links);
  std::vector<std::string> check_active_links = checker.getActiveCollisionObjectNames();
  EXPECT_TRUE(tesseract::common::isIdentical<std::string>(active_links, check_active_links, false));

  EXPECT_TRUE(checker.getContactAllowedValidator() == nullptr);

  checker.setCollisionMarginData(CollisionMarginData(0.25));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.25, 1e-5);

  // Set the collision object transforms
  tesseract::common::LinkIdTransformMap location;
  location[tesseract::common::LinkId("octomap2_link")] = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);

  // Set the collision object transforms
  Eigen::Isometry3d start_pos, end_pos;
  start_pos = Eigen::Isometry3d::Identity();
  end_pos = Eigen::Isometry3d::Identity();
  start_pos.translation() = Eigen::Vector3d(0, -2.0, 0);
  end_pos.translation() = Eigen::Vector3d(0, 2.0, 0);
  checker.setCollisionObjectsTransform("octomap1_link", start_pos, end_pos);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(test_type));

  ContactResultVector result_vector;
  result.flattenCopyResults(result_vector);

  EXPECT_TRUE(!result_vector.empty());
  for (const auto& cr : result_vector)
  {
    EXPECT_NEAR(cr.distance, expected_distance, 0.001);
  }
}
}  // namespace detail

inline void runTest(ContinuousContactManager& checker,
                    double expected_distance,
                    tesseract::geometry::OctreeSubType subtype1,
                    tesseract::geometry::OctreeSubType subtype2)
{
  detail::addCollisionObjects<ContinuousContactManager>(checker, subtype1, subtype2);

  // Call it again to test adding same object
  detail::addCollisionObjects<ContinuousContactManager>(checker, subtype1, subtype2);

  detail::runTestOctomap(checker, ContactTestType::FIRST, expected_distance);
  detail::runTestOctomap(checker, ContactTestType::CLOSEST, expected_distance);
  detail::runTestOctomap(checker, ContactTestType::ALL, expected_distance);

  ContinuousContactManager::Ptr cloned_checker = checker.clone();
  detail::runTestOctomap(*cloned_checker, ContactTestType::FIRST, expected_distance);
}

inline void runTest(DiscreteContactManager& checker,
                    double expected_distance,
                    tesseract::geometry::OctreeSubType subtype1,
                    tesseract::geometry::OctreeSubType subtype2)
{
  detail::addCollisionObjects<DiscreteContactManager>(checker, subtype1, subtype2);

  // Call it again to test adding same object
  detail::addCollisionObjects<DiscreteContactManager>(checker, subtype1, subtype2);

  detail::runTestOctomap(checker, ContactTestType::FIRST, expected_distance);
  detail::runTestOctomap(checker, ContactTestType::CLOSEST, expected_distance);
  detail::runTestOctomap(checker, ContactTestType::ALL, expected_distance);

  DiscreteContactManager::Ptr cloned_checker = checker.clone();
  detail::runTestOctomap(*cloned_checker, ContactTestType::FIRST, expected_distance);
}

}  // namespace tesseract::collision::test_suite
#endif  // TESSERACT_COLLISION_COLLISION_OCTOMAP_OCTOMAP_UNIT_HPP
