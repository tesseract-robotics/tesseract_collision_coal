#ifndef TESSERACT_COLLISION_COLLISION_OCTOMAP_SUBSHAPE_ID_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_OCTOMAP_SUBSHAPE_ID_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <octomap/octomap.h>
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/octree_subshape_id_contract.hpp>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>

namespace tesseract::collision::test_suite
{
/** @brief A discrete octree contact's subshape_id must name the primitive that was hit. */
inline void runTestDiscreteOctreeSubshapeIdNamesPrimitive(DiscreteContactManager& checker, OctreeSubshapeIdKind kind)
{
  tesseract::common::GeneralResourceLocator locator;
  const std::string path = locator.locateResource("package://tesseract/support/meshes/box_2m.bt")->getFilePath();
  auto ot = std::make_shared<octomap::OcTree>(path);
  auto octree_geom = std::make_shared<tesseract::geometry::Octree>(ot, tesseract::geometry::OctreeSubType::BOX);

  const auto leaf_count = static_cast<int>(octree_geom->calcNumSubShapes());
  ASSERT_GT(leaf_count, 0);

  CollisionShapesConst octree_shapes;
  tesseract::common::VectorIsometry3d octree_poses;
  octree_shapes.push_back(octree_geom);
  octree_poses.push_back(Eigen::Isometry3d::Identity());
  checker.addCollisionObject("octomap_link", 0, octree_shapes, octree_poses);

  CollisionShapesConst sphere_shapes;
  tesseract::common::VectorIsometry3d sphere_poses;
  sphere_shapes.push_back(std::make_shared<tesseract::geometry::Sphere>(0.25));
  sphere_poses.push_back(Eigen::Isometry3d::Identity());
  checker.addCollisionObject("sphere_link", 0, sphere_shapes, sphere_poses);

  checker.setActiveCollisionObjects({ "octomap_link", "sphere_link" });
  checker.setDefaultCollisionMargin(0.1);

  tesseract::common::LinkIdTransformMap location;
  location["octomap_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);

  auto query = [&checker]() {
    ContactResultMap result;
    checker.contactTest(result, ContactRequest(ContactTestType::ALL));

    ContactResultVector result_vector;
    result.flattenMoveResults(result_vector);

    std::vector<int> subshape_ids;
    for (const auto& cr : result_vector)
    {
      EXPECT_TRUE(cr.link_ids[0] == "octomap_link" || cr.link_ids[1] == "octomap_link");
      const std::size_t octree_idx = (cr.link_ids[0] == "octomap_link") ? 0U : 1U;
      subshape_ids.push_back(cr.subshape_id[octree_idx]);
    }
    return subshape_ids;
  };

  const std::vector<int> subshape_ids = query();
  ASSERT_FALSE(subshape_ids.empty());

  for (int subshape_id : subshape_ids)
    expectOctreeSubshapeIdNamesPrimitive(subshape_id, kind, leaf_count);

  expectOctreeSubshapeIdsDiscriminate(subshape_ids);
  expectOctreeSubshapeIdsStable(subshape_ids, query());
}

}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_OCTOMAP_SUBSHAPE_ID_UNIT_HPP
