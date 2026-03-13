#ifndef TESSERACT_COLLISION_COLLISION_OCTOMAP_CAST_STATIC_UPDATE_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_OCTOMAP_CAST_STATIC_UPDATE_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <octomap/octomap.h>
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/common.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>

namespace tesseract::collision::test_suite
{
namespace detail
{
inline void addStaticOctreeAndActiveCylinder(ContinuousContactManager& checker)
{
  tesseract::common::GeneralResourceLocator locator;
  std::string path = locator.locateResource("package://tesseract/support/meshes/box_2m.bt")->getFilePath();
  auto ot = std::make_shared<octomap::OcTree>(path);

  CollisionShapesConst static_shapes;
  tesseract::common::VectorIsometry3d static_shape_poses;
  static_shapes.push_back(std::make_shared<tesseract::geometry::Octree>(ot, tesseract::geometry::OctreeSubType::BOX));
  static_shape_poses.push_back(Eigen::Isometry3d::Identity());
  checker.addCollisionObject("static_octree", 0, static_shapes, static_shape_poses);

  CollisionShapesConst active_shapes;
  tesseract::common::VectorIsometry3d active_shape_poses;
  active_shapes.push_back(std::make_shared<tesseract::geometry::Cylinder>(0.1, 0.5));
  active_shape_poses.push_back(Eigen::Isometry3d::Identity());
  checker.addCollisionObject("active_cylinder", 0, active_shapes, active_shape_poses);

  EXPECT_EQ(checker.getCollisionObjects().size(), 2);
}

inline void runStaticOctreeCylinderContinuousTransformUpdatesBroadphase(ContinuousContactManager& checker)
{
  checker.setActiveCollisionObjects({ "active_cylinder" });
  checker.setDefaultCollisionMargin(0.0);

  Eigen::Isometry3d static_far = Eigen::Isometry3d::Identity();
  static_far.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);
  tesseract::common::TransformMap static_far_tf;
  static_far_tf["static_octree"] = static_far;
  checker.setCollisionObjectsTransform(static_far_tf);

  const Eigen::Isometry3d static_origin = Eigen::Isometry3d::Identity();
  tesseract::common::TransformMap static_origin_tf;
  static_origin_tf["static_octree"] = static_origin;
  checker.setCollisionObjectsTransform(static_origin_tf);

  Eigen::Isometry3d active_start = Eigen::Isometry3d::Identity();
  active_start.translation() = Eigen::Vector3d(-2.0, 0.0, 0.0);
  Eigen::Isometry3d active_end = Eigen::Isometry3d::Identity();
  active_end.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  tesseract::common::TransformMap active_tf1;
  tesseract::common::TransformMap active_tf2;
  active_tf1["active_cylinder"] = active_start;
  active_tf2["active_cylinder"] = active_end;
  checker.setCollisionObjectsTransform(active_tf1, active_tf2);

  for (ContactTestType type : { ContactTestType::FIRST, ContactTestType::CLOSEST, ContactTestType::ALL })
  {
    ContactResultMap result;
    checker.contactTest(result, ContactRequest(type));

    ContactResultVector result_vector;
    result.flattenMoveResults(result_vector);

    bool found_pair = false;
    for (const auto& cr : result_vector)
    {
      if ((cr.link_names[0] == "static_octree" && cr.link_names[1] == "active_cylinder") ||
          (cr.link_names[0] == "active_cylinder" && cr.link_names[1] == "static_octree"))
      {
        found_pair = true;
        EXPECT_LT(cr.distance, 0.11) << "Expected contact/penetration for static_octree vs active_cylinder";
      }
    }

    EXPECT_TRUE(found_pair) << "Expected collision between static_octree and active_cylinder";
  }
}

inline void runStaticOctreeCylinderActiveToggleStillCollides(ContinuousContactManager& checker)
{
  checker.setDefaultCollisionMargin(0.0);

  checker.setActiveCollisionObjects({ "active_cylinder" });
  checker.setActiveCollisionObjects({});
  checker.setActiveCollisionObjects({ "active_cylinder" });

  tesseract::common::TransformMap static_tf;
  static_tf["static_octree"] = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(static_tf);

  Eigen::Isometry3d active_start = Eigen::Isometry3d::Identity();
  active_start.translation() = Eigen::Vector3d(-2.0, 0.0, 0.0);
  Eigen::Isometry3d active_end = Eigen::Isometry3d::Identity();
  active_end.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  std::vector<std::string> names{ "active_cylinder" };
  tesseract::common::VectorIsometry3d starts{ active_start };
  tesseract::common::VectorIsometry3d ends{ active_end };
  checker.setCollisionObjectsTransform(names, starts, ends);

  for (ContactTestType type : { ContactTestType::CLOSEST, ContactTestType::ALL })
  {
    ContactResultMap result;
    checker.contactTest(result, ContactRequest(type));

    ContactResultVector result_vector;
    result.flattenMoveResults(result_vector);

    bool found_pair = false;
    for (const auto& cr : result_vector)
    {
      if ((cr.link_names[0] == "static_octree" && cr.link_names[1] == "active_cylinder") ||
          (cr.link_names[0] == "active_cylinder" && cr.link_names[1] == "static_octree"))
      {
        found_pair = true;
        EXPECT_LT(cr.distance, 0.11) << "Expected contact/penetration after active set toggling";
      }
    }

    EXPECT_TRUE(found_pair) << "Expected collision after active set toggling";
  }
}

inline void runStaticOctreeCylinderShapeIdUsesOriginalGeometryIndex(ContinuousContactManager& checker)
{
  checker.setActiveCollisionObjects({ "active_cylinder" });
  checker.setDefaultCollisionMargin(0.0);

  Eigen::Isometry3d active_start = Eigen::Isometry3d::Identity();
  active_start.translation() = Eigen::Vector3d(-2.0, 0.0, 0.0);
  Eigen::Isometry3d active_end = Eigen::Isometry3d::Identity();
  active_end.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);
  checker.setCollisionObjectsTransform("active_cylinder", active_start, active_end);

  ContactRequest request(ContactTestType::ALL);
  request.contact_limit = 2000;

  ContactResultMap result;
  checker.contactTest(result, request);

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty());

  bool found_pair = false;
  for (const auto& cr : result_vector)
  {
    if ((cr.link_names[0] == "static_octree" && cr.link_names[1] == "active_cylinder") ||
        (cr.link_names[0] == "active_cylinder" && cr.link_names[1] == "static_octree"))
    {
      found_pair = true;
      const int octree_shape_id = (cr.link_names[0] == "static_octree") ? cr.shape_id[0] : cr.shape_id[1];
        const int cylinder_shape_id = (cr.link_names[0] == "active_cylinder") ? cr.shape_id[0] : cr.shape_id[1];
      EXPECT_EQ(octree_shape_id, 0)
          << "Static octree should report original geometry index 0, but got " << octree_shape_id;
        EXPECT_EQ(cylinder_shape_id, 0)
          << "Active cylinder should report original geometry index 0, but got " << cylinder_shape_id;
        EXPECT_LT(cr.distance, 0.11) << "Expected contact/penetration for shape-id validation scenario";
    }
  }

  EXPECT_TRUE(found_pair) << "Expected contact between static_octree and active_cylinder";
}
}  // namespace detail

inline void runTestStaticOctreeCylinderContinuousTransformUpdatesBroadphase(ContinuousContactManager& checker)
{
  detail::addStaticOctreeAndActiveCylinder(checker);
  detail::addStaticOctreeAndActiveCylinder(checker);
  detail::runStaticOctreeCylinderContinuousTransformUpdatesBroadphase(checker);

  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runStaticOctreeCylinderContinuousTransformUpdatesBroadphase(*cloned);
}

inline void runTestStaticOctreeCylinderActiveToggleStillCollides(ContinuousContactManager& checker)
{
  detail::addStaticOctreeAndActiveCylinder(checker);
  detail::addStaticOctreeAndActiveCylinder(checker);
  detail::runStaticOctreeCylinderActiveToggleStillCollides(checker);

  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runStaticOctreeCylinderActiveToggleStillCollides(*cloned);
}

inline void runTestStaticOctreeCylinderShapeIdUsesOriginalGeometryIndex(ContinuousContactManager& checker)
{
  detail::addStaticOctreeAndActiveCylinder(checker);
  detail::addStaticOctreeAndActiveCylinder(checker);
  detail::runStaticOctreeCylinderShapeIdUsesOriginalGeometryIndex(checker);

  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runStaticOctreeCylinderShapeIdUsesOriginalGeometryIndex(*cloned);
}

}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_OCTOMAP_CAST_STATIC_UPDATE_UNIT_HPP
