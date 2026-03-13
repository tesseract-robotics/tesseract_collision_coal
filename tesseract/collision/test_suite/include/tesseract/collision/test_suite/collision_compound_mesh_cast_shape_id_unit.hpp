#ifndef TESSERACT_COLLISION_COLLISION_COMPOUND_MESH_CAST_SHAPE_ID_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_COMPOUND_MESH_CAST_SHAPE_ID_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/common.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/mesh_parser.h>
#include <tesseract/common/resource_locator.h>

namespace tesseract::collision::test_suite
{
namespace detail
{
inline void addCompoundMeshAndActiveSphere(ContinuousContactManager& checker)
{
  auto resource_locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
  auto compound_mesh_resource = resource_locator->locateResource("package://tesseract/support/meshes/box_box.dae");
  auto meshes =
      tesseract::geometry::createMeshFromPath<tesseract::geometry::ConvexMesh>(compound_mesh_resource->getFilePath());

  CollisionShapePtr compound_mesh = std::make_shared<tesseract::geometry::CompoundMesh>(meshes);

  CollisionShapesConst obj1_shapes;
  tesseract::common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(compound_mesh);
  obj1_poses.push_back(Eigen::Isometry3d::Identity());

  checker.addCollisionObject("compound_link", 0, obj1_shapes, obj1_poses);

  CollisionShapePtr sphere = std::make_shared<tesseract::geometry::Sphere>(0.25);

  CollisionShapesConst obj2_shapes;
  tesseract::common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(sphere);
  obj2_poses.push_back(Eigen::Isometry3d::Identity());

  checker.addCollisionObject("sphere_link", 0, obj2_shapes, obj2_poses);

  EXPECT_EQ(checker.getCollisionObjects().size(), 2);
}

inline void runCompoundMeshShapeIdUsesOriginalGeometryIndex(ContinuousContactManager& checker)
{
  checker.setActiveCollisionObjects({ "sphere_link" });
  checker.setDefaultCollisionMargin(0.1);

  // Keep compound mesh static at identity
  tesseract::common::TransformMap location;
  location["compound_link"] = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);

  // Sweep sphere into the compound mesh volume.
  Eigen::Isometry3d start_pos = Eigen::Isometry3d::Identity();
  start_pos.translation() = Eigen::Vector3d(0.0, 0.0, 2.0);
  Eigen::Isometry3d end_pos = Eigen::Isometry3d::Identity();
  end_pos.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  checker.setCollisionObjectsTransform("sphere_link", start_pos, end_pos);

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
    if ((cr.link_names[0] == "compound_link" && cr.link_names[1] == "sphere_link") ||
        (cr.link_names[0] == "sphere_link" && cr.link_names[1] == "compound_link"))
    {
      found_pair = true;
      const int compound_shape_id = (cr.link_names[0] == "compound_link") ? cr.shape_id[0] : cr.shape_id[1];
      const int sphere_shape_id = (cr.link_names[0] == "sphere_link") ? cr.shape_id[0] : cr.shape_id[1];

      EXPECT_EQ(compound_shape_id, 0)
          << "Compound mesh should report original geometry index 0, but got " << compound_shape_id;
      EXPECT_EQ(sphere_shape_id, 0) << "Sphere should report original geometry index 0, but got " << sphere_shape_id;
      EXPECT_LT(cr.distance, 0.11) << "Expected contact/penetration for compound mesh cast shape-id scenario";
    }
  }

  EXPECT_TRUE(found_pair) << "Expected contact between compound_link and sphere_link";
}

inline void runCompoundMeshSubshapeIdReportsPrimitiveIdentity(ContinuousContactManager& checker)
{
  checker.setActiveCollisionObjects({ "sphere_link" });
  checker.setDefaultCollisionMargin(0.1);

  tesseract::common::TransformMap location;
  location["compound_link"] = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);

  Eigen::Isometry3d start_pos = Eigen::Isometry3d::Identity();
  start_pos.translation() = Eigen::Vector3d(0.0, 0.0, 2.0);
  Eigen::Isometry3d end_pos = Eigen::Isometry3d::Identity();
  end_pos.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  checker.setCollisionObjectsTransform("sphere_link", start_pos, end_pos);

  ContactRequest request(ContactTestType::ALL);
  request.contact_limit = 2000;

  ContactResultMap result;
  checker.contactTest(result, request);

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty());

  bool found_pair = false;
  bool found_compound_subshape = false;
  for (const auto& cr : result_vector)
  {
    if ((cr.link_names[0] == "compound_link" && cr.link_names[1] == "sphere_link") ||
        (cr.link_names[0] == "sphere_link" && cr.link_names[1] == "compound_link"))
    {
      found_pair = true;
      const std::size_t compound_idx = (cr.link_names[0] == "compound_link") ? 0U : 1U;
      const std::size_t sphere_idx = (cr.link_names[0] == "sphere_link") ? 0U : 1U;

      EXPECT_EQ(cr.shape_id[compound_idx], 0)
          << "Compound mesh should report original geometry index 0, but got " << cr.shape_id[compound_idx];
      EXPECT_EQ(cr.shape_id[sphere_idx], 0)
          << "Sphere should report original geometry index 0, but got " << cr.shape_id[sphere_idx];

      if (cr.subshape_id[compound_idx] >= 0)
        found_compound_subshape = true;
    }
  }

  EXPECT_TRUE(found_pair) << "Expected contact between compound_link and sphere_link";
  EXPECT_TRUE(found_compound_subshape)
      << "Compound mesh should report a primitive subshape_id for at least one contact result. "
      << "If this stays unset, the backend is losing compound child identity on the continuous path.";
}
}  // namespace detail

inline void runTestCompoundMeshCastShapeIdUsesOriginalGeometryIndex(ContinuousContactManager& checker)
{
  detail::addCompoundMeshAndActiveSphere(checker);
  detail::addCompoundMeshAndActiveSphere(checker);
  detail::runCompoundMeshShapeIdUsesOriginalGeometryIndex(checker);

  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runCompoundMeshShapeIdUsesOriginalGeometryIndex(*cloned);
}

inline void runTestCompoundMeshCastSubshapeIdReportsPrimitiveIdentity(ContinuousContactManager& checker)
{
  detail::addCompoundMeshAndActiveSphere(checker);
  detail::addCompoundMeshAndActiveSphere(checker);
  detail::runCompoundMeshSubshapeIdReportsPrimitiveIdentity(checker);

  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runCompoundMeshSubshapeIdReportsPrimitiveIdentity(*cloned);
}

}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_COMPOUND_MESH_CAST_SHAPE_ID_UNIT_HPP
