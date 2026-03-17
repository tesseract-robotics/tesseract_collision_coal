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
inline void addOctreeAndCylinder(ContinuousContactManager& checker,
                                 const std::string& octree_name,
                                 const std::string& cylinder_name)
{
  tesseract::common::GeneralResourceLocator locator;
  std::string path = locator.locateResource("package://tesseract/support/meshes/box_2m.bt")->getFilePath();
  auto ot = std::make_shared<octomap::OcTree>(path);

  CollisionShapesConst octree_shapes;
  tesseract::common::VectorIsometry3d octree_poses;
  octree_shapes.push_back(std::make_shared<tesseract::geometry::Octree>(ot, tesseract::geometry::OctreeSubType::BOX));
  octree_poses.push_back(Eigen::Isometry3d::Identity());
  checker.addCollisionObject(octree_name, 0, octree_shapes, octree_poses);

  CollisionShapesConst cylinder_shapes;
  tesseract::common::VectorIsometry3d cylinder_poses;
  cylinder_shapes.push_back(std::make_shared<tesseract::geometry::Cylinder>(0.1, 0.5));
  cylinder_poses.push_back(Eigen::Isometry3d::Identity());
  checker.addCollisionObject(cylinder_name, 0, cylinder_shapes, cylinder_poses);

  EXPECT_EQ(checker.getCollisionObjects().size(), 2);
}

inline void addStaticOctreeAndActiveCylinder(ContinuousContactManager& checker)
{
  addOctreeAndCylinder(checker, "static_octree", "active_cylinder");
}

inline void addActiveOctreeAndProbeCylinder(ContinuousContactManager& checker)
{
  addOctreeAndCylinder(checker, "moving_octree", "probe_cylinder");
}

inline bool hasMovingOctreeProbePair(const ContactResultVector& result_vector)
{
  return std::any_of(result_vector.begin(), result_vector.end(), [](const ContactResult& cr) {
    return ((cr.link_names[0] == "moving_octree" && cr.link_names[1] == "probe_cylinder") ||
            (cr.link_names[0] == "probe_cylinder" && cr.link_names[1] == "moving_octree"));
  });
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
      EXPECT_EQ(octree_shape_id, 0) << "Static octree should report original geometry index 0, but got "
                                    << octree_shape_id;
      EXPECT_EQ(cylinder_shape_id, 0) << "Active cylinder should report original geometry index 0, but got "
                                      << cylinder_shape_id;
      EXPECT_LT(cr.distance, 0.11) << "Expected contact/penetration for shape-id validation scenario";
    }
  }

  EXPECT_TRUE(found_pair) << "Expected contact between static_octree and active_cylinder";
}

inline void runStaticOctreeSubshapeIdReportsPrimitiveIdentity(ContinuousContactManager& checker)
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
  bool found_octree_subshape = false;
  for (const auto& cr : result_vector)
  {
    if ((cr.link_names[0] == "static_octree" && cr.link_names[1] == "active_cylinder") ||
        (cr.link_names[0] == "active_cylinder" && cr.link_names[1] == "static_octree"))
    {
      found_pair = true;
      const std::size_t octree_idx = (cr.link_names[0] == "static_octree") ? 0U : 1U;
      const std::size_t cylinder_idx = (cr.link_names[0] == "active_cylinder") ? 0U : 1U;

      EXPECT_EQ(cr.shape_id[octree_idx], 0)
          << "Static octree should report original geometry index 0, but got " << cr.shape_id[octree_idx];
      EXPECT_EQ(cr.shape_id[cylinder_idx], 0)
          << "Active cylinder should report original geometry index 0, but got " << cr.shape_id[cylinder_idx];

      if (cr.subshape_id[octree_idx] >= 0)
        found_octree_subshape = true;
    }
  }

  EXPECT_TRUE(found_pair) << "Expected contact between static_octree and active_cylinder";
  EXPECT_TRUE(found_octree_subshape) << "Static octree should report a primitive subshape_id for at least one contact "
                                        "result. "
                                     << "If this stays unset, the backend is losing octree primitive identity on the "
                                        "continuous path.";
}

inline void runActiveOctreeDemotionClearsSweepState(ContinuousContactManager& checker)
{
  checker.setDefaultCollisionMargin(0.0);

  checker.setActiveCollisionObjects({ "moving_octree" });

  Eigen::Isometry3d octree_start = Eigen::Isometry3d::Identity();
  octree_start.translation() = Eigen::Vector3d(-4.0, 0.0, 0.0);
  Eigen::Isometry3d octree_end = Eigen::Isometry3d::Identity();
  octree_end.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  checker.setCollisionObjectsTransform("moving_octree", octree_start, octree_end);

  checker.setActiveCollisionObjects({ "probe_cylinder" });

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  bool found_pair = hasMovingOctreeProbePair(result_vector);

  EXPECT_FALSE(found_pair) << "Demoting a previously swept octree back to static should clear its swept extent. "
                           << "A collision here indicates stale CastHull sweep state is still being used for the "
                           << "static representation.";
}

inline void runActiveOctreeDisabledSweepDoesNotUpdateCastState(ContinuousContactManager& checker)
{
  checker.setDefaultCollisionMargin(0.0);
  checker.setActiveCollisionObjects({ "moving_octree" });

  const Eigen::Isometry3d start = Eigen::Isometry3d(Eigen::Translation3d(-4.0, 0.0, 0.0));
  const Eigen::Isometry3d end = Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.0));

  checker.disableCollisionObject("moving_octree");
  checker.setCollisionObjectsTransform("moving_octree", start, end);
  checker.enableCollisionObject("moving_octree");

  // Refresh the broadphase at the start pose while leaving cast sweep state untouched.
  checker.setCollisionObjectsTransform("moving_octree", start);

  ContactResultMap no_sweep_result;
  checker.contactTest(no_sweep_result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector no_sweep_vector;
  no_sweep_result.flattenMoveResults(no_sweep_vector);

  EXPECT_FALSE(hasMovingOctreeProbePair(no_sweep_vector)) << "A disabled octree sweep should not update cast state or "
                                                             "produce a swept collision volume.";

  checker.setCollisionObjectsTransform("moving_octree", start, end);

  ContactResultMap sweep_result;
  checker.contactTest(sweep_result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector sweep_vector;
  sweep_result.flattenMoveResults(sweep_vector);

  EXPECT_TRUE(hasMovingOctreeProbePair(sweep_vector)) << "After an enabled sweep update, the active octree should "
                                                         "collide with the probe cylinder.";
}

inline void runActiveOctreeRoundTripActiveSetTransitions(ContinuousContactManager& checker)
{
  checker.setDefaultCollisionMargin(0.0);

  const Eigen::Isometry3d start = Eigen::Isometry3d(Eigen::Translation3d(-4.0, 0.0, 0.0));
  const Eigen::Isometry3d end = Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.0));

  checker.setActiveCollisionObjects({ "moving_octree" });
  checker.setCollisionObjectsTransform("moving_octree", start, end);

  ContactResultMap first_active_result;
  checker.contactTest(first_active_result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector first_active_vector;
  first_active_result.flattenMoveResults(first_active_vector);
  EXPECT_TRUE(hasMovingOctreeProbePair(first_active_vector)) << "Active octree should collide with the probe during "
                                                                "the initial sweep.";

  checker.setActiveCollisionObjects({ "probe_cylinder" });

  ContactResultMap first_static_result;
  checker.contactTest(first_static_result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector first_static_vector;
  first_static_result.flattenMoveResults(first_static_vector);
  EXPECT_FALSE(hasMovingOctreeProbePair(first_static_vector)) << "After demotion to static, the octree should not "
                                                                 "retain swept collision volume.";

  checker.setActiveCollisionObjects({ "moving_octree" });
  checker.setCollisionObjectsTransform("moving_octree", start);
  checker.setCollisionObjectsTransform("moving_octree", start, end);

  ContactResultMap second_active_result;
  checker.contactTest(second_active_result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector second_active_vector;
  second_active_result.flattenMoveResults(second_active_vector);
  EXPECT_TRUE(hasMovingOctreeProbePair(second_active_vector)) << "Re-activating the octree and applying a new sweep "
                                                                 "should restore the expected collision.";

  checker.setActiveCollisionObjects({ "probe_cylinder" });

  ContactResultMap second_static_result;
  checker.contactTest(second_static_result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector second_static_vector;
  second_static_result.flattenMoveResults(second_static_vector);
  EXPECT_FALSE(hasMovingOctreeProbePair(second_static_vector)) << "A second demotion back to static should also clear "
                                                                  "any swept extent.";
}

inline void runActiveOctreeSubshapeIdReportsPrimitiveIdentity(ContinuousContactManager& checker)
{
  checker.setDefaultCollisionMargin(0.0);
  checker.setActiveCollisionObjects({ "moving_octree" });

  const Eigen::Isometry3d start = Eigen::Isometry3d(Eigen::Translation3d(-4.0, 0.0, 0.0));
  const Eigen::Isometry3d end = Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.0));
  checker.setCollisionObjectsTransform("moving_octree", start, end);

  ContactRequest request(ContactTestType::ALL);
  request.contact_limit = 2000;

  ContactResultMap result;
  checker.contactTest(result, request);

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty());

  bool found_pair = false;
  bool found_octree_subshape = false;
  for (const auto& cr : result_vector)
  {
    if ((cr.link_names[0] == "moving_octree" && cr.link_names[1] == "probe_cylinder") ||
        (cr.link_names[0] == "probe_cylinder" && cr.link_names[1] == "moving_octree"))
    {
      found_pair = true;
      const std::size_t octree_idx = (cr.link_names[0] == "moving_octree") ? 0U : 1U;
      const std::size_t cylinder_idx = (cr.link_names[0] == "probe_cylinder") ? 0U : 1U;

      EXPECT_EQ(cr.shape_id[octree_idx], 0)
          << "Active octree should report original geometry index 0, but got " << cr.shape_id[octree_idx];
      EXPECT_EQ(cr.shape_id[cylinder_idx], 0)
          << "Probe cylinder should report original geometry index 0, but got " << cr.shape_id[cylinder_idx];

      if (cr.subshape_id[octree_idx] >= 0)
        found_octree_subshape = true;
    }
  }

  EXPECT_TRUE(found_pair) << "Expected contact between moving_octree and probe_cylinder";
  EXPECT_TRUE(found_octree_subshape) << "Active octree should report a primitive subshape_id for at least one contact "
                                        "result. "
                                     << "If this stays unset, the backend is losing octree primitive identity on the "
                                        "continuous path.";
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

inline void runTestStaticOctreeSubshapeIdReportsPrimitiveIdentity(ContinuousContactManager& checker)
{
  detail::addStaticOctreeAndActiveCylinder(checker);
  detail::addStaticOctreeAndActiveCylinder(checker);
  detail::runStaticOctreeSubshapeIdReportsPrimitiveIdentity(checker);

  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runStaticOctreeSubshapeIdReportsPrimitiveIdentity(*cloned);
}

inline void runTestActiveOctreeDemotionClearsSweepState(ContinuousContactManager& checker)
{
  detail::addActiveOctreeAndProbeCylinder(checker);
  detail::addActiveOctreeAndProbeCylinder(checker);
  detail::runActiveOctreeDemotionClearsSweepState(checker);

  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runActiveOctreeDemotionClearsSweepState(*cloned);
}

inline void runTestActiveOctreeDisabledSweepDoesNotUpdateCastState(ContinuousContactManager& checker)
{
  detail::addActiveOctreeAndProbeCylinder(checker);
  detail::addActiveOctreeAndProbeCylinder(checker);
  detail::runActiveOctreeDisabledSweepDoesNotUpdateCastState(checker);

  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runActiveOctreeDisabledSweepDoesNotUpdateCastState(*cloned);
}

inline void runTestActiveOctreeRoundTripActiveSetTransitions(ContinuousContactManager& checker)
{
  detail::addActiveOctreeAndProbeCylinder(checker);
  detail::addActiveOctreeAndProbeCylinder(checker);
  detail::runActiveOctreeRoundTripActiveSetTransitions(checker);

  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runActiveOctreeRoundTripActiveSetTransitions(*cloned);
}

inline void runTestActiveOctreeSubshapeIdReportsPrimitiveIdentity(ContinuousContactManager& checker)
{
  detail::addActiveOctreeAndProbeCylinder(checker);
  detail::addActiveOctreeAndProbeCylinder(checker);
  detail::runActiveOctreeSubshapeIdReportsPrimitiveIdentity(checker);

  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runActiveOctreeSubshapeIdReportsPrimitiveIdentity(*cloned);
}

}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_OCTOMAP_CAST_STATIC_UPDATE_UNIT_HPP
