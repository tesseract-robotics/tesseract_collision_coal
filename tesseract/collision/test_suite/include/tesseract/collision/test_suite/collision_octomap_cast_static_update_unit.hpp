#ifndef TESSERACT_COLLISION_COLLISION_OCTOMAP_CAST_STATIC_UPDATE_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_OCTOMAP_CAST_STATIC_UPDATE_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <octomap/octomap.h>
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/octree_subshape_id_contract.hpp>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/common.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>

namespace tesseract::collision::test_suite
{
namespace detail
{
/** @brief Locate the octree the fixtures below build. */
inline std::string fixtureOctreePath()
{
  tesseract::common::GeneralResourceLocator locator;
  return locator.locateResource("package://tesseract/support/meshes/box_2m.bt")->getFilePath();
}

inline void addOctreeAndCylinder(ContinuousContactManager& checker,
                                 const std::string& octree_name,
                                 const std::string& cylinder_name)
{
  auto ot = std::make_shared<octomap::OcTree>(fixtureOctreePath());

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
    return ((cr.link_ids[0] == "moving_octree" && cr.link_ids[1] == "probe_cylinder") ||
            (cr.link_ids[0] == "probe_cylinder" && cr.link_ids[1] == "moving_octree"));
  });
}

/** @brief Occupied-leaf count of the fixture octree; the file is immutable, so derive it once. */
inline int occupiedLeafCount()
{
  static const int count =
      static_cast<int>(tesseract::geometry::Octree(std::make_shared<octomap::OcTree>(fixtureOctreePath()),
                                                   tesseract::geometry::OctreeSubType::BOX)
                           .calcNumSubShapes());
  return count;
}

/**
 * @brief Collect the octree-side subshape ids of every contact between the named pair.
 *
 * Both links must report original geometry index 0; the octree side's subshape ids are returned in
 * contact order. Callers invoke this twice on an unchanged scene to check the ids do not move.
 */
inline std::vector<int> collectOctreeSubshapeIds(ContinuousContactManager& checker,
                                                 const std::string& octree_name,
                                                 const std::string& cylinder_name)
{
  ContactRequest request(ContactTestType::ALL);
  request.contact_limit = 2000;

  ContactResultMap result;
  checker.contactTest(result, request);

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  std::vector<int> subshape_ids;
  for (const auto& cr : result_vector)
  {
    const bool is_pair = (cr.link_ids[0] == octree_name && cr.link_ids[1] == cylinder_name) ||
                         (cr.link_ids[0] == cylinder_name && cr.link_ids[1] == octree_name);
    if (!is_pair)
      continue;

    const std::size_t octree_idx = (cr.link_ids[0] == octree_name) ? 0U : 1U;
    const std::size_t cylinder_idx = (cr.link_ids[0] == cylinder_name) ? 0U : 1U;

    EXPECT_EQ(cr.shape_id[octree_idx], 0)
        << octree_name << " should report original geometry index 0, but got " << cr.shape_id[octree_idx];
    EXPECT_EQ(cr.shape_id[cylinder_idx], 0)
        << cylinder_name << " should report original geometry index 0, but got " << cr.shape_id[cylinder_idx];

    subshape_ids.push_back(cr.subshape_id[octree_idx]);
  }
  return subshape_ids;
}

inline void runStaticOctreeCylinderContinuousTransformUpdatesBroadphase(ContinuousContactManager& checker)
{
  checker.setActiveCollisionObjects({ "active_cylinder" });
  checker.setDefaultCollisionMargin(0.0);

  Eigen::Isometry3d static_far = Eigen::Isometry3d::Identity();
  static_far.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);
  tesseract::common::LinkIdTransformMap static_far_tf;
  static_far_tf["static_octree"] = static_far;
  checker.setCollisionObjectsTransform(static_far_tf);

  const Eigen::Isometry3d static_origin = Eigen::Isometry3d::Identity();
  tesseract::common::LinkIdTransformMap static_origin_tf;
  static_origin_tf["static_octree"] = static_origin;
  checker.setCollisionObjectsTransform(static_origin_tf);

  Eigen::Isometry3d active_start = Eigen::Isometry3d::Identity();
  active_start.translation() = Eigen::Vector3d(-2.0, 0.0, 0.0);
  Eigen::Isometry3d active_end = Eigen::Isometry3d::Identity();
  active_end.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  tesseract::common::LinkIdTransformMap active_tf1;
  tesseract::common::LinkIdTransformMap active_tf2;
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
      if ((cr.link_ids[0] == "static_octree" && cr.link_ids[1] == "active_cylinder") ||
          (cr.link_ids[0] == "active_cylinder" && cr.link_ids[1] == "static_octree"))
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

  tesseract::common::LinkIdTransformMap static_tf;
  static_tf["static_octree"] = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(static_tf);

  Eigen::Isometry3d active_start = Eigen::Isometry3d::Identity();
  active_start.translation() = Eigen::Vector3d(-2.0, 0.0, 0.0);
  Eigen::Isometry3d active_end = Eigen::Isometry3d::Identity();
  active_end.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  std::vector<tesseract::common::LinkId> link_ids{ "active_cylinder" };
  tesseract::common::VectorIsometry3d starts{ active_start };
  tesseract::common::VectorIsometry3d ends{ active_end };
  checker.setCollisionObjectsTransform(link_ids, starts, ends);

  for (ContactTestType type : { ContactTestType::CLOSEST, ContactTestType::ALL })
  {
    ContactResultMap result;
    checker.contactTest(result, ContactRequest(type));

    ContactResultVector result_vector;
    result.flattenMoveResults(result_vector);

    bool found_pair = false;
    for (const auto& cr : result_vector)
    {
      if ((cr.link_ids[0] == "static_octree" && cr.link_ids[1] == "active_cylinder") ||
          (cr.link_ids[0] == "active_cylinder" && cr.link_ids[1] == "static_octree"))
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
    if ((cr.link_ids[0] == "static_octree" && cr.link_ids[1] == "active_cylinder") ||
        (cr.link_ids[0] == "active_cylinder" && cr.link_ids[1] == "static_octree"))
    {
      found_pair = true;
      const int octree_shape_id = (cr.link_ids[0] == "static_octree") ? cr.shape_id[0] : cr.shape_id[1];
      const int cylinder_shape_id = (cr.link_ids[0] == "active_cylinder") ? cr.shape_id[0] : cr.shape_id[1];
      EXPECT_EQ(octree_shape_id, 0) << "Static octree should report original geometry index 0, but got "
                                    << octree_shape_id;
      EXPECT_EQ(cylinder_shape_id, 0) << "Active cylinder should report original geometry index 0, but got "
                                      << cylinder_shape_id;
      EXPECT_LT(cr.distance, 0.11) << "Expected contact/penetration for shape-id validation scenario";
    }
  }

  EXPECT_TRUE(found_pair) << "Expected contact between static_octree and active_cylinder";
}

/** @brief A static octree contact's subshape_id must name the primitive that was hit. */
inline void runStaticOctreeSubshapeIdNamesPrimitive(ContinuousContactManager& checker, OctreeSubshapeIdKind kind)
{
  checker.setActiveCollisionObjects({ "active_cylinder" });
  checker.setDefaultCollisionMargin(0.0);

  Eigen::Isometry3d active_start = Eigen::Isometry3d::Identity();
  active_start.translation() = Eigen::Vector3d(-2.0, 0.0, 0.0);
  Eigen::Isometry3d active_end = Eigen::Isometry3d::Identity();
  active_end.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);
  checker.setCollisionObjectsTransform("active_cylinder", active_start, active_end);

  auto query = [&checker]() { return collectOctreeSubshapeIds(checker, "static_octree", "active_cylinder"); };

  const std::vector<int> subshape_ids = query();
  ASSERT_FALSE(subshape_ids.empty()) << "Expected contact between static_octree and active_cylinder";

  for (int subshape_id : subshape_ids)
    expectOctreeSubshapeIdNamesPrimitive(subshape_id, kind, occupiedLeafCount());

  expectOctreeSubshapeIdsDiscriminate(subshape_ids);
  expectOctreeSubshapeIdsStable(subshape_ids, query());
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

/** @brief An active octree contact's subshape_id must name the primitive that was hit. */
inline void runActiveOctreeSubshapeIdNamesPrimitive(ContinuousContactManager& checker)
{
  checker.setDefaultCollisionMargin(0.0);
  checker.setActiveCollisionObjects({ "moving_octree" });

  const Eigen::Isometry3d start = Eigen::Isometry3d(Eigen::Translation3d(-4.0, 0.0, 0.0));
  const Eigen::Isometry3d end = Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.0));
  checker.setCollisionObjectsTransform("moving_octree", start, end);

  auto query = [&checker]() { return collectOctreeSubshapeIds(checker, "moving_octree", "probe_cylinder"); };

  const std::vector<int> subshape_ids = query();
  ASSERT_FALSE(subshape_ids.empty()) << "Expected contact between moving_octree and probe_cylinder";

  // An active octree is swept, which every backend implements by expanding it into per-voxel
  // shapes, so the reported id is an index into those shapes whatever the backend does statically.
  for (int subshape_id : subshape_ids)
    expectOctreeSubshapeIdNamesPrimitive(subshape_id, OctreeSubshapeIdKind::LeafOrdinal, occupiedLeafCount());

  expectOctreeSubshapeIdsDiscriminate(subshape_ids);
  expectOctreeSubshapeIdsStable(subshape_ids, query());
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

inline void runTestStaticOctreeSubshapeIdNamesPrimitive(ContinuousContactManager& checker, OctreeSubshapeIdKind kind)
{
  detail::addStaticOctreeAndActiveCylinder(checker);
  detail::addStaticOctreeAndActiveCylinder(checker);
  detail::runStaticOctreeSubshapeIdNamesPrimitive(checker, kind);

  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runStaticOctreeSubshapeIdNamesPrimitive(*cloned, kind);
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

inline void runTestActiveOctreeSubshapeIdNamesPrimitive(ContinuousContactManager& checker)
{
  detail::addActiveOctreeAndProbeCylinder(checker);
  detail::addActiveOctreeAndProbeCylinder(checker);
  detail::runActiveOctreeSubshapeIdNamesPrimitive(checker);

  ContinuousContactManager::Ptr cloned = checker.clone();
  detail::runActiveOctreeSubshapeIdNamesPrimitive(*cloned);
}

}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_OCTOMAP_CAST_STATIC_UPDATE_UNIT_HPP
