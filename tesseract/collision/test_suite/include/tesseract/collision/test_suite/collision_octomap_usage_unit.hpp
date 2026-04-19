#ifndef TESSERACT_COLLISION_COLLISION_OCTOMAP_USAGE_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_OCTOMAP_USAGE_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <octomap/octomap.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <algorithm>

#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>

namespace tesseract::collision::test_suite
{
namespace detail
{
inline std::shared_ptr<octomap::OcTree> loadOctreeBox2m()
{
  tesseract::common::GeneralResourceLocator locator;
  std::string path = locator.locateResource("package://tesseract/support/meshes/box_2m.bt")->getFilePath();
  return std::make_shared<octomap::OcTree>(path);
}

inline bool hasPair(const ContactResultVector& results, const std::string& a, const std::string& b)
{
  return std::any_of(results.begin(), results.end(), [&a, &b](const ContactResult& cr) {
    return ((cr.link_ids[0].name() == a && cr.link_ids[1].name() == b) || (cr.link_ids[0].name() == b && cr.link_ids[1].name() == a));
  });
}

inline const ContactResult* findPair(const ContactResultVector& results, const std::string& a, const std::string& b)
{
  auto it = std::find_if(results.begin(), results.end(), [&a, &b](const ContactResult& cr) {
    return ((cr.link_ids[0].name() == a && cr.link_ids[1].name() == b) || (cr.link_ids[0].name() == b && cr.link_ids[1].name() == a));
  });

  if (it == results.end())
    return nullptr;

  return &(*it);
}

inline std::size_t getLinkIndex(const ContactResult& cr, const std::string& name)
{
  if (cr.link_ids[0].name() == name)
    return 0;

  if (cr.link_ids[1].name() == name)
    return 1;

  ADD_FAILURE() << "Link name '" << name << "' not found in contact result (link_names: '" << cr.link_ids[0].name() << "', '"
                << cr.link_ids[1].name() << "')";
  return 0;
}

inline ContactResultVector runClosest(DiscreteContactManager& checker)
{
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector result_vec;
  result.flattenMoveResults(result_vec);
  return result_vec;
}

inline ContactResultVector runClosest(ContinuousContactManager& checker)
{
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector result_vec;
  result.flattenMoveResults(result_vec);
  return result_vec;
}

inline void addDiscreteOctreeAndSphere(DiscreteContactManager& checker, tesseract::geometry::OctreeSubType subtype)
{
  auto ot = loadOctreeBox2m();

  CollisionShapesConst octree_shapes;
  tesseract::common::VectorIsometry3d octree_poses;
  octree_shapes.push_back(std::make_shared<tesseract::geometry::Octree>(ot, subtype));
  octree_poses.push_back(Eigen::Isometry3d::Identity());
  ASSERT_TRUE(checker.addCollisionObject("octomap_link", 0, octree_shapes, octree_poses, true));

  CollisionShapesConst sphere_shapes;
  tesseract::common::VectorIsometry3d sphere_poses;
  sphere_shapes.push_back(std::make_shared<tesseract::geometry::Sphere>(0.25));
  sphere_poses.push_back(Eigen::Isometry3d::Identity());
  ASSERT_TRUE(checker.addCollisionObject("sphere_link", 0, sphere_shapes, sphere_poses, true));
}

inline void addContinuousOctreePair(ContinuousContactManager& checker, tesseract::geometry::OctreeSubType subtype)
{
  auto ot_a = loadOctreeBox2m();
  auto ot_b = loadOctreeBox2m();

  CollisionShapesConst octree1_shapes;
  tesseract::common::VectorIsometry3d octree1_poses;
  octree1_shapes.push_back(std::make_shared<tesseract::geometry::Octree>(ot_a, subtype));
  octree1_poses.emplace_back(Eigen::Translation3d(1.1, 0.0, 0.0));
  ASSERT_TRUE(checker.addCollisionObject("octomap1_link", 0, octree1_shapes, octree1_poses, true));

  CollisionShapesConst octree2_shapes;
  tesseract::common::VectorIsometry3d octree2_poses;
  octree2_shapes.push_back(std::make_shared<tesseract::geometry::Octree>(ot_b, subtype));
  octree2_poses.emplace_back(Eigen::Translation3d(-1.1, 0.0, 0.0));
  ASSERT_TRUE(checker.addCollisionObject("octomap2_link", 0, octree2_shapes, octree2_poses, true));
}
}  // namespace detail

inline void runDiscreteOctomapTransformOverloadUsageTest(DiscreteContactManager& checker,
                                                         tesseract::geometry::OctreeSubType subtype)
{
  detail::addDiscreteOctreeAndSphere(checker, subtype);

  checker.setActiveCollisionObjects(std::vector<std::string>{ "octomap_link", "sphere_link" });
  checker.setDefaultCollisionMargin(0.0);

  const Eigen::Isometry3d far_pose = Eigen::Isometry3d(Eigen::Translation3d(5.0, 0.0, 0.0));

  // Single-name overload
  checker.setCollisionObjectsTransform("octomap_link", far_pose);
  ContactResultVector far_single = detail::runClosest(checker);
  EXPECT_FALSE(detail::hasPair(far_single, "octomap_link", "sphere_link"));

  // Vector overload
  std::vector<std::string> names{ "octomap_link" };
  tesseract::common::VectorIsometry3d near_poses{ Eigen::Isometry3d::Identity() };
  checker.setCollisionObjectsTransform(names, near_poses);
  ContactResultVector near_vector = detail::runClosest(checker);
  EXPECT_TRUE(detail::hasPair(near_vector, "octomap_link", "sphere_link"));
  {
    const ContactResult* cr = detail::findPair(near_vector, "octomap_link", "sphere_link");
    ASSERT_NE(cr, nullptr);
    const std::size_t octree_idx = detail::getLinkIndex(*cr, "octomap_link");
    EXPECT_TRUE(cr->transform[octree_idx].isApprox(Eigen::Isometry3d::Identity(), 1e-9));
  }

  // Map overload
  tesseract::common::LinkIdTransformMap map_far;
  map_far[tesseract::common::LinkId("octomap_link")] = far_pose;
  checker.setCollisionObjectsTransform(map_far);
  ContactResultVector far_map = detail::runClosest(checker);
  EXPECT_FALSE(detail::hasPair(far_map, "octomap_link", "sphere_link"));
}

inline void runContinuousOctomapTransformOverloadUsageTest(ContinuousContactManager& checker,
                                                           tesseract::geometry::OctreeSubType subtype)
{
  detail::addContinuousOctreePair(checker, subtype);

  checker.setActiveCollisionObjects(std::vector<std::string>{ "octomap1_link" });
  checker.setCollisionMarginData(CollisionMarginData(0.25));

  const Eigen::Isometry3d static_far = Eigen::Isometry3d(Eigen::Translation3d(5.0, 0.0, 0.0));
  const Eigen::Isometry3d static_near = Eigen::Isometry3d::Identity();

  // Rigid single-name overload on static octree
  checker.setCollisionObjectsTransform("octomap2_link", static_far);

  // Sweep single-name overload on active octree
  const Eigen::Isometry3d start = Eigen::Isometry3d(Eigen::Translation3d(0.0, -2.0, 0.0));
  const Eigen::Isometry3d end = Eigen::Isometry3d(Eigen::Translation3d(0.0, 2.0, 0.0));
  checker.setCollisionObjectsTransform("octomap1_link", start, end);

  ContactResultVector single_sweep_far = detail::runClosest(checker);
  EXPECT_FALSE(detail::hasPair(single_sweep_far, "octomap1_link", "octomap2_link"));

  // Rigid vector overload on static octree
  std::vector<std::string> static_names{ "octomap2_link" };
  tesseract::common::VectorIsometry3d static_near_poses{ static_near };
  checker.setCollisionObjectsTransform(static_names, static_near_poses);

  // Sweep vector overload on active octree
  std::vector<std::string> active_names{ "octomap1_link" };
  tesseract::common::VectorIsometry3d starts{ start };
  tesseract::common::VectorIsometry3d ends{ end };
  checker.setCollisionObjectsTransform(active_names, starts, ends);

  ContactResultVector vector_sweep_near = detail::runClosest(checker);
  EXPECT_TRUE(detail::hasPair(vector_sweep_near, "octomap1_link", "octomap2_link"));
  {
    const ContactResult* cr = detail::findPair(vector_sweep_near, "octomap1_link", "octomap2_link");
    ASSERT_NE(cr, nullptr);
    const std::size_t moving_idx = detail::getLinkIndex(*cr, "octomap1_link");
    const std::size_t static_idx = detail::getLinkIndex(*cr, "octomap2_link");
    EXPECT_TRUE(cr->transform[moving_idx].isApprox(start, 1e-9));
    EXPECT_TRUE(cr->cc_transform[moving_idx].isApprox(end, 1e-9));
    EXPECT_NE(cr->cc_type[moving_idx], ContinuousCollisionType::CCType_None);
    EXPECT_TRUE(cr->transform[static_idx].isApprox(static_near, 1e-9));
    EXPECT_NEAR(cr->cc_time[static_idx], -1.0, 1e-9);
    EXPECT_EQ(cr->cc_type[static_idx], ContinuousCollisionType::CCType_None);
  }

  // Rigid map overload on static octree
  tesseract::common::LinkIdTransformMap static_map_far;
  static_map_far[tesseract::common::LinkId("octomap2_link")] = static_far;
  checker.setCollisionObjectsTransform(static_map_far);

  // Sweep map overload on active octree
  tesseract::common::LinkIdTransformMap map_start;
  tesseract::common::LinkIdTransformMap map_end;
  map_start[tesseract::common::LinkId("octomap1_link")] = start;
  map_end[tesseract::common::LinkId("octomap1_link")] = end;
  checker.setCollisionObjectsTransform(map_start, map_end);

  ContactResultVector map_sweep_far = detail::runClosest(checker);
  EXPECT_FALSE(detail::hasPair(map_sweep_far, "octomap1_link", "octomap2_link"));

  // End-to-end sanity check after overload updates.
  checker.setCollisionObjectsTransform("octomap2_link", static_near);
  checker.setCollisionObjectsTransform(map_start, map_end);
  ContactResultVector map_sweep_near = detail::runClosest(checker);
  EXPECT_TRUE(detail::hasPair(map_sweep_near, "octomap1_link", "octomap2_link"));
  {
    const ContactResult* cr = detail::findPair(map_sweep_near, "octomap1_link", "octomap2_link");
    ASSERT_NE(cr, nullptr);
    const std::size_t moving_idx = detail::getLinkIndex(*cr, "octomap1_link");
    const std::size_t static_idx = detail::getLinkIndex(*cr, "octomap2_link");
    EXPECT_TRUE(cr->transform[moving_idx].isApprox(start, 1e-9));
    EXPECT_TRUE(cr->cc_transform[moving_idx].isApprox(end, 1e-9));
    EXPECT_NE(cr->cc_type[moving_idx], ContinuousCollisionType::CCType_None);
    EXPECT_TRUE(cr->transform[static_idx].isApprox(static_near, 1e-9));
    EXPECT_NEAR(cr->cc_time[static_idx], -1.0, 1e-9);
    EXPECT_EQ(cr->cc_type[static_idx], ContinuousCollisionType::CCType_None);
  }
}

}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_OCTOMAP_USAGE_UNIT_HPP
