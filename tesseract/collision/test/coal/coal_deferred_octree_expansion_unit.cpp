#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <octomap/octomap.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/collision/common.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>

using namespace tesseract::collision;
using namespace tesseract_collision_coal;

/**
 * @brief Verify deferred octree voxel expansion in CoalCastBVHManager.
 *
 * Static octrees should keep raw OcTree geometry in their cast COW (no CastHullShape wrapping).
 * Expansion to per-voxel CastHullShape boxes happens only on promotion to active (kinematic).
 * Once expanded, demotion/re-promotion reuses the cached expansion.
 */
class DeferredOctreeExpansionUnit : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Add static octree (2m box at origin, res 0.5m)
    tesseract::common::GeneralResourceLocator locator;
    std::string path = locator.locateResource("package://tesseract/support/meshes/box_2m.bt")->getFilePath();
    auto ot = std::make_shared<octomap::OcTree>(path);
    CollisionShapePtr dense_octomap =
        std::make_shared<tesseract::geometry::Octree>(ot, tesseract::geometry::OctreeSubType::BOX);

    CollisionShapesConst octree_shapes;
    tesseract::common::VectorIsometry3d octree_poses;
    octree_shapes.push_back(dense_octomap);
    octree_poses.push_back(Eigen::Isometry3d::Identity());
    checker_.addCollisionObject("octree_link", 0, octree_shapes, octree_poses);

    // Add a cylinder for sweep tests
    CollisionShapePtr cyl = std::make_shared<tesseract::geometry::Cylinder>(0.1, 0.4);
    CollisionShapesConst cyl_shapes;
    tesseract::common::VectorIsometry3d cyl_poses;
    cyl_shapes.push_back(cyl);
    cyl_poses.push_back(Eigen::Isometry3d::Identity());
    checker_.addCollisionObject("cyl_link", 0, cyl_shapes, cyl_poses);

    checker_.setDefaultCollisionMargin(0.0);
  }

  CoalCastBVHManager checker_;
};

TEST_F(DeferredOctreeExpansionUnit, StaticOctreeNotExpandedOnAdd)  // NOLINT
{
  // Both objects start static — octree cast COW should contain raw OcTree (deferred).
  const auto& cast_map = checker_.getCastCollisionObjectMap();
  auto it = cast_map.find("octree_link");
  ASSERT_NE(it, cast_map.end());
  EXPECT_TRUE(castCowNeedsOctreeExpansion(it->second));
}

TEST_F(DeferredOctreeExpansionUnit, PromotionExpandsOctree)  // NOLINT
{
  checker_.setActiveCollisionObjects({ "octree_link", "cyl_link" });

  const auto& cast_map = checker_.getCastCollisionObjectMap();
  auto it = cast_map.find("octree_link");
  ASSERT_NE(it, cast_map.end());
  EXPECT_FALSE(castCowNeedsOctreeExpansion(it->second));

  // All collision objects in the expanded cast COW should be CastHullShape (GEOM_CUSTOM).
  for (const auto& co : it->second->getCollisionObjects())
    EXPECT_EQ(co->collisionGeometry()->getNodeType(), coal::GEOM_CUSTOM);
}

TEST_F(DeferredOctreeExpansionUnit, PromotedOctreeProducesContacts)  // NOLINT
{
  checker_.setActiveCollisionObjects({ "octree_link", "cyl_link" });

  // Sweep cylinder through the octree
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(2.0, 0, 0);
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0.5, 0, 0);

  checker_.setCollisionObjectsTransform("cyl_link", start, end);
  checker_.setCollisionObjectsTransform("octree_link", Eigen::Isometry3d::Identity(),
                                        Eigen::Isometry3d::Identity());

  ContactResultMap result;
  ContactRequest request(ContactTestType::ALL);
  checker_.contactTest(result, request);
  EXPECT_FALSE(result.empty());
}

TEST_F(DeferredOctreeExpansionUnit, DemotionPreservesExpansion)  // NOLINT
{
  // Promote, then demote
  checker_.setActiveCollisionObjects({ "octree_link", "cyl_link" });
  checker_.setActiveCollisionObjects({});

  // Expanded cast COW should be cached — not reverted to raw OcTree.
  const auto& cast_map = checker_.getCastCollisionObjectMap();
  auto it = cast_map.find("octree_link");
  ASSERT_NE(it, cast_map.end());
  EXPECT_FALSE(castCowNeedsOctreeExpansion(it->second));
}

TEST_F(DeferredOctreeExpansionUnit, RePromotionSkipsReExpansion)  // NOLINT
{
  // Promote -> demote -> re-promote
  checker_.setActiveCollisionObjects({ "octree_link", "cyl_link" });
  checker_.setActiveCollisionObjects({});
  checker_.setActiveCollisionObjects({ "octree_link", "cyl_link" });

  const auto& cast_map = checker_.getCastCollisionObjectMap();
  auto it = cast_map.find("octree_link");
  ASSERT_NE(it, cast_map.end());
  EXPECT_FALSE(castCowNeedsOctreeExpansion(it->second));

  // Verify contacts still work after re-promotion
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(2.0, 0, 0);
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0.5, 0, 0);

  checker_.setCollisionObjectsTransform("cyl_link", start, end);
  checker_.setCollisionObjectsTransform("octree_link", Eigen::Isometry3d::Identity(),
                                        Eigen::Isometry3d::Identity());

  ContactResultMap result;
  ContactRequest request(ContactTestType::ALL);
  checker_.contactTest(result, request);
  EXPECT_FALSE(result.empty());
}

TEST_F(DeferredOctreeExpansionUnit, DualPoseTransformOnStaticOctreeIsHarmless)  // NOLINT
{
  // Octree is static. Calling dual-pose setCollisionObjectsTransform should not crash
  // (exercises the StaticFilter guard in collectCastTransformUpdate).
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() = Eigen::Vector3d(1, 0, 0);
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(2, 0, 0);

  EXPECT_NO_THROW(checker_.setCollisionObjectsTransform("octree_link", pose1, pose2));

  // Promote cylinder, sweep it, verify the static octree still produces contacts.
  checker_.setActiveCollisionObjects({ "cyl_link" });

  Eigen::Isometry3d cyl_start = Eigen::Isometry3d::Identity();
  cyl_start.translation() = Eigen::Vector3d(2.0, 0, 0);
  Eigen::Isometry3d cyl_end = Eigen::Isometry3d::Identity();
  cyl_end.translation() = Eigen::Vector3d(0.5, 0, 0);

  checker_.setCollisionObjectsTransform("cyl_link", cyl_start, cyl_end);

  ContactResultMap result;
  ContactRequest request(ContactTestType::ALL);
  checker_.contactTest(result, request);
  EXPECT_FALSE(result.empty());
}

TEST_F(DeferredOctreeExpansionUnit, CloneWithActiveOctreeExpands)  // NOLINT
{
  // Promote octree, then clone. The clone's addCollisionObject kinematic branch
  // should expand the deferred octree.
  checker_.setActiveCollisionObjects({ "octree_link", "cyl_link" });

  auto clone = checker_.clone();
  auto* cast_clone = dynamic_cast<CoalCastBVHManager*>(clone.get());
  ASSERT_NE(cast_clone, nullptr);

  const auto& cast_map = cast_clone->getCastCollisionObjectMap();
  auto it = cast_map.find("octree_link");
  ASSERT_NE(it, cast_map.end());
  EXPECT_FALSE(castCowNeedsOctreeExpansion(it->second));

  // Verify contacts work on the clone
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(2.0, 0, 0);
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0.5, 0, 0);

  cast_clone->setCollisionObjectsTransform("cyl_link", start, end);
  cast_clone->setCollisionObjectsTransform("octree_link", Eigen::Isometry3d::Identity(),
                                           Eigen::Isometry3d::Identity());

  ContactResultMap result;
  ContactRequest request(ContactTestType::ALL);
  cast_clone->contactTest(result, request);
  EXPECT_FALSE(result.empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
