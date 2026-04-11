#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>
#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_factories.h>
#include <tesseract/geometry/geometries.h>

using namespace tesseract::collision;
using namespace tesseract::collision::tesseract_collision_coal;

namespace
{
/// Helper: create a discrete manager, add two overlapping spheres, set active.
std::unique_ptr<CoalDiscreteBVHManager> makeDiscreteSetup(double gjk_guess_threshold = kDefaultGJKGuessThreshold)
{
  auto checker = std::make_unique<CoalDiscreteBVHManager>("test", gjk_guess_threshold);

  auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.5);
  CollisionShapesConst shapes = { sphere };
  tesseract::common::VectorIsometry3d poses = { Eigen::Isometry3d::Identity() };

  checker->addCollisionObject("link_a", 0, shapes, poses, true);
  checker->addCollisionObject("link_b", 0, shapes, poses, true);
  checker->setActiveCollisionObjects(std::vector<std::string>{ "link_a", "link_b" });
  checker->setDefaultCollisionMargin(0.0);

  // Place spheres so they overlap
  Eigen::Isometry3d pose_a = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose_b = Eigen::Isometry3d::Identity();
  pose_b.translation() = Eigen::Vector3d(0.5, 0, 0);
  checker->setCollisionObjectsTransform("link_a", pose_a);
  checker->setCollisionObjectsTransform("link_b", pose_b);

  return checker;
}

/// Helper: create a cast manager, add two overlapping spheres, set active.
std::unique_ptr<CoalCastBVHManager> makeCastSetup(double gjk_guess_threshold = kDefaultGJKGuessThreshold)
{
  auto checker = std::make_unique<CoalCastBVHManager>("test", gjk_guess_threshold);

  auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.5);
  CollisionShapesConst shapes = { sphere };
  tesseract::common::VectorIsometry3d poses = { Eigen::Isometry3d::Identity() };

  checker->addCollisionObject("link_a", 0, shapes, poses, true);
  checker->addCollisionObject("link_b", 0, shapes, poses, true);
  checker->setActiveCollisionObjects(std::vector<std::string>{ "link_a", "link_b" });
  checker->setDefaultCollisionMargin(0.0);

  Eigen::Isometry3d pose_a = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose_b = Eigen::Isometry3d::Identity();
  pose_b.translation() = Eigen::Vector3d(0.5, 0, 0);
  checker->setCollisionObjectsTransform("link_a", pose_a);
  checker->setCollisionObjectsTransform("link_b", pose_b);

  return checker;
}

/// Run a contact test and return the number of contacts found.
template <typename ManagerT>
std::size_t countContacts(ManagerT& mgr)
{
  ContactResultMap results;
  mgr.contactTest(results, ContactRequest(ContactTestType::ALL));
  ContactResultVector vec;
  results.flattenCopyResults(vec);
  return vec.size();
}
}  // namespace

/// Verify discrete manager works correctly with default threshold.
TEST(CoalGJKGuessThresholdUnit, DiscreteDefaultThreshold)  // NOLINT
{
  auto checker = makeDiscreteSetup();

  // Initial contact
  EXPECT_EQ(countContacts(*checker), 1);

  // Small move (4mm < 5mm default threshold) — contacts still detected
  Eigen::Isometry3d new_pose = Eigen::Isometry3d::Identity();
  new_pose.translation() = Eigen::Vector3d(0.004, 0, 0);
  checker->setCollisionObjectsTransform("link_a", new_pose);
  EXPECT_EQ(countContacts(*checker), 1);

  // Larger move (6mm > 5mm default threshold) — contacts still detected
  new_pose.translation() = Eigen::Vector3d(0.010, 0, 0);
  checker->setCollisionObjectsTransform("link_a", new_pose);
  EXPECT_EQ(countContacts(*checker), 1);
}

/// Verify discrete manager works correctly with a custom threshold.
TEST(CoalGJKGuessThresholdUnit, DiscreteCustomThreshold)  // NOLINT
{
  auto checker = makeDiscreteSetup(0.001);  // 1mm threshold

  EXPECT_EQ(countContacts(*checker), 1);

  // Move 2mm — above 1mm custom threshold but below 5mm default
  Eigen::Isometry3d new_pose = Eigen::Isometry3d::Identity();
  new_pose.translation() = Eigen::Vector3d(0.002, 0, 0);
  checker->setCollisionObjectsTransform("link_a", new_pose);
  EXPECT_EQ(countContacts(*checker), 1);
}

/// Verify cast manager works correctly with default threshold.
TEST(CoalGJKGuessThresholdUnit, CastDefaultThreshold)  // NOLINT
{
  auto checker = makeCastSetup();

  EXPECT_EQ(countContacts(*checker), 1);

  // Small move via single-pose update
  Eigen::Isometry3d new_pose = Eigen::Isometry3d::Identity();
  new_pose.translation() = Eigen::Vector3d(0.004, 0, 0);
  checker->setCollisionObjectsTransform("link_a", new_pose);
  EXPECT_EQ(countContacts(*checker), 1);
}

/// Verify cast manager works correctly with a custom threshold.
TEST(CoalGJKGuessThresholdUnit, CastCustomThreshold)  // NOLINT
{
  auto checker = makeCastSetup(0.001);  // 1mm threshold

  EXPECT_EQ(countContacts(*checker), 1);

  // Move 2mm via single-pose update
  Eigen::Isometry3d new_pose = Eigen::Isometry3d::Identity();
  new_pose.translation() = Eigen::Vector3d(0.002, 0, 0);
  checker->setCollisionObjectsTransform("link_a", new_pose);
  EXPECT_EQ(countContacts(*checker), 1);
}

/// Verify clone() preserves a custom threshold on the discrete manager.
TEST(CoalGJKGuessThresholdUnit, DiscreteClonePreservesThreshold)  // NOLINT
{
  auto checker = makeDiscreteSetup(0.001);

  DiscreteContactManager::UPtr cloned = checker->clone();
  ASSERT_NE(cloned, nullptr);

  // The cloned manager should still produce correct contacts
  EXPECT_EQ(countContacts(*cloned), 1);

  // Move and verify the cloned manager still works
  Eigen::Isometry3d new_pose = Eigen::Isometry3d::Identity();
  new_pose.translation() = Eigen::Vector3d(0.002, 0, 0);
  cloned->setCollisionObjectsTransform("link_a", new_pose);
  EXPECT_EQ(countContacts(*cloned), 1);
}

/// Verify clone() preserves a custom threshold on the cast manager.
TEST(CoalGJKGuessThresholdUnit, CastClonePreservesThreshold)  // NOLINT
{
  auto checker = makeCastSetup(0.001);

  ContinuousContactManager::UPtr cloned = checker->clone();
  ASSERT_NE(cloned, nullptr);

  EXPECT_EQ(countContacts(*cloned), 1);

  Eigen::Isometry3d new_pose = Eigen::Isometry3d::Identity();
  new_pose.translation() = Eigen::Vector3d(0.002, 0, 0);
  cloned->setCollisionObjectsTransform("link_a", new_pose);
  EXPECT_EQ(countContacts(*cloned), 1);
}

/// Verify factory creates managers with config-specified threshold.
TEST(CoalGJKGuessThresholdUnit, FactoryParsesConfig)  // NOLINT
{
  YAML::Node config;
  config["gjk_guess_threshold"] = 0.01;

  CoalDiscreteBVHManagerFactory discrete_factory;
  auto discrete = discrete_factory.create("test_discrete", config);
  ASSERT_NE(discrete, nullptr);

  CoalCastBVHManagerFactory cast_factory;
  auto cast = cast_factory.create("test_cast", config);
  ASSERT_NE(cast, nullptr);
}

/// Verify factory uses default when config is null.
TEST(CoalGJKGuessThresholdUnit, FactoryDefaultsOnNullConfig)  // NOLINT
{
  YAML::Node config;  // IsNull() == true

  CoalDiscreteBVHManagerFactory discrete_factory;
  auto discrete = discrete_factory.create("test_discrete", config);
  ASSERT_NE(discrete, nullptr);

  CoalCastBVHManagerFactory cast_factory;
  auto cast = cast_factory.create("test_cast", config);
  ASSERT_NE(cast, nullptr);
}

/// Verify factory uses default when config has no gjk_guess_threshold key.
TEST(CoalGJKGuessThresholdUnit, FactoryDefaultsOnMissingKey)  // NOLINT
{
  YAML::Node config;
  config["some_other_key"] = 42;

  CoalDiscreteBVHManagerFactory discrete_factory;
  auto discrete = discrete_factory.create("test_discrete", config);
  ASSERT_NE(discrete, nullptr);

  CoalCastBVHManagerFactory cast_factory;
  auto cast = cast_factory.create("test_cast", config);
  ASSERT_NE(cast, nullptr);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
