#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_casthullshape.h>
#include <tesseract/collision/coal/coal_factories.h>
#include <tesseract/geometry/geometries.h>

using namespace tesseract::collision;
using namespace tesseract::collision::tesseract_collision_coal;

namespace
{
/// Helper: access the CastHullShape for a given link from the cast manager.
CastHullShape* getCastHullShape(CoalCastBVHManager& mgr, const std::string& link_name)
{
  const auto& cast_map = mgr.getCastCollisionObjectMap();
  auto it = cast_map.find(tesseract::common::LinkId::fromName(link_name));
  if (it == cast_map.end())
    return nullptr;
  auto& cos = it->second->getCollisionObjects();
  if (cos.empty())
    return nullptr;
  return static_cast<CastHullShape*>(cos[0]->collisionGeometry().get());
}

/// Helper: create a cast manager with a single sphere link.
std::unique_ptr<CoalCastBVHManager> makeCastSetup(bool d_arc_compensation)
{
  auto checker = std::make_unique<CoalCastBVHManager>("test", kDefaultGJKGuessThreshold, d_arc_compensation);

  auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.1);
  CollisionShapesConst shapes = { sphere };
  tesseract::common::VectorIsometry3d poses = { Eigen::Isometry3d::Identity() };

  checker->addCollisionObject("link_a", 0, shapes, poses, true);
  checker->setActiveCollisionObjects(std::vector<std::string>{ "link_a" });
  checker->setDefaultCollisionMargin(0.0);

  return checker;
}

/// Helper: run a contact test and return the result vector.
ContactResultVector runContactTest(ContinuousContactManager& mgr)
{
  ContactResultMap results;
  mgr.contactTest(results, ContactRequest(ContactTestType::ALL));
  ContactResultVector vec;
  results.flattenCopyResults(vec);
  return vec;
}
}  // namespace

/// Verify that with d_arc_compensation disabled (default), SSR stays zero after rotational motion.
TEST(CoalDArcCompensationUnit, DisabledKeepsZeroSSR)  // NOLINT
{
  auto checker = makeCastSetup(false);

  // Apply a 45-degree rotation about Z
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.rotate(Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()));
  checker->setCollisionObjectsTransform("link_a", pose1, pose2);

  auto* cast_shape = getCastHullShape(*checker, "link_a");
  ASSERT_NE(cast_shape, nullptr);
  EXPECT_DOUBLE_EQ(cast_shape->getSweptSphereRadius(), 0.0);
}

/// Verify that pure translation produces zero d_arc even with compensation enabled.
TEST(CoalDArcCompensationUnit, PureTranslationZeroDArc)  // NOLINT
{
  auto checker = makeCastSetup(true);

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);
  checker->setCollisionObjectsTransform("link_a", pose1, pose2);

  auto* cast_shape = getCastHullShape(*checker, "link_a");
  ASSERT_NE(cast_shape, nullptr);
  EXPECT_DOUBLE_EQ(cast_shape->getSweptSphereRadius(), 0.0);
}

/// Verify that a known rotation produces the expected d_arc value.
TEST(CoalDArcCompensationUnit, KnownRotationCorrectDArc)  // NOLINT
{
  auto checker = makeCastSetup(true);

  // 45-degree rotation about Z, shape centered at origin.
  // For a sphere of radius 0.1 centered at origin with rotation about Z through the origin:
  //   screw axis passes through origin (pure rotation, no translation)
  //   r_max = dist_center_to_axis + aabb_radius = 0 + aabb_radius
  //   d_arc = aabb_radius * (1 - cos(22.5°))
  const double phi = M_PI / 4.0;
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.rotate(Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()));
  checker->setCollisionObjectsTransform("link_a", pose1, pose2);

  auto* cast_shape = getCastHullShape(*checker, "link_a");
  ASSERT_NE(cast_shape, nullptr);

  // The underlying shape's aabb_radius is the circumradius of its local AABB.
  // For a Sphere(0.1), Coal computes aabb_local as a cube of half-extent 0.1,
  // so aabb_radius = sqrt(3) * 0.1.
  const double aabb_radius = cast_shape->getUnderlyingShape()->aabb_radius;
  EXPECT_GT(aabb_radius, 0.0);

  const double expected_d_arc = aabb_radius * (1.0 - std::cos(phi / 2.0));
  EXPECT_NEAR(cast_shape->getSweptSphereRadius(), expected_d_arc, 1e-12);
}

/// Verify d_arc with shape offset from rotation axis (nonzero screw axis distance).
TEST(CoalDArcCompensationUnit, OffsetShapeCorrectDArc)  // NOLINT
{
  auto checker = std::make_unique<CoalCastBVHManager>("test", kDefaultGJKGuessThreshold, true);

  // Place a small sphere at local offset (0.5, 0, 0) from the link frame.
  auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.05);
  CollisionShapesConst shapes = { sphere };
  Eigen::Isometry3d shape_offset = Eigen::Isometry3d::Identity();
  shape_offset.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);
  tesseract::common::VectorIsometry3d poses = { shape_offset };

  checker->addCollisionObject("link_a", 0, shapes, poses, true);
  checker->setActiveCollisionObjects(std::vector<std::string>{ "link_a" });
  checker->setDefaultCollisionMargin(0.0);

  // Rotate 30 degrees about Z at the link frame origin.
  // The shape is at (0.5, 0, 0), so it sweeps an arc at radius ~0.5 from the Z axis.
  const double phi = M_PI / 6.0;  // 30 degrees
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.rotate(Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()));
  checker->setCollisionObjectsTransform("link_a", pose1, pose2);

  auto* cast_shape = getCastHullShape(*checker, "link_a");
  ASSERT_NE(cast_shape, nullptr);

  const double ssr = cast_shape->getSweptSphereRadius();
  EXPECT_GT(ssr, 0.0);

  // The d_arc should be significantly larger than for a shape at the origin,
  // because the shape is 0.5m from the rotation axis.
  // r_max ≈ 0.5 + aabb_radius, d_arc = r_max * (1 - cos(15°)) ≈ 0.5 * 0.0341 ≈ 17mm
  const double aabb_radius = cast_shape->getUnderlyingShape()->aabb_radius;
  const double cos_half = std::cos(phi / 2.0);
  // The exact r_max depends on the screw axis computation, but we can bound it:
  // the shape center in the shape's own local frame is at aabb_center (should be origin for sphere).
  // In the link frame it's at (0.5, 0, 0). The castTransform_ encodes the relative motion
  // in the shape's local frame, so the screw axis position is transformed accordingly.
  // Rather than replicating the full formula, just verify the SSR is in the right ballpark.
  const double d_arc_lower = 0.5 * (1.0 - cos_half);                         // r = 0.5 (ignoring aabb_radius)
  const double d_arc_upper = (0.5 + aabb_radius + 0.01) * (1.0 - cos_half);  // generous upper bound
  EXPECT_GT(ssr, d_arc_lower * 0.9);
  EXPECT_LT(ssr, d_arc_upper * 1.1);
}

/// Verify clone() preserves the d_arc_compensation flag.
TEST(CoalDArcCompensationUnit, ClonePreservesFlag)  // NOLINT
{
  auto checker = makeCastSetup(true);

  auto cloned_base = checker->clone();
  ASSERT_NE(cloned_base, nullptr);
  auto* cloned = dynamic_cast<CoalCastBVHManager*>(cloned_base.get());
  ASSERT_NE(cloned, nullptr);

  // Apply rotation to the clone and verify SSR is nonzero (compensation active).
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.rotate(Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()));
  cloned->setCollisionObjectsTransform("link_a", pose1, pose2);

  auto* cast_shape = getCastHullShape(*cloned, "link_a");
  ASSERT_NE(cast_shape, nullptr);
  EXPECT_GT(cast_shape->getSweptSphereRadius(), 0.0);
}

/// Verify factory parses d_arc_compensation from YAML.
TEST(CoalDArcCompensationUnit, FactoryParsesConfig)  // NOLINT
{
  YAML::Node config;
  config["d_arc_compensation"] = true;

  CoalCastBVHManagerFactory cast_factory;
  auto cast = cast_factory.create("test_cast", config);
  ASSERT_NE(cast, nullptr);

  // Add a shape, apply rotation, verify SSR is nonzero (compensation was enabled).
  auto* mgr = dynamic_cast<CoalCastBVHManager*>(cast.get());
  ASSERT_NE(mgr, nullptr);

  auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.1);
  CollisionShapesConst shapes = { sphere };
  tesseract::common::VectorIsometry3d poses = { Eigen::Isometry3d::Identity() };
  mgr->addCollisionObject("link_a", 0, shapes, poses, true);
  mgr->setActiveCollisionObjects(std::vector<std::string>{ "link_a" });

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.rotate(Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()));
  mgr->setCollisionObjectsTransform("link_a", pose1, pose2);

  auto* cast_shape = getCastHullShape(*mgr, "link_a");
  ASSERT_NE(cast_shape, nullptr);
  EXPECT_GT(cast_shape->getSweptSphereRadius(), 0.0);
}

/// Verify factory defaults to disabled when config is null or key is missing.
TEST(CoalDArcCompensationUnit, FactoryDefaultsToDisabled)  // NOLINT
{
  // Null config
  {
    YAML::Node config;
    CoalCastBVHManagerFactory factory;
    auto mgr_base = factory.create("test", config);
    auto* mgr = dynamic_cast<CoalCastBVHManager*>(mgr_base.get());
    ASSERT_NE(mgr, nullptr);

    auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.1);
    CollisionShapesConst shapes = { sphere };
    tesseract::common::VectorIsometry3d poses = { Eigen::Isometry3d::Identity() };
    mgr->addCollisionObject("link_a", 0, shapes, poses, true);
    mgr->setActiveCollisionObjects(std::vector<std::string>{ "link_a" });

    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
    pose2.rotate(Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()));
    mgr->setCollisionObjectsTransform("link_a", pose1, pose2);

    auto* cast_shape = getCastHullShape(*mgr, "link_a");
    ASSERT_NE(cast_shape, nullptr);
    EXPECT_DOUBLE_EQ(cast_shape->getSweptSphereRadius(), 0.0);
  }

  // Missing key
  {
    YAML::Node config;
    config["some_other_key"] = 42;
    CoalCastBVHManagerFactory factory;
    auto mgr_base = factory.create("test", config);
    auto* mgr = dynamic_cast<CoalCastBVHManager*>(mgr_base.get());
    ASSERT_NE(mgr, nullptr);

    auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.1);
    CollisionShapesConst shapes = { sphere };
    tesseract::common::VectorIsometry3d poses = { Eigen::Isometry3d::Identity() };
    mgr->addCollisionObject("link_a", 0, shapes, poses, true);
    mgr->setActiveCollisionObjects(std::vector<std::string>{ "link_a" });

    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
    pose2.rotate(Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()));
    mgr->setCollisionObjectsTransform("link_a", pose1, pose2);

    auto* cast_shape = getCastHullShape(*mgr, "link_a");
    ASSERT_NE(cast_shape, nullptr);
    EXPECT_DOUBLE_EQ(cast_shape->getSweptSphereRadius(), 0.0);
  }
}

/// Key correctness test: verify that d_arc compensation catches a collision
/// that the uncompensated convex hull misses.
///
/// Setup: a small sphere on link_a rotates 90° about Z at radius 0.5m from the axis.
/// A static obstacle sphere is placed on the arc path (at the arc midpoint) but NOT
/// on the chord. Without d_arc compensation, the convex hull check misses it.
/// With compensation, the inflated distance detects the contact.
TEST(CoalDArcCompensationUnit, MissedCollisionCaught)  // NOLINT
{
  const double phi = M_PI / 2.0;  // 90 degrees
  const double r = 0.5;           // distance from rotation axis to shape center
  const double shape_radius = 0.02;
  const double obstacle_radius = 0.02;

  // Arc midpoint: (r*cos(45°), r*sin(45°), 0)
  const double arc_mid_x = r * std::cos(phi / 2.0);
  const double arc_mid_y = r * std::sin(phi / 2.0);
  // Chord midpoint: ((r + r*cos(90°))/2, (0 + r*sin(90°))/2, 0) = (r/2, r/2, 0)
  // Distance from chord midpoint to origin: r/sqrt(2) ≈ 0.354
  // Distance from arc midpoint to origin: r = 0.5
  // Gap (sagitta): r - r*cos(45°) = 0.5 * (1 - cos(45°)) ≈ 0.146m — well above obstacle size

  // Place obstacle between the arc and the chord, closer to the arc.
  // The obstacle center is placed slightly inside the arc (at distance r - obstacle_radius from
  // the origin along the arc midpoint direction), so the arc truly intersects the obstacle.
  const double obstacle_dist = r - (obstacle_radius * 0.5);
  const Eigen::Vector3d arc_dir(arc_mid_x, arc_mid_y, 0.0);
  const Eigen::Vector3d obstacle_pos = arc_dir.normalized() * obstacle_dist;

  auto run_test = [&](bool compensation) -> bool {
    auto checker = std::make_unique<CoalCastBVHManager>("test", kDefaultGJKGuessThreshold, compensation);

    // Moving shape: small sphere at local offset (0.5, 0, 0) from link frame.
    auto sphere = std::make_shared<tesseract::geometry::Sphere>(shape_radius);
    CollisionShapesConst shapes = { sphere };
    Eigen::Isometry3d shape_offset = Eigen::Isometry3d::Identity();
    shape_offset.translation() = Eigen::Vector3d(r, 0.0, 0.0);
    tesseract::common::VectorIsometry3d shape_poses = { shape_offset };

    checker->addCollisionObject("link_a", 0, shapes, shape_poses, true);

    // Static obstacle
    auto obs_sphere = std::make_shared<tesseract::geometry::Sphere>(obstacle_radius);
    CollisionShapesConst obs_shapes = { obs_sphere };
    Eigen::Isometry3d obs_pose = Eigen::Isometry3d::Identity();
    obs_pose.translation() = obstacle_pos;
    tesseract::common::VectorIsometry3d obs_poses = { Eigen::Isometry3d::Identity() };

    checker->addCollisionObject("obstacle", 0, obs_shapes, obs_poses, true);
    checker->setActiveCollisionObjects(std::vector<std::string>{ "link_a", "obstacle" });

    // Place obstacle at its world position
    Eigen::Isometry3d world_obs_pose = Eigen::Isometry3d::Identity();
    world_obs_pose.translation() = obstacle_pos;
    checker->setCollisionObjectsTransform("obstacle", world_obs_pose);

    // Set the collision margin to include distance queries
    checker->setDefaultCollisionMargin(shape_radius + obstacle_radius + 0.01);

    // Link_a sweeps from identity (shape at (0.5,0,0)) to 90° rotation (shape at (0,0.5,0))
    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
    pose2.rotate(Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()));
    checker->setCollisionObjectsTransform("link_a", pose1, pose2);

    auto contacts = runContactTest(*checker);
    return !contacts.empty();
  };

  // Without compensation: the convex hull chord misses the obstacle on the arc
  EXPECT_FALSE(run_test(false)) << "Uncompensated check should miss the collision on the arc";

  // With compensation: the inflated distance catches it
  EXPECT_TRUE(run_test(true)) << "Compensated check should detect the collision on the arc";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
