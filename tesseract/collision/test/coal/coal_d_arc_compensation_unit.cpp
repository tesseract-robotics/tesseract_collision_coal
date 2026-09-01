#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_casthullshape.h>
#include <tesseract/collision/coal/coal_factories.h>
#include <tesseract/collision/test_suite/collision_cast_scenario_unit.hpp>
#include <tesseract/geometry/geometries.h>

using namespace tesseract::collision;
using namespace tesseract::collision::tesseract_collision_coal;

namespace
{
/// Helper: access the CastHullShape for a given link from the cast manager.
CastHullShape* getCastHullShape(CoalCastBVHManager& mgr, const std::string& link_name)
{
  const auto& cast_map = mgr.getCastCollisionObjectMap();
  auto it = cast_map.find(link_name);
  if (it == cast_map.end())
    return nullptr;
  auto& cos = it->second->getCollisionObjects();
  if (cos.empty())
    return nullptr;
  return static_cast<CastHullShape*>(cos[0]->collisionGeometryPtr());
}

/// Helper: create a cast manager with a single link, "link_a". Defaults to a Sphere(0.1) at the
/// link origin; pass `shape` and/or `shape_pose` to place a different shape, or the same shape at a
/// non-identity shape-local offset, instead.
std::unique_ptr<CoalCastBVHManager> makeCastSetup(bool d_arc_compensation,
                                                  const CollisionShapeConstPtr& shape = nullptr,
                                                  const Eigen::Isometry3d& shape_pose = Eigen::Isometry3d::Identity())
{
  auto checker = std::make_unique<CoalCastBVHManager>("test", d_arc_compensation);

  CollisionShapesConst shapes = { shape != nullptr ? shape : std::make_shared<tesseract::geometry::Sphere>(0.1) };
  tesseract::common::VectorIsometry3d poses = { shape_pose };

  checker->addCollisionObject("link_a", 0, shapes, poses, true);
  checker->setActiveCollisionObjects({ "link_a" });
  checker->setDefaultCollisionMargin(0.0);

  return checker;
}

/// Independent oracle for d_arc: the screw-axis construction evaluated from Eigen's own axis-angle
/// decomposition. Shares no code with computeDArc, so it can see the screw-axis half of the
/// formula that the implementation's own expression cannot.
///
/// `shape_local_offset` is the shape's pose within the link; the relative motion seen by the shape
/// is the link's relative motion conjugated into that frame. `aabb_center` and `aabb_radius` are the
/// underlying shape's, in its own local frame -- computeDArc is handed
/// `*cast_shape->getUnderlyingShape()`, not the cast hull, and reads both off it. They are taken as
/// parameters rather than assumed: a sphere's aabb_center is its own origin, and hardcoding that
/// would make this oracle silently wrong for any other geometry.
double referenceDArc(const Eigen::Isometry3d& pose1,
                     const Eigen::Isometry3d& pose2,
                     const Eigen::Vector3d& shape_local_offset,
                     const Eigen::Vector3d& aabb_center,
                     double aabb_radius)
{
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = shape_local_offset;
  const Eigen::Isometry3d rel = offset.inverse() * (pose1.inverse() * pose2) * offset;

  const Eigen::AngleAxisd aa(rel.linear());
  const double phi = aa.angle();
  // Same small-rotation guard as computeDArcScalars' `one_plus_cos > 2.0 - 1e-14`, restated on
  // cos(phi) rather than 1 + cos(phi): `1 + cos_phi > 2 - 1e-14` is `cos_phi > 1 - 1e-14`. Written in
  // the implementation's own units rather than re-derived on phi, so the one threshold the two sides
  // must agree on cannot drift out of step if it ever moves.
  if (std::cos(phi) > 1.0 - 1e-14)
    return 0.0;

  const Eigen::Vector3d& k = aa.axis();
  const Eigen::Vector3d t = rel.translation();
  const Eigen::Vector3d t_perp = t - t.dot(k) * k;
  // At phi = pi the axis sign is ambiguous, and cot(phi/2) is zero there, so the term it multiplies
  // vanishes with it -- both signs give the same answer.
  const Eigen::Vector3d c = 0.5 * t_perp + 0.5 * (std::cos(phi / 2.0) / std::sin(phi / 2.0)) * k.cross(t_perp);

  const Eigen::Vector3d pc = aabb_center - c;
  return ((pc - pc.dot(k) * k).norm() + aabb_radius) * (1.0 - std::cos(phi / 2.0));
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

  // Sphere::computeLocalAABB ends `aabb_radius = radius`, so for a Sphere(0.1) this is 0.1 -- not
  // the circumradius of the local AABB, which is a cube of half-extent 0.1. Assert the documented
  // value rather than only reading it back, so the number and the comment cannot drift apart.
  const auto& underlying = cast_shape->getUnderlyingShape();
  const double aabb_radius = underlying->aabb_radius;
  EXPECT_NEAR(aabb_radius, 0.1, 1e-15);

  const double expected = referenceDArc(pose1, pose2, Eigen::Vector3d::Zero(), underlying->aabb_center, aabb_radius);
  // Relative, not absolute: the oracle reaches the same value by a different route (AngleAxisd
  // rather than half-angle identities on the trace), so the two agree to rounding, not to the last
  // bit. 1e-12 absolute on a value of order 0.02 is a tighter claim than that supports.
  EXPECT_NEAR(cast_shape->getSweptSphereRadius(), expected, 1e-10 * std::max(1.0, std::abs(expected)));
}

/// Verify d_arc with shape offset from rotation axis (nonzero screw axis distance).
TEST(CoalDArcCompensationUnit, OffsetShapeCorrectDArc)  // NOLINT
{
  // Place a small sphere at local offset (0.5, 0, 0) from the link frame.
  auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.05);
  Eigen::Isometry3d shape_offset = Eigen::Isometry3d::Identity();
  shape_offset.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);
  auto checker = makeCastSetup(true, sphere, shape_offset);

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

  // This is the only test whose shape sits off the rotation axis, so it is the only one that
  // exercises the screw axis, the axis point c, and r_max = dist_to_axis + aabb_radius. Assert the
  // exact value: a band wide enough to pass regardless of those three is not an oracle for them.
  const auto& underlying = cast_shape->getUnderlyingShape();
  const double expected =
      referenceDArc(pose1, pose2, shape_offset.translation(), underlying->aabb_center, underlying->aabb_radius);
  EXPECT_NEAR(ssr, expected, 1e-10 * std::max(1.0, std::abs(expected)));
}

/// Verify the single-pose setter clears a swept-sphere radius the previous sweep left behind.
TEST(CoalDArcCompensationUnit, SinglePoseClearsDArcRadius)  // NOLINT
{
  auto checker = makeCastSetup(true);

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.rotate(Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()));
  checker->setCollisionObjectsTransform("link_a", pose1, pose2);

  auto* cast_shape = getCastHullShape(*checker, "link_a");
  ASSERT_NE(cast_shape, nullptr);
  ASSERT_GT(cast_shape->getSweptSphereRadius(), 0.0);

  // A pose without a sweep is a zero-length sweep, whose arc sagitta is zero.
  Eigen::Isometry3d pose3 = Eigen::Isometry3d::Identity();
  pose3.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
  checker->setCollisionObjectsTransform("link_a", pose3);

  EXPECT_DOUBLE_EQ(cast_shape->getSweptSphereRadius(), 0.0);
}

/// Verify promotion clears a swept-sphere radius carried from before the link went static.
TEST(CoalDArcCompensationUnit, PromotionClearsDArcRadius)  // NOLINT
{
  auto checker = makeCastSetup(true);

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.rotate(Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()));
  checker->setCollisionObjectsTransform("link_a", pose1, pose2);

  ASSERT_NE(getCastHullShape(*checker, "link_a"), nullptr);
  ASSERT_GT(getCastHullShape(*checker, "link_a")->getSweptSphereRadius(), 0.0);

  // Demote by naming a different link: an empty active set means every link is active, not none.
  // No setter runs between the two calls, so the radius survives unless the swap itself clears it.
  auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.1);
  checker->addCollisionObject("link_b", 0, { sphere }, { Eigen::Isometry3d::Identity() }, true);
  checker->setActiveCollisionObjects({ "link_b" });
  checker->setActiveCollisionObjects({ "link_a" });

  auto* cast_shape = getCastHullShape(*checker, "link_a");
  ASSERT_NE(cast_shape, nullptr);
  EXPECT_DOUBLE_EQ(cast_shape->getSweptSphereRadius(), 0.0);
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
  mgr->setActiveCollisionObjects({ "link_a" });

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
    mgr->setActiveCollisionObjects({ "link_a" });

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
    mgr->setActiveCollisionObjects({ "link_a" });

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
    auto checker = std::make_unique<CoalCastBVHManager>("test", compensation);

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
    // The shape's pose within the link is deliberately identity: the obstacle is placed by the
    // setCollisionObjectsTransform call below. Passing obstacle_pos here as well would apply the
    // offset twice and move the obstacle off the arc, which is the premise this test rests on.
    tesseract::common::VectorIsometry3d obs_poses = { Eigen::Isometry3d::Identity() };

    checker->addCollisionObject("obstacle", 0, obs_shapes, obs_poses, true);
    checker->setActiveCollisionObjects({ "link_a", "obstacle" });

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

/// A relative rotation of pi must leave the swept-sphere radius finite and on the oracle. Nothing
/// downstream can reject a bad one: coal's only validation is `radius < 0`, and NaN < 0 is false, so
/// a non-finite radius reaches the reported contact distance as an arbitrary zero rather than a
/// dropped contact. A half turn is the hardest case for any axis extraction -- the skew-symmetric
/// part of R vanishes there and the axis sign becomes ambiguous -- so it is pinned directly.
TEST(CoalDArcCompensationUnit, HalfTurnProducesFiniteDArc)  // NOLINT
{
  const double shape_radius = 0.05;
  const Eigen::Vector3d offset(0.5, 0.0, 0.0);
  const std::vector<Eigen::Vector3d> axes = {
    Eigen::Vector3d::UnitX(),        Eigen::Vector3d::UnitY(),       Eigen::Vector3d::UnitZ(),
    Eigen::Vector3d(0.3, -0.7, 1.0), Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(1.0, 2.0, 3.0),
  };
  // The exact singularity is not the whole requirement: the trace rounds to exactly -1 -- an
  // indistinguishable half turn -- for every phi within ~1.5e-8 of pi, once (pi - phi)^2 falls under
  // the double epsilon. 1e-8 is inside that band and 1e-7 is outside it, so the pair brackets it.
  const std::vector<double> phis = { M_PI, M_PI - 1e-8, M_PI - 1e-7 };

  for (const auto& axis : axes)
  {
    for (double phi : phis)
    {
      Eigen::Isometry3d shape_offset = Eigen::Isometry3d::Identity();
      shape_offset.translation() = offset;
      auto checker = makeCastSetup(true, std::make_shared<tesseract::geometry::Sphere>(shape_radius), shape_offset);

      Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
      pose2.rotate(Eigen::AngleAxisd(phi, axis.normalized()));
      checker->setCollisionObjectsTransform("link_a", pose1, pose2);

      auto* cast_shape = getCastHullShape(*checker, "link_a");
      ASSERT_NE(cast_shape, nullptr);
      const double ssr = cast_shape->getSweptSphereRadius();

      // The deficit rather than phi itself: all three angles round to the same six decimals, so the
      // trace is the only thing that says which of the 18 rows a failure came from.
      std::ostringstream row;
      row << "axis=" << axis.x() << "," << axis.y() << "," << axis.z() << " pi-phi=" << std::scientific << (M_PI - phi);
      SCOPED_TRACE(row.str());
      EXPECT_TRUE(std::isfinite(ssr));
      EXPECT_GE(ssr, 0.0);
      const auto& underlying = cast_shape->getUnderlyingShape();
      // 1e-7, not the 1e-10 the tests away from pi use, and not because the fix is approximate:
      // within 1e-3 of a half turn neither this value nor the oracle's recovers phi to better than
      // ~1e-8, so they disagree by ~5e-9 with neither side wrong. Tightening this does not catch a
      // defect, it manufactures a failure.
      EXPECT_NEAR(ssr,
                  referenceDArc(pose1, pose2, offset, underlying->aabb_center, underlying->aabb_radius),
                  1e-7 * std::max(1.0, std::abs(ssr)));
    }
  }
}

/// The axis extraction switches from the skew-symmetric part of R to its symmetric part at 120
/// degrees. Both are well conditioned across a wide overlap, so the switch must be invisible in the
/// result: this sweeps 110 to 130 degrees in half-degree steps and holds every step to the oracle,
/// so a wrong column, a mis-transposed symmetric part or a misplaced threshold surfaces as a step
/// out of line with its neighbours. Away from this range the symmetric branch runs only near a half
/// turn, where cot(phi/2) is ~0 and kills the term the axis feeds.
///
/// The second geometry's aabb_center is displaced from its own origin, and that displacement is
/// what puts the extracted axis's *sign* under test. The screw-axis point is built from t_perp and
/// k x t_perp, which are orthogonal and of sign-independent magnitude, so negating k leaves its
/// norm unchanged; a shape whose aabb_center lies on the screw axis measures the same distance
/// either way and the sign cancels identically. Only an off-axis aabb_center introduces the cross
/// term that separates the two, which is why a sphere alone cannot cover the sign recovery.
TEST(CoalDArcCompensationUnit, HandoffIsContinuousAcrossTheBranch)  // NOLINT
{
  const Eigen::Vector3d offset(0.5, 0.0, 0.0);
  const std::vector<Eigen::Vector3d> axes = {
    Eigen::Vector3d::UnitZ(),
    Eigen::Vector3d(0.3, -0.7, 1.0),
    Eigen::Vector3d(1.0, 2.0, 3.0),
  };

  // A cube of half-extent 0.1 built around (0.3, 0.1, -0.2) rather than its own origin, so the
  // convex hull coal derives from it carries an aabb_center well away from zero.
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  const Eigen::Vector3d hull_center(0.3, 0.1, -0.2);
  for (int i = 0; i < 8; ++i)
    vertices->emplace_back(hull_center + Eigen::Vector3d(((i & 1) != 0) ? 0.1 : -0.1,
                                                         ((i & 2) != 0) ? 0.1 : -0.1,
                                                         ((i & 4) != 0) ? 0.1 : -0.1));

  // Six quads, each written as [vertex count, indices...].
  auto faces = std::make_shared<Eigen::VectorXi>(30);
  (*faces) << 4, 0, 1, 3, 2,  // -z
      4, 4, 6, 7, 5,          // +z
      4, 0, 4, 5, 1,          // -y
      4, 2, 3, 7, 6,          // +y
      4, 0, 2, 6, 4,          // -x
      4, 1, 5, 7, 3;          // +x

  struct Case
  {
    CollisionShapeConstPtr shape;
    std::string label;
    double min_aabb_center_norm;  ///< 0 where the shape is its own AABB's centre.
  };
  const std::vector<Case> cases = {
    { std::make_shared<tesseract::geometry::Sphere>(0.05), "sphere", 0.0 },
    { std::make_shared<tesseract::geometry::ConvexMesh>(vertices, faces, 6), "off_origin_hull", 0.1 },
  };

  for (const auto& test_case : cases)
  {
    for (const auto& axis : axes)
    {
      for (int i = 0; i <= 40; ++i)
      {
        // 110 to 130 degrees in half-degree steps, straddling the 120-degree handoff.
        const double phi = (110.0 + (0.5 * i)) * M_PI / 180.0;

        Eigen::Isometry3d shape_offset = Eigen::Isometry3d::Identity();
        shape_offset.translation() = offset;
        auto checker = makeCastSetup(true, test_case.shape, shape_offset);

        Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
        pose2.rotate(Eigen::AngleAxisd(phi, axis.normalized()));
        checker->setCollisionObjectsTransform("link_a", pose1, pose2);

        auto* cast_shape = getCastHullShape(*checker, "link_a");
        ASSERT_NE(cast_shape, nullptr);
        const double ssr = cast_shape->getSweptSphereRadius();
        const auto& underlying = cast_shape->getUnderlyingShape();
        const double expected = referenceDArc(pose1, pose2, offset, underlying->aabb_center, underlying->aabb_radius);

        SCOPED_TRACE("geometry=" + test_case.label + " phi_deg=" + std::to_string(110.0 + (0.5 * i)));

        // Read off the shape rather than assumed: should a future coal release centre a convex
        // hull's AABB on its own origin, the sign recovery would silently stop reaching the result
        // and this sweep would go back to covering only the axis direction. Only the hull case
        // carries a nonzero floor here -- the sphere's is always 0.0, which norm() >= 0.0 cannot fail.
        if (test_case.min_aabb_center_norm > 0.0)
        {
          ASSERT_GE(underlying->aabb_center.norm(), test_case.min_aabb_center_norm);
        }

        EXPECT_NEAR(ssr, expected, 1e-10 * std::max(1.0, std::abs(expected)));
      }
    }
  }
}

/// An octree link's voxel boxes are created by the cast expansion, never by the shape cache, so the
/// expansion is the only thing that can bound them. d_arc reads that bound off the wrapped shape; an
/// unbounded box carries coal's default sentinel centre and a negative radius, which makes d_arc NaN
/// and poisons the cast AABB -- and a NaN AABB fails every broadphase overlap test, so the pairs are
/// dropped with no contact and no error.
TEST(CoalDArcCompensationUnit, OctreeVoxelBoxesAreBounded)  // NOLINT
{
  CoalCastBVHManager checker("test", true);
  test_suite::detail::addOctree(checker, "octomap_link");
  checker.setActiveCollisionObjects({ "octomap_link" });
  checker.setDefaultCollisionMargin(0.0);

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.rotate(Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()));
  checker.setCollisionObjectsTransform("octomap_link", pose1, pose2);

  auto* cast_shape = getCastHullShape(checker, "octomap_link");
  ASSERT_NE(cast_shape, nullptr);

  const auto& voxel_box = cast_shape->getUnderlyingShape();
  ASSERT_NE(voxel_box, nullptr);
  EXPECT_GT(voxel_box->aabb_radius, 0.0);

  const double ssr = cast_shape->getSweptSphereRadius();
  EXPECT_TRUE(std::isfinite(ssr)) << "d_arc came out " << ssr;
  EXPECT_GT(ssr, 0.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
