#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <atomic>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>
#include <tesseract/collision/common.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/impl/signed_distance_field_utils.h>

using namespace tesseract::collision;
using namespace tesseract_collision_coal;

namespace
{
/** @brief f(p) = ||p|| - 0.5 sampled over [-1, 1]^3, the field the tesseract SDF tests use. */
CollisionShapePtr makeSphereSDF()
{
  const tesseract::geometry::SignedDistanceFunction sphere = [](const Eigen::Vector3d& p) { return p.norm() - 0.5; };
  return tesseract::geometry::createDiscreteSignedDistanceField(
      sphere, Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1), Eigen::Vector3i(16, 16, 16));
}

/** @brief A 0.2 cube as a convex mesh, which has no analytic implicit form. */
CollisionShapePtr makeConvexCube()
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  for (int i = 0; i < 8; ++i)
    vertices->emplace_back(((i & 1) != 0) ? 0.1 : -0.1, ((i & 2) != 0) ? 0.1 : -0.1, ((i & 4) != 0) ? 0.1 : -0.1);
  auto faces = std::make_shared<Eigen::VectorXi>(30);
  *faces << 4, 0, 2, 3, 1, 4, 4, 5, 7, 6, 4, 0, 1, 5, 4, 4, 2, 6, 7, 3, 4, 0, 4, 6, 2, 4, 1, 3, 7, 5;
  return std::make_shared<tesseract::geometry::ConvexMesh>(vertices, faces);
}

ContactResultVector closest(DiscreteContactManager& checker)
{
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);
  return result_vector;
}

/** @brief Probe the sphere field along +x with @p probe, whose half-width along x is 0.1. */
void runProbeTest(const CollisionShapeConstPtr& probe, const CollisionShapeConstPtr& sdf = makeSphereSDF())
{
  CoalDiscreteBVHManager checker;
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("sdf_link", 0, { sdf }, poses, true);
  checker.addCollisionObject("probe_link", 0, { probe }, poses, true);
  checker.setActiveCollisionObjects({ "sdf_link", "probe_link" });
  checker.setDefaultCollisionMargin(0.05);

  // Probe centre at 0.55: its near side reaches 0.45, 0.05 inside the surface at 0.5.
  checker.setCollisionObjectsTransform("probe_link", Eigen::Isometry3d(Eigen::Translation3d(0.55, 0, 0)));
  ContactResultVector contacts = closest(checker);
  ASSERT_EQ(contacts.size(), 1);
  EXPECT_NEAR(contacts[0].distance, -0.05, 0.02);
  EXPECT_NEAR(contacts[0].normal.norm(), 1.0, 1e-3);
  EXPECT_NEAR(std::abs(contacts[0].normal.x()), 1.0, 0.05);
  EXPECT_EQ(contacts[0].subshape_id[0], -1);
  EXPECT_EQ(contacts[0].subshape_id[1], -1);

  // Separated by 0.03, inside the 0.05 margin.
  checker.setCollisionObjectsTransform("probe_link", Eigen::Isometry3d(Eigen::Translation3d(0.63, 0, 0)));
  contacts = closest(checker);
  ASSERT_EQ(contacts.size(), 1);
  EXPECT_NEAR(contacts[0].distance, 0.03, 0.02);

  // Separated by 0.15, outside the margin.
  checker.setCollisionObjectsTransform("probe_link", Eigen::Isometry3d(Eigen::Translation3d(0.75, 0, 0)));
  EXPECT_TRUE(closest(checker).empty());
}
}  // namespace

TEST(CoalSignedDistanceFieldUnit, Sphere)  // NOLINT
{
  runProbeTest(std::make_shared<tesseract::geometry::Sphere>(0.1));
}

TEST(CoalSignedDistanceFieldUnit, Box)  // NOLINT
{
  runProbeTest(std::make_shared<tesseract::geometry::Box>(0.2, 0.2, 0.2));
}

TEST(CoalSignedDistanceFieldUnit, ConvexMesh)  // NOLINT
{
  runProbeTest(makeConvexCube());
}

TEST(CoalSignedDistanceFieldUnit, SDFPair)  // NOLINT
{
  // A second sphere field is a probe of half-width 0.5, so offset the probe positions by 0.4.
  CoalDiscreteBVHManager checker;
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("sdf0_link", 0, { makeSphereSDF() }, poses, true);
  checker.addCollisionObject("sdf1_link", 0, { makeSphereSDF() }, poses, true);
  checker.setActiveCollisionObjects({ "sdf0_link", "sdf1_link" });
  checker.setDefaultCollisionMargin(0.05);

  checker.setCollisionObjectsTransform("sdf1_link", Eigen::Isometry3d(Eigen::Translation3d(0.8, 0, 0)));
  ContactResultVector contacts = closest(checker);
  ASSERT_EQ(contacts.size(), 1);
  EXPECT_NEAR(contacts[0].distance, -0.2, 0.02);
  EXPECT_NEAR((contacts[0].nearest_points[1] - contacts[0].nearest_points[0]).dot(contacts[0].normal),
              contacts[0].distance,
              1e-3);

  checker.setCollisionObjectsTransform("sdf1_link", Eigen::Isometry3d(Eigen::Translation3d(1.03, 0, 0)));
  contacts = closest(checker);
  ASSERT_EQ(contacts.size(), 1);
  EXPECT_NEAR(contacts[0].distance, 0.03, 0.02);

  checker.setCollisionObjectsTransform("sdf1_link", Eigen::Isometry3d(Eigen::Translation3d(1.1, 0, 0)));
  EXPECT_TRUE(closest(checker).empty());
}

TEST(CoalSignedDistanceFieldUnit, FunctionBackedFieldIsNotDiscretized)  // NOLINT
{
  auto calls = std::make_shared<std::atomic<int>>(0);
  const tesseract::geometry::BatchedSignedDistanceFunction sampler =
      [calls](const std::vector<Eigen::Vector3d>& points) {
        ++(*calls);
        std::vector<double> distances;
        distances.reserve(points.size());
        for (const auto& p : points)
          distances.push_back(p.norm() - 0.5);
        return distances;
      };
  auto sdf = std::make_shared<tesseract::geometry::SignedDistanceField>(
      Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-1), Eigen::Vector3d::Constant(1)),
      Eigen::Vector3i(16, 16, 16),
      sampler);

  runProbeTest(std::make_shared<tesseract::geometry::Sphere>(0.1), sdf);

  EXPECT_GT(calls->load(), 0);
  EXPECT_FALSE(sdf->isDiscretized());
}

TEST(CoalSignedDistanceFieldUnit, RejectsUnsupportedPair)  // NOLINT
{
  CoalDiscreteBVHManager checker;
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("sdf_link", 0, { makeSphereSDF() }, poses, true);
  checker.addCollisionObject(
      "plane_link", 0, { std::make_shared<tesseract::geometry::Plane>(0.0, 0.0, 1.0, 0.0) }, poses, true);
  checker.setActiveCollisionObjects({ "sdf_link", "plane_link" });

  ContactResultMap result;
  EXPECT_THROW(checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST)), std::runtime_error);  // NOLINT
}

TEST(CoalSignedDistanceFieldUnit, RejectsNonuniformScale)  // NOLINT
{
  CoalDiscreteBVHManager checker;
  auto sdf = std::make_shared<tesseract::geometry::SignedDistanceField>(
      Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-1), Eigen::Vector3d::Constant(1)),
      Eigen::Vector3i(2, 2, 2),
      std::vector<double>(8, 0.0),
      Eigen::Vector3d(1.0, 2.0, 1.0));
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  EXPECT_THROW(checker.addCollisionObject("sdf_link", 0, { sdf }, poses, true), std::invalid_argument);  // NOLINT
}

TEST(CoalSignedDistanceFieldUnit, CastAgainstStaticField)  // NOLINT
{
  // A 0.1 sphere sweeps along y at x = 0.55, from y = -0.5 to y = 0.5. Both end poses clear the field by 0.14,
  // so only the sweep finds the contact: 0.05 deep at mid-motion.
  CoalCastBVHManager checker;
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("sdf_link", 0, { makeSphereSDF() }, poses, true);
  checker.addCollisionObject("probe_link", 0, { std::make_shared<tesseract::geometry::Sphere>(0.1) }, poses, true);
  checker.setActiveCollisionObjects({ "probe_link" });
  checker.setDefaultCollisionMargin(0.0);

  checker.setCollisionObjectsTransform("sdf_link", Eigen::Isometry3d::Identity());
  checker.setCollisionObjectsTransform("probe_link",
                                       Eigen::Isometry3d(Eigen::Translation3d(0.55, -0.5, 0)),
                                       Eigen::Isometry3d(Eigen::Translation3d(0.55, 0.5, 0)));

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector contacts;
  result.flattenMoveResults(contacts);

  ASSERT_EQ(contacts.size(), 1);
  EXPECT_NEAR(contacts[0].distance, -0.05, 0.02);
  EXPECT_NEAR(std::abs(contacts[0].normal.x()), 1.0, 0.05);

  const std::size_t probe = (contacts[0].link_ids[0] == "probe_link") ? 0 : 1;
  EXPECT_EQ(contacts[0].cc_type[probe], ContinuousCollisionType::CCType_Between);
  EXPECT_NEAR(contacts[0].cc_time[probe], 0.5, 0.05);
  EXPECT_EQ(contacts[0].cc_type[1 - probe], ContinuousCollisionType::CCType_None);

  // Head-on, the probe is deepest at the end of the motion.
  checker.setCollisionObjectsTransform("probe_link",
                                       Eigen::Isometry3d(Eigen::Translation3d(0.9, 0, 0)),
                                       Eigen::Isometry3d(Eigen::Translation3d(0.55, 0, 0)));
  result.clear();
  contacts.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(contacts);
  ASSERT_EQ(contacts.size(), 1);
  EXPECT_NEAR(contacts[0].distance, -0.05, 0.02);
  EXPECT_EQ(contacts[0].cc_type[probe], ContinuousCollisionType::CCType_Time1);

  // Moved clear of the field, the sweep finds nothing.
  checker.setCollisionObjectsTransform("probe_link",
                                       Eigen::Isometry3d(Eigen::Translation3d(0.75, -0.5, 0)),
                                       Eigen::Isometry3d(Eigen::Translation3d(0.75, 0.5, 0)));
  result.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  EXPECT_TRUE(result.empty());
}

TEST(CoalSignedDistanceFieldUnit, CastDArcCompensation)  // NOLINT
{
  // A 0.2 cube centred at x = 0.62 turns 90 degrees about z. Both end poses keep its near face at 0.52, 0.02 clear of
  // the field, so the hull of the two poses never reaches it. Mid-turn the corners (radius 0.1414) swing in to 0.4786;
  // d_arc inflates the hull by that sagitta, 0.1414 * (1 - cos 45deg) = 0.0414, giving a contact 0.021 deep.
  const auto run = [](bool d_arc_compensation) {
    CoalCastBVHManager checker("test", d_arc_compensation);
    tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
    checker.addCollisionObject("sdf_link", 0, { makeSphereSDF() }, poses, true);
    checker.addCollisionObject(
        "probe_link", 0, { std::make_shared<tesseract::geometry::Box>(0.2, 0.2, 0.2) }, poses, true);
    checker.setActiveCollisionObjects({ "probe_link" });
    checker.setDefaultCollisionMargin(0.0);
    checker.setCollisionObjectsTransform("sdf_link", Eigen::Isometry3d::Identity());
    const Eigen::Isometry3d start(Eigen::Translation3d(0.62, 0, 0));
    checker.setCollisionObjectsTransform(
        "probe_link", start, start * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));

    ContactResultMap result;
    checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
    ContactResultVector contacts;
    result.flattenMoveResults(contacts);
    return contacts;
  };

  EXPECT_TRUE(run(false).empty());

  const ContactResultVector contacts = run(true);
  ASSERT_EQ(contacts.size(), 1);
  EXPECT_NEAR(contacts[0].distance, -0.021, 0.01);
}

TEST(CoalSignedDistanceFieldUnit, CastRejectsKinematicField)  // NOLINT
{
  CoalCastBVHManager checker;
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("sdf_link", 0, { makeSphereSDF() }, poses, true);
  EXPECT_THROW(checker.setActiveCollisionObjects({ "sdf_link" }), std::runtime_error);  // NOLINT
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
