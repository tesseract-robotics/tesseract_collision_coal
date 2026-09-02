#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>
#include <tesseract/collision/common.h>
#include <tesseract/collision/test_suite/collision_link_fixtures.hpp>

using namespace tesseract::collision;
using namespace tesseract_collision_coal;

/**
 * @brief The configured collision margin applies to objects added after it was configured.
 *
 * The margin grows each collision object's broadphase AABB. An object whose AABB was never grown is
 * culled before narrowphase runs, so a near-contact inside the margin is absent from the report rather
 * than mis-measured. Callers that configure a margin and then populate the manager are the subject: the
 * paths that populate first and configure second are repaired by the margin call itself.
 */
namespace
{
/** @brief Add a unit-diameter sphere at @p x on the x axis. */
template <typename ContactManager>
void addSphere(ContactManager& checker, const std::string& link, double x)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, 0, 0);
  test_suite::detail::addSphereLink(checker, link, 0.5, pose);
}
}  // namespace

TEST(CoalMarginOnAddUnit, DiscreteMarginAppliesToObjectAddedAfterwards)  // NOLINT
{
  CoalDiscreteBVHManager checker;

  // Configure first, populate second. The two spheres are 0.05 m apart surface to surface, inside the
  // 0.1 m margin and outside a zero margin.
  checker.setDefaultCollisionMargin(0.1);
  addSphere(checker, "sphere_a", 0.0);
  addSphere(checker, "sphere_b", 1.05);
  checker.setActiveCollisionObjects({ "sphere_a" });

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));

  ASSERT_FALSE(result.empty());
  auto it = result.find(tesseract::common::LinkIdPair("sphere_a", "sphere_b"));
  ASSERT_NE(it, result.end());
  ASSERT_FALSE(it->second.empty());
  EXPECT_NEAR(it->second.front().distance, 0.05, 1e-6);
}

TEST(CoalMarginOnAddUnit, DiscreteZeroMarginStillCullsTheSameGap)  // NOLINT
{
  // The control. Same geometry, no margin: the gap must NOT be reported, or the test above would pass
  // for a reason that has nothing to do with the margin.
  CoalDiscreteBVHManager checker;

  checker.setDefaultCollisionMargin(0.0);
  addSphere(checker, "sphere_a", 0.0);
  addSphere(checker, "sphere_b", 1.05);
  checker.setActiveCollisionObjects({ "sphere_a" });

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));

  EXPECT_TRUE(result.empty());
}

TEST(CoalMarginOnAddUnit, CastMarginAppliesToObjectAddedAfterwards)  // NOLINT
{
  CoalCastBVHManager checker;

  checker.setDefaultCollisionMargin(0.1);
  addSphere(checker, "sphere_a", 0.0);
  addSphere(checker, "sphere_b", 1.05);
  checker.setActiveCollisionObjects({ "sphere_a" });

  // A degenerate sweep: start and end coincide, so the swept volume is the sphere itself and the only
  // thing that can bring the pair together is the margin.
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform("sphere_a", pose, pose);

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));

  ASSERT_FALSE(result.empty());
  auto it = result.find(tesseract::common::LinkIdPair("sphere_a", "sphere_b"));
  ASSERT_NE(it, result.end());
  ASSERT_FALSE(it->second.empty());
  EXPECT_NEAR(it->second.front().distance, 0.05, 1e-6);
}

TEST(CoalMarginOnAddUnit, CastZeroMarginStillCullsTheSameGap)  // NOLINT
{
  CoalCastBVHManager checker;

  checker.setDefaultCollisionMargin(0.0);
  addSphere(checker, "sphere_a", 0.0);
  addSphere(checker, "sphere_b", 1.05);
  checker.setActiveCollisionObjects({ "sphere_a" });

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform("sphere_a", pose, pose);

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));

  EXPECT_TRUE(result.empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
